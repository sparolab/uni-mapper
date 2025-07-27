#include "backend_optimizer_incremental.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GncOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <open_lmm/common/registration.hpp>

#include "BetweenFactorWithAnchoring.h"

namespace open_lmm {

BackendOptimizerIncremental::BackendOptimizerIncremental(Config config) {
  parseConfig(config);
  initNoise();
}

BackendOptimizerIncremental::~BackendOptimizerIncremental() {}

void BackendOptimizerIncremental::parseConfig(Config config) {
  param_.relinearize_threshold =
      config.param<double>("backend_optimizer", "relinearizeThreshold", 0.1);
  param_.relinearize_skip =
      config.param<int>("backend_optimizer", "relinearizeSkip", 1);
}

std::vector<std::pair<int, Eigen::Isometry3d>>
BackendOptimizerIncremental::process(
    std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
    PoseVec poses, LoopPairVec intra_loops, LoopPairVec inter_loops) {
  gtsam::Pose3 anchor_node = gtsam::Pose3(Eigen::Matrix4d::Identity());

  //! 1. initialize graph(anchor + prior + btw)
  gtsam::Symbol anchor_symbol(agent_id, ANCHOR_IDX);
  if (agent_id == 'A') {
    shared_data->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        anchor_symbol, anchor_node, prior_noise_));
  } else {
    shared_data->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
        anchor_symbol, anchor_node, large_noise_));
  }
  shared_data->values.insert(anchor_symbol, anchor_node);

  // TODO(gil) : refactor
  //! 2. add odometry
  for (size_t i = 0; i < poses.size(); i++) {
    gtsam::Symbol node_current(agent_id, i);
    gtsam::Symbol node_prev(agent_id, i - 1);
    if (i == 0) {
      shared_data->values.insert(node_current, gtsam::Pose3(poses[i].matrix()));
      if (agent_id == 'A') {
        shared_data->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            node_current, gtsam::Pose3(poses[i].matrix()), prior_noise_));
      } else {
        shared_data->graph.add(gtsam::PriorFactor<gtsam::Pose3>(
            node_current, gtsam::Pose3(poses[i].matrix()), large_noise_));
      }

    } else {
      shared_data->values.insert(node_current, gtsam::Pose3(poses[i].matrix()));
      Eigen::Isometry3d relative_pose = poses[i - 1].inverse() * poses[i];
      shared_data->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          node_prev, node_current, gtsam::Pose3(relative_pose.matrix()),
          odometry_noise_));
    }
  }

  //! 3. add intra-agent loop
  auto T1 = tq::tqdm(intra_loops);
  T1.set_prefix("Intra Backend Optimizer");
  for (auto loop : T1) {
    if (loop.from.second - loop.to.second < 30) continue;
    gtsam::Symbol node_from(loop.from.first, loop.from.second);
    gtsam::Symbol node_to(loop.to.first, loop.to.second);
    std::optional<Eigen::Isometry3d> refined_pose =
        registerPointCloud(*shared_data, loop, 3);
    if (refined_pose) {
      Eigen::Matrix4d refined_pose_mat = refined_pose.value().matrix();
      shared_data->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
          node_from, node_to, gtsam::Pose3(refined_pose_mat),
          robust_loop_noise_));
    }
  }
  T1.finish();

  //! 4. add inter-agent loop
  if (agent_id != 'A') {
    auto T2 = tq::tqdm(inter_loops);
    T2.set_prefix("Inter Backend Optimizer");
    for (auto loop : T2) {
      gtsam::Symbol node_from(loop.from.first, loop.from.second);
      gtsam::Symbol node_to(loop.to.first, loop.to.second);
      std::optional<Eigen::Isometry3d> refined_pose =
          registerPointCloud(*shared_data, loop, 3);
      if (refined_pose) {
        Eigen::Matrix4d refined_pose_mat = refined_pose.value().matrix();
        // TODO(gil) : use BetweenFactorWithAnchoring?
        gtsam::Symbol anchor_symbol_to(loop.to.first, ANCHOR_IDX);
        shared_data->graph.add(gtsam::BetweenFactor<gtsam::Pose3>(
            node_from, node_to, gtsam::Pose3(refined_pose_mat),
            robust_loop_noise_));
        // shared_data->graph.add(gtsam::BetweenFactorWithAnchoring<gtsam::Pose3>(
        //   node_to, node_from,
        //   anchor_symbol_to, anchor_symbol,
        //   gtsam::Pose3(refined_pose_mat), robust_loop_noise_)
        // );
      }
    }
    T2.finish();
  }

  gtsam::ISAM2Params isam_param;
  isam_param.relinearizeThreshold = param_.relinearize_threshold;
  isam_param.relinearizeSkip = param_.relinearize_skip;
  gtsam::ISAM2 isam_(isam_param);
  isam_.update(shared_data->graph, shared_data->values);
  isam_.update();
  isam_.update();
  isam_.update();
  isam_.update();
  isam_.update();
  shared_data->values = isam_.calculateBestEstimate();

  for (const auto& key_value : shared_data->values) {
    gtsam::Key key = key_value.key;
    gtsam::Symbol symbol(key);
    gtsam::Symbol anchor_symbol(symbol.chr(), ANCHOR_IDX);
    // TODO(gil) : use anchor factor?
    Eigen::Matrix4d anchor_pose =
        shared_data->values.at<gtsam::Pose3>(anchor_symbol).matrix();
    Eigen::Matrix4d pose = shared_data->values.at<gtsam::Pose3>(key).matrix();
    Eigen::Isometry3d global_pose(anchor_pose * pose);
    // TODO(gil) : must refactored
    if (symbol.index() != ANCHOR_IDX) {
      if (symbol.chr() == agent_id) {
        shared_data->db_optimized_poses[symbol.chr()].push_back(
            std::make_pair(symbol.index(), global_pose));
        shared_data->db_kdtree_poses[symbol.chr()].push_back(pcl::PointXYZ(
            global_pose.translation().x(), global_pose.translation().y(),
            global_pose.translation().z()));
      } else {
        shared_data->db_optimized_poses[symbol.chr()].at(symbol.index()) =
            (std::make_pair(symbol.index(), global_pose));
        shared_data->db_kdtree_poses[symbol.chr()].at(symbol.index()) =
            pcl::PointXYZ(global_pose.translation().x(),
                          global_pose.translation().y(),
                          global_pose.translation().z());
      }
    }
  }
  auto optimized_poses = shared_data->db_optimized_poses[agent_id];

  return optimized_poses;
}

// TODO(gil) : add noise parameter config and parsing function
void BackendOptimizerIncremental::initNoise() {
  prior_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12)
          .finished());

  odometry_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());

  gtsam::Vector robust_noise_vec(6);
  robust_noise_vec << 1e-1, 1e-1, 1e-1, 1e-1, 1e-1, 1e-1;
  robust_loop_noise_ = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Cauchy::Create(1),
      gtsam::noiseModel::Diagonal::Variances(robust_noise_vec));

  large_noise_ = gtsam::noiseModel::Diagonal::Variances(
      (gtsam::Vector(6) << M_PI * M_PI, M_PI * M_PI, M_PI * M_PI, 1e8, 1e8, 1e8)
          .finished());
}

}  // namespace open_lmm