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
  param_.isam_extra_updates =
      config.param<int>("backend_optimizer", "isam_extra_updates", 5);
  param_.min_loop_frame_gap =
      config.param<int>("backend_optimizer", "min_loop_frame_gap", 30);
  param_.icp_search_num =
      config.param<int>("backend_optimizer", "icp_search_num", 3);
}

std::map<char, AgentOptimizedData> BackendOptimizerIncremental::Process(
    const AgentContext&                 ctx,
    const AgentRawData&                 raw_data,
    const LoopPairVec&                  intra_loops,
    const LoopPairVec&                  inter_loops,
    const std::map<char, AgentRawData>& all_raw_data) {

  gtsam::Pose3 anchor_node = gtsam::Pose3(Eigen::Matrix4d::Identity());
  const auto& anchor_prior = ctx.is_anchor() ? prior_noise_ : large_noise_;

  //! 1. anchor prior
  gtsam::Symbol anchor_symbol(ctx.id, ANCHOR_IDX);
  accumulated_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
      anchor_symbol, anchor_node, anchor_prior));
  accumulated_values_.insert(anchor_symbol, anchor_node);

  //! 2. odometry
  for (size_t i = 0; i < raw_data.odom_poses.size(); i++) {
    gtsam::Symbol node_current(ctx.id, i);
    accumulated_values_.insert(node_current,
                               gtsam::Pose3(raw_data.odom_poses[i].matrix()));
    if (i == 0) {
      accumulated_graph_.add(gtsam::PriorFactor<gtsam::Pose3>(
          node_current, gtsam::Pose3(raw_data.odom_poses[i].matrix()), anchor_prior));
    } else {
      gtsam::Symbol node_prev(ctx.id, i - 1);
      Eigen::Isometry3d rel = raw_data.odom_poses[i - 1].inverse() * raw_data.odom_poses[i];
      accumulated_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          node_prev, node_current, gtsam::Pose3(rel.matrix()), odometry_noise_));
    }
  }

  //! 3. intra-loop
  auto T1 = tq::tqdm(intra_loops);
  T1.set_prefix("Intra Backend Optimizer");
  for (auto loop : T1) {
    if (loop.from.second - loop.to.second <
        static_cast<size_t>(param_.min_loop_frame_gap)) continue;
    gtsam::Symbol node_from(loop.from.first, loop.from.second);
    gtsam::Symbol node_to(loop.to.first,   loop.to.second);
    auto refined = registerPointCloud(
        raw_data.filtered_scans, raw_data.odom_poses,
        raw_data.filtered_scans[loop.from.second], loop, param_.icp_search_num);
    if (refined) {
      accumulated_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
          node_from, node_to, gtsam::Pose3(refined.value().matrix()),
          robust_loop_noise_));
    }
  }
  T1.finish();

  //! 4. inter-loop (follower만)
  if (!ctx.is_anchor()) {
    auto T2 = tq::tqdm(inter_loops);
    T2.set_prefix("Inter Backend Optimizer");
    for (auto loop : T2) {
      gtsam::Symbol node_from(loop.from.first, loop.from.second);
      gtsam::Symbol node_to(loop.to.first,   loop.to.second);
      const auto& raw_to = all_raw_data.at(loop.to.first);
      auto refined = registerPointCloud(
          raw_to.filtered_scans, raw_to.odom_poses,
          raw_data.filtered_scans[loop.from.second], loop, param_.icp_search_num);
      if (refined) {
        // TODO(gil) : use BetweenFactorWithAnchoring?
        accumulated_graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(
            node_from, node_to, gtsam::Pose3(refined.value().matrix()),
            robust_loop_noise_));
      }
    }
    T2.finish();
  }

  //! 5. ISAM2 최적화 (누적된 전체 그래프로 배치 최적화)
  // 진짜 증분화는 BackendOptimizer를 MapServer 레벨에서 공유할 때 가능 (Step 05 이후)
  gtsam::ISAM2Params isam_param;
  isam_param.relinearizeThreshold = param_.relinearize_threshold;
  isam_param.relinearizeSkip      = param_.relinearize_skip;
  gtsam::ISAM2 isam_(isam_param);
  isam_.update(accumulated_graph_, accumulated_values_);
  for (int i = 0; i < param_.isam_extra_updates; ++i) {
    isam_.update();
  }
  accumulated_values_ = isam_.calculateBestEstimate();

  //! 6. 모든 에이전트 포즈 추출
  std::map<char, std::vector<std::pair<int, Eigen::Isometry3d>>> all_poses;
  for (const auto& key_value : accumulated_values_) {
    gtsam::Key key = key_value.key;
    gtsam::Symbol symbol(key);
    if (symbol.index() == ANCHOR_IDX) continue;

    char agent_chr = symbol.chr();
    gtsam::Symbol anchor_sym(agent_chr, ANCHOR_IDX);
    if (!accumulated_values_.exists(anchor_sym)) continue;

    Eigen::Matrix4d anchor_pose =
        accumulated_values_.at<gtsam::Pose3>(anchor_sym).matrix();
    Eigen::Matrix4d pose = accumulated_values_.at<gtsam::Pose3>(key).matrix();
    Eigen::Isometry3d global_pose(anchor_pose * pose);
    all_poses[agent_chr].push_back({static_cast<int>(symbol.index()), global_pose});
  }

  std::map<char, AgentOptimizedData> all_results;
  for (auto& [chr, poses] : all_poses) {
    std::sort(poses.begin(), poses.end(),
              [](const auto& a, const auto& b) { return a.first < b.first; });
    AgentOptimizedData opt;
    opt.agent_id = chr;
    opt.optimized_poses = std::move(poses);
    for (const auto& [idx, p] : opt.optimized_poses) {
      opt.kdtree_poses.push_back(pcl::PointXYZ(p.translation().x(),
                                                p.translation().y(),
                                                p.translation().z()));
    }
    all_results[chr] = std::move(opt);
  }
  return all_results;
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