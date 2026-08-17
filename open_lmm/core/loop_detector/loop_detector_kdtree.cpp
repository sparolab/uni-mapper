#include "loop_detector_kdtree.hpp"

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <open_lmm/common/pointcloud_utils.hpp>

namespace open_lmm {

KdtreeParams::KdtreeParams() {
  Config config =
      Config(GlobalConfig::get_global_config_path("config_loop_detector"));
  num_candidates = config.param<int>("database", "num_candidates", 5);
  distance_threshold =
      config.param<double>("database", "distance_threshold", 0.13);
  kdtree_rebuild_threshold =
      config.param<int>("database", "rebuild_threshold", 50);
  model = config.param<std::string>("loop_detector", "model", "");
}

LoopDetectorKdtree::LoopDetectorKdtree(const KdtreeParams& params)
    : params_(params) {
  std::string so_model_name = "libcreate_" + params_.model + ".so";
  model_descriptor_ = loadModule(so_model_name);
  database_ = DatabaseKdtree();
}

LoopPair LoopDetectorKdtree::createLoopPair(
    char agent_id, size_t current_idx,
    const LoopCandidateInfo& candidate_info) {
  LoopPair loop;
  auto [db_id, key, init_rel_pose] = candidate_info;
  loop.to = std::make_pair(db_id, key);
  loop.from = std::make_pair(agent_id, current_idx);
  loop.init_rel_pose = init_rel_pose;
  return loop;
}

std::vector<LoopPair> LoopDetectorKdtree::detectIntraLoops(
    const ScanVec& scans, const AgentContext& agent_ctx) {
  std::vector<LoopPair> intra_loop_pairs;
  int total_scans = scans.size();
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Intra Loop Detector");
  for (auto idx : T) {
    auto scan = scans[idx];
    auto descriptor = model_descriptor_->makeDescriptor(scan);
    std::optional<LoopCandidateInfo> intra_loop_candidates =
        database_->query(descriptor);

    if (intra_loop_candidates != std::nullopt) {
      intra_loop_pairs.push_back(
          createLoopPair(agent_ctx.id, idx, intra_loop_candidates.value()));
    }

    database_->insert(agent_ctx.id, idx, descriptor);
  }
  T.finish();
  return intra_loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::detectInterLoops(
    const ScanVec& scans, const DescriptorStore& descriptor_store,
    const AgentContext& agent_ctx) {
  std::vector<LoopPair> inter_loop_pairs;

  if (agent_ctx.is_anchor()) {
    return inter_loop_pairs;
  }

  int total_scans = scans.size();
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Inter Loop Detector");
  for (auto idx : T) {
    auto scan = scans[idx];
    auto descriptor = model_descriptor_->makeDescriptor(scan);

    std::optional<LoopCandidateInfo> inter_loop_candidates =
        descriptor_store.total_db.query(descriptor);

    if (inter_loop_candidates != std::nullopt) {
      inter_loop_pairs.push_back(
          createLoopPair(agent_ctx.id, idx, inter_loop_candidates.value()));
    }
  }
  T.finish();
  return inter_loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::findLoopPairsFromKdTree(
    const std::map<char, AgentOptimizedData>& all_optimized,
    const std::map<char, AgentRawData>& all_raw_data,
    const std::vector<Eigen::Isometry3f>& transformed_poses,
    const AgentContext& agent_ctx,
    float distance_threshold) {
  std::vector<LoopPair> loop_pairs;

  for (const auto& [db_id, opt_data] : all_optimized) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = opt_data.kdtree_poses;
    kdtree.setInputCloud(cloud);

    Eigen::Vector3f prev_pose = transformed_poses[0].translation();

    for (size_t idx = 0; idx < transformed_poses.size(); idx++) {
      auto pose = transformed_poses[idx];

      Eigen::Vector3f curr_pose = pose.translation();
      float distance = (curr_pose - prev_pose).norm();
      if (distance < 10.0F) {
        continue;
      } else {
        prev_pose = curr_pose;
      }

      std::vector<int> pointIdxNKNSearch(1);
      std::vector<float> pointNKNSquaredDistance(1);
      pcl::PointXYZ src_point(pose.translation().x(), pose.translation().y(),
                              pose.translation().z());
      kdtree.nearestKSearch(src_point, 1, pointIdxNKNSearch,
                            pointNKNSquaredDistance);

      if (pointIdxNKNSearch[0] != -1 &&
          std::sqrt(pointNKNSquaredDistance[0]) < distance_threshold) {
        auto key = pointIdxNKNSearch[0];
        auto init_rel_pose = pose.cast<double>().inverse() *
                             all_raw_data.at(db_id).odom_poses[key];

        LoopPair inter_loop;
        inter_loop.to = std::make_pair(db_id, static_cast<size_t>(key));
        inter_loop.from = std::make_pair(agent_ctx.id, idx);
        inter_loop.init_rel_pose = init_rel_pose;
        loop_pairs.push_back(inter_loop);
      }
    }
  }

  return loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::detectKissMatcherLoops(
    const LoopDetectorInput& input,
    std::vector<Eigen::Vector3f>& out_transformed_map_points) {
  std::vector<LoopPair> additional_loops;
  constexpr float kMapMatchingThreshold = 2.0f;
  constexpr float kDistanceThreshold = 10.0f;

  // anchor: 맵 포인트를 out으로 전달 (caller가 descriptor_store에 set_anchor 호출)
  if (input.agent_ctx.is_anchor()) {
    out_transformed_map_points = input.current.map_points;
    return additional_loops;
  }

  Eigen::Matrix4f relative_map_pose;
  if (!TryKissMatcher(input.descriptor_store.merged_map,
                      input.current.map_points,
                      kMapMatchingThreshold, false, relative_map_pose)) {
    return additional_loops;
  }

  auto transformed_poses = transformEigenPoses(
      input.current.odom_poses, relative_map_pose);

  additional_loops = findLoopPairsFromKdTree(
      input.all_optimized, input.all_raw_data,
      transformed_poses, input.agent_ctx, kDistanceThreshold);

  out_transformed_map_points = transformEigenPoints(
      input.current.map_points, relative_map_pose);

  return additional_loops;
}

LoopDetectorOutput LoopDetectorKdtree::Process(const LoopDetectorInput& input) {
  database_->setAgentId(input.agent_ctx.id);

  auto intra_loops = detectIntraLoops(input.current.filtered_scans, input.agent_ctx);
  auto inter_loops = detectInterLoops(input.current.filtered_scans,
                                      input.descriptor_store, input.agent_ctx);

  std::vector<Eigen::Vector3f> transformed_map_points;
  auto additional_loops = detectKissMatcherLoops(input, transformed_map_points);
  inter_loops.insert(inter_loops.end(), additional_loops.begin(),
                     additional_loops.end());

  return LoopDetectorOutput{
      .intra_loops             = std::move(intra_loops),
      .inter_loops             = std::move(inter_loops),
      .agent_db                = std::move(database_.value()),
      .transformed_map_points  = std::move(transformed_map_points),
  };
}

std::shared_ptr<IDescriptorKdtree> LoopDetectorKdtree::loadModule(
    const std::string& so_name) {
  return load_module_from_so<IDescriptorKdtree>(
      so_name, "create_descriptor_kdtree_module");
}

}  // namespace open_lmm