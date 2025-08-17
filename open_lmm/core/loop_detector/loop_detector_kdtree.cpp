#pragma once

#include "loop_detector_kdtree.hpp"

#include <open_lmm/common/pointcloud_utils.hpp>

#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

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

std::vector<LoopPair> LoopDetectorKdtree::detectIntraLoops(const ScanVec& scans,
                                                           char agent_id) {
  std::vector<LoopPair> intra_loop_pairs;
  int total_scans = scans.size();
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Intra Loop Detector");
  for (auto idx : T) {
    // for (size_t idx = 0; idx < scans.size(); ++idx) {
    auto scan = scans[idx];
    auto descriptor = model_descriptor_->makeDescriptor(scan);
    // Check for loop candidates in the database
    std::optional<LoopCandidateInfo> intra_loop_candidates =
        database_->query(descriptor);

    if (intra_loop_candidates != std::nullopt) {
      intra_loop_pairs.push_back(
          createLoopPair(agent_id, idx, intra_loop_candidates.value()));
    }

    // Add current scan to database
    database_->insert(agent_id, idx, descriptor);
  }
  T.finish();
  return intra_loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::detectInterLoops(
    const ScanVec& scans, std::shared_ptr<SharedDatabase>& shared_data,
    char agent_id) {
  std::vector<LoopPair> inter_loop_pairs;

  // No inter-loops for agent A
  if (agent_id == 'A') {
    return inter_loop_pairs;
  }

  int total_scans = scans.size();
  auto T = tq::trange(0, total_scans);
  T.set_prefix("Inter Loop Detector");
  for (auto idx : T) {
    // for (size_t idx = 0; idx < scans.size(); ++idx) {
    auto scan = scans[idx];
    auto descriptor = model_descriptor_->makeDescriptor(scan);

    // Check for loop candidates in the shared database
    std::optional<LoopCandidateInfo> inter_loop_candidates =
        shared_data->total_db_descriptors.query(descriptor);

    if (inter_loop_candidates != std::nullopt) {
      inter_loop_pairs.push_back(
          createLoopPair(agent_id, idx, inter_loop_candidates.value()));
    }
  }
  T.finish();
  return inter_loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::findLoopPairsFromKdTree(
    std::shared_ptr<SharedDatabase>& shared_data,
    const std::vector<Eigen::Isometry3f>& transformed_poses, char agent_id,
    float distance_threshold) {
  std::vector<LoopPair> loop_pairs;

  for (auto& pose_kdtree : shared_data->db_kdtree_poses) {
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    *cloud = pose_kdtree.second;
    kdtree.setInputCloud(cloud);

    Eigen::Vector3f prev_pose = transformed_poses[0].translation();

    for (size_t idx = 0; idx < transformed_poses.size(); idx++) {
      auto pose = transformed_poses[idx];

      Eigen::Vector3f curr_pose = pose.translation();
      float distance = (curr_pose - prev_pose).norm();
      // TODO(gil) : hardcoded distance
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
        LoopPair inter_loop;
        auto db_id = pose_kdtree.first;
        auto key = pointIdxNKNSearch[0];
        auto init_rel_pose = pose.cast<double>().inverse() *
                             shared_data->db_odom_poses[db_id][key];

        inter_loop.to = std::make_pair(db_id, key);
        inter_loop.from = std::make_pair(agent_id, idx);
        inter_loop.init_rel_pose = init_rel_pose;
        loop_pairs.push_back(inter_loop);
      }
    }
  }

  return loop_pairs;
}

std::vector<LoopPair> LoopDetectorKdtree::detectKissMatcherLoops(
    std::shared_ptr<SharedDatabase>& shared_data, char agent_id) {
  std::vector<LoopPair> additional_loops;
  constexpr float kMapMatchingThreshold = 2.0f;
  constexpr float kDistanceThreshold = 10.0f;

  if (agent_id == 'A') {
    shared_data->db_merged_map = shared_data->db_original_maps[agent_id];
    return additional_loops;
  }

  Eigen::Matrix4f relative_map_pose;
  if (!TryKissMatcher(shared_data->db_merged_map,
                      shared_data->db_original_maps[agent_id],
                      kMapMatchingThreshold, false, relative_map_pose)) {
    return additional_loops;  // If map matching fails, return empty vector
  }

  // Transform poses for the current agent
  auto transformed_poses = transformEigenPoses(
      shared_data->db_odom_poses[agent_id], relative_map_pose);

  // Find additional loop pairs using KdTree search
  additional_loops = findLoopPairsFromKdTree(shared_data, transformed_poses,
                                             agent_id, kDistanceThreshold);

  // Transform and merge map points
  auto transformed_map_points = transformEigenPoints(
      shared_data->db_original_maps[agent_id], relative_map_pose);

  // Add transformed points to the merged map
  shared_data->db_merged_map.insert(shared_data->db_merged_map.end(),
                                    transformed_map_points.begin(),
                                    transformed_map_points.end());

  return additional_loops;
}

std::tuple<LoopPairVec, LoopPairVec> LoopDetectorKdtree::process(
    std::shared_ptr<SharedDatabase>& shared_data, const char agent_id,
    ScanVec scans) {
  database_->setAgentId(agent_id);
  // Detect intra-agent loops
  std::vector<LoopPair> intra_loop_pairs = detectIntraLoops(scans, agent_id);
  // Detect inter-agent loops
  std::vector<LoopPair> inter_loop_pairs =
      detectInterLoops(scans, shared_data, agent_id);
  // Transform and merge maps, get additional loops
  auto additional_loops = detectKissMatcherLoops(shared_data, agent_id);
  // Add additional loops from map matching to inter_loop_pairs
  inter_loop_pairs.insert(inter_loop_pairs.end(), additional_loops.begin(),
                          additional_loops.end());

  if (agent_id != 'A') {
    shared_data->total_db_descriptors.merge(database_.value());
  } else {
    shared_data->total_db_descriptors = std::move(database_.value());
  }

  return {intra_loop_pairs, inter_loop_pairs};
}

std::shared_ptr<IDescriptorKdtree> LoopDetectorKdtree::loadModule(
    const std::string& so_name) {
  return load_module_from_so<IDescriptorKdtree>(
      so_name, "create_descriptor_kdtree_module");
}

}  // namespace open_lmm