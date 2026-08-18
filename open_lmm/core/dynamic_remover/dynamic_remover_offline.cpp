#include "dynamic_remover_offline.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tqdmcpp/tqdmcpp.hpp>

#include <open_lmm/common/validation.hpp>
#include <open_lmm/core/algorithm_invariants.hpp>

#include <algorithm>
#include <set>

namespace open_lmm {

DynamicRemoverOffline::DynamicRemoverOffline(
    const OfflineParams& params, std::shared_ptr<IOfflineRemoverPlugin> model)
    : params_(params), offline_model_(std::move(model)) {}

pcl::PointCloud<pcl::PointXYZI>::Ptr DynamicRemoverOffline::genRawMap(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
  auto ordered_result = OrderOptimizedPosesByFrameId(
      optimized_poses, scans.size(), "offline raw-map generation");
  if (!ordered_result) {
    throw std::invalid_argument(ordered_result.GetError().Message());
  }
  PoseVec ordered_poses = std::move(ordered_result).Value();
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  for (std::size_t i = 0; i < scans.size(); ++i) {
    auto valid = ValidatePointCloud(
        scans[i], "offline raw-map scan " + std::to_string(i));
    if (!valid) throw std::invalid_argument(valid.GetError().Message());
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*scans[i], *transformed_scan,
                             ordered_poses[i].matrix());
    *raw_map += *transformed_scan;
  }

  return raw_map;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DynamicRemoverOffline::process(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
  auto ordered_result = OrderOptimizedPosesByFrameId(
      optimized_poses, scans.size(), "offline dynamic remover");
  if (!ordered_result) {
    throw std::invalid_argument(ordered_result.GetError().Message());
  }
  PoseVec ordered_poses = std::move(ordered_result).Value();
  for (std::size_t index = 0; index < scans.size(); ++index) {
    auto valid = ValidatePointCloud(
        scans[index], "offline dynamic remover scan " +
                          std::to_string(index));
    if (!valid) throw std::invalid_argument(valid.GetError().Message());
  }
  if (offline_model_->needsRawMap()) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map =
        genRawMap(scans, optimized_poses);
    offline_model_->setRawMap(raw_map);
  }

  auto T = tq::tqdm(scans);
  T.set_prefix("Dynamic Remover");
  int idx = 0;
  for (auto scan : T) {
    // pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan =
    //     pcl::PointCloud<pcl::PointXYZI>::Ptr(
    //         new pcl::PointCloud<pcl::PointXYZI>());
    // pcl::transformPointCloud(*scan, *transformed_scan,
    //                          optimized_poses[idx].second.matrix());
    offline_model_->run(scan, ordered_poses[idx]);
    idx++;
  }
  T.finish();

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map =
      offline_model_->getStaticMap();

  auto valid_map = ValidatePointCloud(static_map, "offline remover output");
  if (!valid_map) {
    throw std::invalid_argument(valid_map.GetError().Message());
  }

  return static_map;
}

Result<DynamicRemoverBase::PointCloud::Ptr>
DynamicRemoverOffline::processStreaming(
    const RawScanSource& source,
    const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses,
    const HeavyPhaseAdmission& heavy_phase_admission) {
  // Read a potentially stateful source exactly once.  Keeping the validated
  // indexed snapshot prevents a later traversal from changing frame identity
  // after validation but before plugin invocation.
  std::vector<std::pair<std::size_t, PointCloud::Ptr>> indexed_scans;
  std::set<std::size_t> frame_ids;
  auto loaded = source([&](std::size_t index, const PointCloud::Ptr& scan) {
    if (!frame_ids.insert(index).second) {
      return Result<void>::Failure(Error::InvalidArgument(
          "offline dynamic remover source returned duplicate frame " +
          std::to_string(index)));
    }
    auto valid = ValidatePointCloud(
        scan, "offline dynamic remover source frame " +
                  std::to_string(index));
    if (!valid) return valid;
    indexed_scans.emplace_back(index, scan);
    return Result<void>::Ok();
  });
  if (!loaded) {
    return Result<PointCloud::Ptr>::Failure(loaded.GetError());
  }
  if (loaded.Value() != indexed_scans.size()) {
    return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
        "offline dynamic remover source count differs from visited scan count"));
  }
  std::sort(indexed_scans.begin(), indexed_scans.end(),
            [](const auto& lhs, const auto& rhs) {
              return lhs.first < rhs.first;
            });
  ScanVec scans;
  scans.reserve(indexed_scans.size());
  for (std::size_t expected = 0; expected < indexed_scans.size(); ++expected) {
    if (indexed_scans[expected].first != expected) {
      return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
          "offline dynamic remover source is missing frame " +
          std::to_string(expected)));
    }
    scans.push_back(std::move(indexed_scans[expected].second));
  }
  auto ordered_result = OrderOptimizedPosesByFrameId(
      optimized_poses, scans.size(), "offline dynamic remover");
  if (!ordered_result) {
    return Result<PointCloud::Ptr>::Failure(ordered_result.GetError());
  }
  PoseVec ordered_poses = std::move(ordered_result).Value();

  if (offline_model_->needsRawMap()) {
    std::shared_ptr<void> heavy_phase;
    if (heavy_phase_admission) {
      auto admitted = heavy_phase_admission();
      if (!admitted) {
        return Result<PointCloud::Ptr>::Failure(admitted.GetError());
      }
      heavy_phase = std::move(admitted).Value();
    }
    auto raw_map = std::make_shared<PointCloud>();
    for (std::size_t index = 0; index < scans.size(); ++index) {
      PointCloud transformed_scan;
      pcl::transformPointCloud(*scans[index], transformed_scan,
                               ordered_poses[index].matrix());
      *raw_map += transformed_scan;
    }
    offline_model_->setRawMap(raw_map);
    heavy_phase.reset();
  }

  for (std::size_t index = 0; index < scans.size(); ++index) {
    PointCloud::Ptr mutable_scan = scans[index];
    Eigen::Isometry3d pose = ordered_poses[index];
    offline_model_->run(mutable_scan, pose);
  }
  auto static_map = offline_model_->getStaticMap();
  auto valid_map = ValidatePointCloud(static_map, "offline remover output");
  if (!valid_map) {
    return Result<PointCloud::Ptr>::Failure(valid_map.GetError());
  }
  return Result<PointCloud::Ptr>::Ok(std::move(static_map));
}

Result<std::shared_ptr<IOfflineRemoverPlugin>> DynamicRemoverOffline::loadModule(
    const std::string& so_name, const std::string& config_json) {
  return load_plugin_v1<IOfflineRemoverPlugin>(
      so_name, "dynamic_remover_offline", config_json);
}

}  // namespace open_lmm
