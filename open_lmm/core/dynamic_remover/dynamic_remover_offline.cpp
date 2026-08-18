#include "dynamic_remover_offline.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tqdmcpp/tqdmcpp.hpp>

namespace open_lmm {

DynamicRemoverOffline::DynamicRemoverOffline(
    const OfflineParams& params, std::shared_ptr<IOfflineRemoverPlugin> model)
    : params_(params), offline_model_(std::move(model)) {}

pcl::PointCloud<pcl::PointXYZI>::Ptr DynamicRemoverOffline::genRawMap(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map =
      pcl::PointCloud<pcl::PointXYZI>::Ptr(
          new pcl::PointCloud<pcl::PointXYZI>());
  for (int i = 0; i < scans.size(); i++) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*scans[i], *transformed_scan,
                             optimized_poses[i].second.matrix());
    *raw_map += *transformed_scan;
  }

  return raw_map;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DynamicRemoverOffline::process(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
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
    offline_model_->run(scan, optimized_poses[idx].second);
    idx++;
  }
  T.finish();

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map =
      offline_model_->getStaticMap();

  return static_map;
}

Result<DynamicRemoverBase::PointCloud::Ptr>
DynamicRemoverOffline::processStreaming(
    const RawScanSource& source,
    const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses,
    const HeavyPhaseAdmission& heavy_phase_admission) {
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
    auto loaded = source([&](std::size_t index, const PointCloud::Ptr& scan) {
      if (index >= optimized_poses.size()) {
        return Result<void>::Failure(Error::InvalidArgument(
            "raw scan count exceeds optimized pose count"));
      }
      PointCloud transformed_scan;
      pcl::transformPointCloud(*scan, transformed_scan,
                               optimized_poses[index].second.matrix());
      *raw_map += transformed_scan;
      return Result<void>::Ok();
    });
    if (!loaded) return Result<PointCloud::Ptr>::Failure(loaded.GetError());
    if (loaded.Value() != optimized_poses.size()) {
      return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
          "raw scan and optimized pose counts differ"));
    }
    offline_model_->setRawMap(raw_map);
    heavy_phase.reset();
  }

  auto processed = source([&](std::size_t index, const PointCloud::Ptr& scan) {
    if (index >= optimized_poses.size()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "raw scan count exceeds optimized pose count"));
    }
    PointCloud::Ptr mutable_scan = scan;
    Eigen::Isometry3d pose = optimized_poses[index].second;
    offline_model_->run(mutable_scan, pose);
    return Result<void>::Ok();
  });
  if (!processed) {
    return Result<PointCloud::Ptr>::Failure(processed.GetError());
  }
  if (processed.Value() != optimized_poses.size()) {
    return Result<PointCloud::Ptr>::Failure(Error::InvalidArgument(
        "raw scan and optimized pose counts differ"));
  }
  return Result<PointCloud::Ptr>::Ok(offline_model_->getStaticMap());
}

Result<std::shared_ptr<IOfflineRemoverPlugin>> DynamicRemoverOffline::loadModule(
    const std::string& so_name, const std::string& config_json) {
  return load_plugin_v1<IOfflineRemoverPlugin>(
      so_name, "dynamic_remover_offline", config_json);
}

}  // namespace open_lmm
