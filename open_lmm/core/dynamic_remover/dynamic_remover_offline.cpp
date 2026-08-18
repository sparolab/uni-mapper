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

Result<std::shared_ptr<IOfflineRemoverPlugin>> DynamicRemoverOffline::loadModule(
    const std::string& so_name, const std::string& config_json) {
  return load_plugin_v1<IOfflineRemoverPlugin>(
      so_name, "dynamic_remover_offline", config_json);
}

}  // namespace open_lmm
