#include "dynamic_remover_offline.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

namespace open_lmm {

OfflineParams::OfflineParams() {
  Config config =
      Config(GlobalConfig::get_global_config_path("config_dynamic_remover"));
  dynamic_remover_type =
      config.param<std::string>("dynamic_remover", "dynamic_remover_type", "");
  model = config.param<std::string>("dynamic_remover", "model", "");
}

DynamicRemoverOffline::DynamicRemoverOffline(const OfflineParams& params)
    : params_(params) {
  std::string so_model_name = "libcreate_" + params_.model + ".so";
  offline_model_ = loadModule(so_model_name);
}

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
  pcl::PointCloud<pcl::PointXYZI>::Ptr raw_map =
      genRawMap(scans, optimized_poses);

  offline_model_->setRawMap(raw_map);

  auto T = tq::tqdm(scans);
  T.set_prefix("Dynamic Remover");
  int idx = 0;
  for (auto scan : T) {
    pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan =
        pcl::PointCloud<pcl::PointXYZI>::Ptr(
            new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*scan, *transformed_scan,
                             optimized_poses[idx].second.matrix());
    offline_model_->run(transformed_scan, optimized_poses[idx].second);
    idx++;
  }
  T.finish();

  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map =
      offline_model_->getStaticMap();

  return static_map;
}

std::shared_ptr<IOfflineRemoverPlugin> DynamicRemoverOffline::loadModule(
    const std::string& so_name) {
  return load_module_from_so<IOfflineRemoverPlugin>(
      so_name, "create_dynamic_remover_module");
}

}  // namespace open_lmm