#include "dynamic_remover_online.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>
#include <open_lmm/common/pointcloud_utils.hpp>

namespace open_lmm {

OnlineParams::OnlineParams() {
  Config config =
      Config(GlobalConfig::get_global_config_path("config_dynamic_remover"));
  dynamic_remover_type =
      config.param<std::string>("dynamic_remover", "dynamic_remover_type", "");
  model = config.param<std::string>("dynamic_remover", "model", "");
}

DynamicRemoverOnline::DynamicRemoverOnline(const OnlineParams& params)
    : params_(params) {
  std::string so_model_name = "libcreate_" + params_.model + ".so";
  online_model_ = loadModule(so_model_name);
}

pcl::PointCloud<pcl::PointXYZI>::Ptr DynamicRemoverOnline::process(
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
    std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) {
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_scan(
      new pcl::PointCloud<pcl::PointXYZI>);

  auto T = tq::tqdm(scans);
  T.set_prefix("Dynamic Remover");
  int idx = 0;
  for (auto scan : T) {
    static_scan = online_model_->run(scans[idx], optimized_poses[idx].second);
    idx++;
  }
  T.finish();

  std::cout << "Saving static map" << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map =
      online_model_->getStaticMap();

  // TODO(gil) : hardcoded voxel leaf size
  downsampleWithRangeFilter(static_map, 0.2, 0, 0, false);

  return static_map;
}

std::shared_ptr<IOnlineRemoverPlugin> DynamicRemoverOnline::loadModule(
    const std::string& so_name) {
  return load_module_from_so<IOnlineRemoverPlugin>(
      so_name, "create_dynamic_remover_module");
}

}  // namespace open_lmm