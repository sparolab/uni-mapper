#include "dynamic_remover_online.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <tqdmcpp/tqdmcpp.hpp>

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/utils/logging.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/util/downsampling_tbb.hpp>

namespace open_lmm {

DynamicRemoverOnline::DynamicRemoverOnline(
    const OnlineParams& params, std::shared_ptr<IOnlineRemoverPlugin> model)
    : params_(params), online_model_(std::move(model)) {}

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

  LogInfo("[dynamic_remover] building static map from " +
          std::to_string(scans.size()) + " scans");
  pcl::PointCloud<pcl::PointXYZI>::Ptr static_map =
      online_model_->getStaticMap();

  // TODO(gil) : hardcoded voxel leaf size
  downsampleWithRangeFilter(static_map, 0.2, 0, 0, false);

  return static_map;
}

Result<std::shared_ptr<IOnlineRemoverPlugin>> DynamicRemoverOnline::loadModule(
    const std::string& so_name, const std::string& config_json) {
  return load_plugin_v1<IOnlineRemoverPlugin>(
      so_name, "dynamic_remover_online", config_json);
}

}  // namespace open_lmm
