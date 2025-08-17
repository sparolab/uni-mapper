#pragma once

#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/dynamic_remover/remover_factory/offline/interface_offline_plugin.hpp>
#include <open_lmm/core/dynamic_remover/remover_factory/online/interface_online_plugin.hpp>
#include <open_lmm/utils/config.hpp>
#include <open_lmm/utils/load_module.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <filesystem>
#include <memory>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string>

namespace fs = std::filesystem;

namespace open_lmm {

class DynamicRemoverBase {
 public:
  DynamicRemoverBase() = default;
  virtual ~DynamicRemoverBase() = default;
  virtual pcl::PointCloud<pcl::PointXYZI>::Ptr process(
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) = 0;
  static std::shared_ptr<DynamicRemoverBase> createInstance(Config config);
};

}  // namespace open_lmm