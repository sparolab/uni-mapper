#pragma once
#include <tqdmcpp/tqdmcpp.hpp>


#include <open_lmm/core/dynamic_remover/remover_factory/offline/interface_offline_plugin.hpp>

#include "dynamic_remover_base.hpp"

namespace open_lmm {

struct OfflineParams {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit OfflineParams();
  ~OfflineParams() = default;

 public:
  std::string dynamic_remover_type;
  std::string model;
};

class DynamicRemoverOffline : public DynamicRemoverBase {
 public:
  DynamicRemoverOffline(const OfflineParams& params);
  ~DynamicRemoverOffline() override = default;
  pcl::PointCloud<pcl::PointXYZI>::Ptr process(
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) override;
  pcl::PointCloud<pcl::PointXYZI>::Ptr genRawMap(
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses);

 private:
  OfflineParams params_;
  std::shared_ptr<IOfflineRemoverPlugin> offline_model_;

  /**
   * @brief Load an dynamic removal module from a dynamic library
   * @param so_name  Dynamic library name
   * @return         Loaded dynamic removal module
   */
  static std::shared_ptr<IOfflineRemoverPlugin> loadModule(
      const std::string& so_name);
};

}  // namespace open_lmm
