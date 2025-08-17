#pragma once
#include <open_lmm/core/dynamic_remover/remover_factory/online/interface_online_plugin.hpp>
#include <tqdmcpp/tqdmcpp.hpp>

#include "dynamic_remover_base.hpp"

namespace open_lmm {

struct OnlineParams {
 public:
  //   EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  explicit OnlineParams();
  ~OnlineParams() = default;

 public:
  std::string dynamic_remover_type;
  std::string model;
};

class DynamicRemoverOnline : public DynamicRemoverBase {
 public:
  DynamicRemoverOnline(const OnlineParams& params);
  ~DynamicRemoverOnline() override = default;
  pcl::PointCloud<pcl::PointXYZI>::Ptr process(
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) override;

 private:
  OnlineParams params_;
  std::shared_ptr<IOnlineRemoverPlugin> online_model_;

  static std::shared_ptr<IOnlineRemoverPlugin> loadModule(
      const std::string& so_name);
};

}  // namespace open_lmm
