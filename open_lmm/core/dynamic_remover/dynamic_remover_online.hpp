#pragma once
#include <open_lmm/core/dynamic_remover/remover_factory/online/interface_online_plugin.hpp>

#include "dynamic_remover_base.hpp"

namespace open_lmm {

using OnlineParams = DynamicRemoverConfig;

class DynamicRemoverOnline : public DynamicRemoverBase {
 public:
  DynamicRemoverOnline(const OnlineParams& params,
                       std::shared_ptr<IOnlineRemoverPlugin> model);
  ~DynamicRemoverOnline() override = default;
  pcl::PointCloud<pcl::PointXYZI>::Ptr process(
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses) override;

  static Result<std::shared_ptr<IOnlineRemoverPlugin>> loadModule(
      const std::string& so_name, const std::string& config_json);

 private:
  OnlineParams params_;
  std::shared_ptr<IOnlineRemoverPlugin> online_model_;

};

}  // namespace open_lmm
