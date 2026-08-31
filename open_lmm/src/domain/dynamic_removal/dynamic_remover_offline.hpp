#pragma once
#include <domain/dynamic_removal/plugin/offline_plugin.hpp>

#include "dynamic_remover_base.hpp"

namespace open_lmm {

using OfflineParams = DynamicRemoverConfig;

class DynamicRemoverOffline : public DynamicRemoverBase {
 public:
  DynamicRemoverOffline(const OfflineParams& params,
                        std::shared_ptr<IOfflineRemoverPlugin> model);
  ~DynamicRemoverOffline() override = default;
  Result<PointCloud::Ptr> Process(
      const AlgorithmExecutionContext& context,
      DynamicRemoverInput input) override;
  Result<PointCloud::Ptr> ProcessStreaming(
      const AlgorithmExecutionContext& context,
      const DynamicRemoverStreamingInput& input) override;
  pcl::PointCloud<pcl::PointXYZI>::Ptr genRawMap(
      std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses);

 private:
  PointCloud::Ptr ProcessImpl(
      const AlgorithmExecutionContext& context,
      std::vector<PointCloud::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses);
  Result<PointCloud::Ptr> ProcessStreamingImpl(
      const AlgorithmExecutionContext& context,
      const DynamicRemoverStreamingInput& input);
  OfflineParams params_;
  std::shared_ptr<IOfflineRemoverPlugin> offline_model_;

};

}  // namespace open_lmm
