#pragma once
#include <domain/dynamic_removal/plugin/online_plugin.hpp>

#include "dynamic_remover_base.hpp"

namespace open_lmm {

using OnlineParams = DynamicRemoverConfig;

class DynamicRemoverOnline : public DynamicRemoverBase {
 public:
  DynamicRemoverOnline(const OnlineParams& params,
                       std::shared_ptr<IOnlineRemoverPlugin> model);
  ~DynamicRemoverOnline() override = default;
  Result<PointCloud::Ptr> Process(
      const AlgorithmExecutionContext& context,
      DynamicRemoverInput input) override;
  Result<PointCloud::Ptr> ProcessStreaming(
      const AlgorithmExecutionContext& context,
      const DynamicRemoverStreamingInput& input) override;
  [[nodiscard]] DynamicRemoverStreamingMode StreamingMode() const override {
    return DynamicRemoverStreamingMode::kDirect;
  }

  static Result<std::shared_ptr<IOnlineRemoverPlugin>> loadModule(
      const std::string& so_name, const std::string& config_json);

 private:
  Result<PointCloud::Ptr> ProcessImpl(
      const AlgorithmExecutionContext& context,
      std::vector<PointCloud::Ptr> scans,
      std::vector<std::pair<int, Eigen::Isometry3d>> optimized_poses);
  OnlineParams params_;
  std::shared_ptr<IOnlineRemoverPlugin> online_model_;

};

}  // namespace open_lmm
