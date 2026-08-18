#pragma once

#include "dynamic_remover_base.hpp"

#include <open_lmm/common/plugin_host_v2.hpp>

#include <memory>
#include <string>

namespace open_lmm {

class DynamicRemoverV2 final : public DynamicRemoverBase {
 public:
  DynamicRemoverV2(std::shared_ptr<PluginV2> plugin,
                   uint64_t required_mode_capability);

  Result<PointCloud::Ptr> Process(const AlgorithmExecutionContext& context,
                                  DynamicRemoverInput input) override;
  Result<PointCloud::Ptr> ProcessStreaming(
      const AlgorithmExecutionContext& context,
      const DynamicRemoverStreamingInput& input) override;

  static Result<std::optional<std::shared_ptr<DynamicRemoverV2>>> TryLoad(
      const std::string& shared_library, std::string_view config_json,
      uint64_t required_mode_capability);

 private:
  struct IndexedScan {
    uint64_t frame_id = 0;
    PointCloud::Ptr cloud;
  };

  Result<PointCloud::Ptr> Execute(
      const AlgorithmExecutionContext& context,
      std::vector<IndexedScan> scans,
      const std::vector<std::pair<int, Eigen::Isometry3d>>& optimized_poses);

  std::shared_ptr<PluginV2> plugin_;
  uint64_t required_mode_capability_ = 0;
};

}  // namespace open_lmm
