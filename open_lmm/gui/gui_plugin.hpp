#pragma once
#include <open_lmm/common/result.hpp>
#include <open_lmm/server/pipeline_controller.hpp>
#include <functional>

namespace open_lmm {
struct GuiServices {
  std::function<Result<uint64_t>()> submit_run_all;
  std::function<Result<uint64_t>(StageId)> submit_stage;
  std::function<Result<uint64_t>(NodeId, char)> submit_node;
  std::function<Result<uint64_t>(char)> submit_optimize_through;
  std::function<Result<void>(uint64_t)> cancel_job;
  std::function<Result<void>(ConfigDomain, uint64_t)> apply_config;
  std::function<PipelineSnapshot()> snapshot;
  std::function<Result<VisualizationSnapshot>(char, std::size_t)>
      visualization_snapshot;
  std::function<ExecutionEventSubscription(
      std::function<void(const ExecutionEvent&)>)> subscribe_events;
};

class GuiPlugin {
 public:
  virtual ~GuiPlugin() = default;
  virtual Result<void> Start(GuiServices services) = 0;
  [[nodiscard]] virtual bool IsOpen() const = 0;
  virtual void RequestStop() = 0;
  virtual void Join() = 0;
};
inline constexpr const char* kCreateGuiPluginSymbol = "create_gui_plugin";
}  // namespace open_lmm
