#pragma once
#include <open_lmm/common/result.hpp>
#include <open_lmm/server/pipeline_controller.hpp>
#include <functional>
#include <string>

namespace open_lmm {
struct GuiServices {
  std::string config_file_path;
  std::function<Result<uint64_t>()> submit_run_all;
  std::function<Result<uint64_t>(StageId)> submit_stage;
  std::function<Result<uint64_t>(NodeId, AgentId)> submit_node;
  std::function<Result<uint64_t>(AgentId)> submit_optimize_through;
  std::function<Result<void>(uint64_t)> cancel_job;
  std::function<Result<void>(ConfigDomain, uint64_t)> apply_config;
  std::function<Result<void>(const std::string&)> create_session;
  std::function<std::vector<NodeDescriptor>()> node_descriptors;
  std::function<PipelineSnapshot()> snapshot;
  std::function<Result<VisualizationSnapshot>(const AgentId&)> visualization_snapshot;
  std::function<std::optional<AlignmentFeedbackSnapshot>()>
      alignment_feedback_snapshot;
  std::function<Result<void>(uint64_t, AlignmentResponse)>
      respond_to_alignment;
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
}  // namespace open_lmm
