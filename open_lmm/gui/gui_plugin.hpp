#pragma once
#include <open_lmm/common/result.hpp>
#include <open_lmm/common/config_transaction.hpp>
#include <open_lmm/common/runtime_api.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <open_lmm/common/alignment_types.hpp>
#include <functional>
#include <optional>
#include <string>

namespace open_lmm {
struct GuiServices {
  std::string config_file_path;
  std::function<Result<uint64_t>()> submit_run_all;
  std::function<Result<uint64_t>(StageId)> submit_stage;
  std::function<Result<uint64_t>(NodeId, std::optional<AgentId>)> submit_node;
  std::function<Result<uint64_t>(AgentId)> submit_optimize_through;
  std::function<Result<void>(uint64_t)> cancel_job;
  std::function<Result<ConfigApplyReceipt>(SessionId, ConfigCandidate,
                                           ExpectedRevision)> apply_config;
  std::function<Result<void>(ConfigCandidate)> replace_session;
  std::function<std::vector<NodeDescriptor>()> node_descriptors;
  std::function<Result<RuntimeSessionSnapshot>()> runtime_snapshot;
  std::function<PipelineSnapshot()> snapshot;
  std::function<Result<VisualizationSnapshot>(const AgentId&)> visualization_snapshot;
  std::function<std::optional<AlignmentFeedbackSnapshot>()>
      alignment_feedback_snapshot;
  std::function<Result<void>(uint64_t, AlignmentResponse)>
      respond_to_alignment;
  std::function<Result<ExecutionEventSubscription>(
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
