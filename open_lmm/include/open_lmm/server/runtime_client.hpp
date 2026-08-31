#pragma once

#include <open_lmm/common/alignment_types.hpp>
#include <open_lmm/common/config_transaction.hpp>
#include <open_lmm/common/runtime_api.hpp>
#include <open_lmm/common/runtime_contracts.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>

#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <vector>

namespace open_lmm {

// Stable PImpl façade for the one active runtime in this process.
class RuntimeClient {
 public:
  explicit RuntimeClient(std::size_t max_agent_tasks = 4);
  ~RuntimeClient();
  RuntimeClient(RuntimeClient&&) noexcept;
  RuntimeClient& operator=(RuntimeClient&&) noexcept;
  RuntimeClient(const RuntimeClient&) = delete;
  RuntimeClient& operator=(const RuntimeClient&) = delete;

  Result<void> Open(const BootstrapRequest& request);
  Result<void> Open(const BootstrapRequest& request,
                    const ConfigCandidate& root_candidate);
  Result<RuntimeReplaceReceipt> ReplaceRootConfig(
      const BootstrapRequest& request, const ConfigCandidate& root_candidate,
      const ExpectedRevision& expected);
  Result<JobHandle> Submit(const ExecutionRequest& request);
  Result<JobHandle> SubmitRunAll();
  Result<void> Cancel(JobHandle job);
  Result<void> Wait(JobHandle job);
  Result<RuntimeSnapshot> Snapshot() const;
  Result<std::vector<NodeDescriptor>> NodeDescriptors() const;
  Result<VisualizationSnapshot> Visualization(const AgentId& agent) const;
  Result<VisualizationSnapshot> Visualization(
      const VisualizationQuery& query) const;
  Result<std::optional<AlignmentFeedbackSnapshot>> AlignmentFeedback() const;
  Result<void> RespondToAlignment(JobHandle job, AlignmentResponse response);
  Result<void> SetAlignmentFeedbackEnabled(bool enabled);
  Result<ConfigApplyReceipt> ApplyConfig(
      const ConfigCandidate& candidate, const ExpectedRevision& expected);
  Result<ExecutionEventSubscription> SubscribeEvents(
      std::function<void(const ExecutionEvent&)> callback);
  Result<void> Close(CloseMode mode = CloseMode::kCancelAndWait);
  [[nodiscard]] bool IsOpen() const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace open_lmm
