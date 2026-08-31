#pragma once

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/runtime_api.hpp>
#include <runtime/execution/stage_ports.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace open_lmm {

struct ExecutionEventSubscriberRegistry;

class PipelineController {
 public:
  explicit PipelineController(std::shared_ptr<StageRuntimePort> port);
  PipelineController(std::shared_ptr<StageCommandPort> command_port,
                     std::shared_ptr<RuntimeQueryPort> query_port);
  ~PipelineController();
  PipelineController(const PipelineController&) = delete;
  PipelineController& operator=(const PipelineController&) = delete;

  Result<uint64_t> SubmitRunAll();
  Result<uint64_t> SubmitStage(StageId stage);
  Result<uint64_t> SubmitNode(
      NodeId node, std::optional<AgentId> agent = std::nullopt);
  Result<uint64_t> SubmitOptimizeThrough(AgentId target_agent);
  Result<void> ApplyConfig(ConfigDomain domain, uint64_t revision);
  Result<ConfigApplyReceipt> ApplyConfig(
      const ConfigCandidate& candidate, const ExpectedRevision& expected);
  Result<void> ReplacePorts(std::shared_ptr<StageCommandPort> command_port,
                            std::shared_ptr<RuntimeQueryPort> query_port);
  [[nodiscard]] std::vector<NodeDescriptor> NodeDescriptors() const;
  Result<void> Cancel(uint64_t job_id);
  Result<void> Wait(uint64_t job_id);
  // Lifecycle barrier for owners that must retire callback-visible context.
  // Wait(job) only waits for terminal journal commit by design.
  Result<void> WaitForEventCallbacks();
  [[nodiscard]] bool IsInEventCallback() const noexcept;
  [[nodiscard]] PipelineSnapshot Snapshot() const;
  [[nodiscard]] Result<VisualizationSnapshot> GetVisualizationSnapshot(
      const AgentId& agent) const;
  [[nodiscard]] Result<VisualizationSnapshot> GetVisualizationSnapshot(
      const VisualizationQuery& query) const;
  [[nodiscard]] std::optional<AlignmentFeedbackSnapshot>
  GetAlignmentFeedbackSnapshot() const;
  Result<void> RespondToAlignment(uint64_t job_id,
                                  AlignmentResponse response);
  void SetAlignmentFeedbackEnabled(bool enabled);
  [[nodiscard]] ExecutionEventSubscription SubscribeEvents(
      std::function<void(const ExecutionEvent&)> callback);

 private:
  using Work = std::function<Result<void>(uint64_t, const ExecutionContext&)>;
  Result<uint64_t> submit(Work work);
  Result<void> runOneStage(uint64_t job_id, StageId stage,
                           const ExecutionContext& context);
  ExecutionContext withProgress(
      const ExecutionContext& context, uint64_t job_id, StageId stage,
      std::optional<NodeId> node,
      std::optional<AgentId> fallback_agent = std::nullopt);
  bool cancellationRequested() const;
  void emit(ExecutionEvent event);
  void commitTerminal(uint64_t job_id, const Result<void>& result);
  void drainEventCallbacks();
  void synchronizeCommittedRuntime(
      const std::shared_ptr<RuntimeQueryPort>& query_port);
  Result<void> acceptExecutionReceipt(
      const ExecutionReceipt& receipt, uint64_t sent_base_revision,
      const std::shared_ptr<RuntimeQueryPort>& query_port);

  std::shared_ptr<StageCommandPort> command_port_;
  std::shared_ptr<RuntimeQueryPort> query_port_;
  // Serializes commands that may replace the worker, runner, or cancellation
  // token. The state mutex alone cannot protect the check-then-start sequence.
  mutable std::mutex command_mutex_;
  mutable std::mutex mutex_;
  std::condition_variable completed_;
  std::optional<JobSnapshot> job_;
  std::vector<AgentId> agents_;
  CommittedRuntimeSnapshot committed_runtime_;
  // A successful command followed by an invalid receipt leaves commit outcome
  // uncertain from the controller's perspective.  Keep the best query-port
  // snapshot for diagnostics, but reject further commands until the ports are
  // replaced with a fresh runtime instance.
  std::optional<Error> protocol_failure_;
  bool maintenance_in_progress_ = false;
  std::shared_ptr<ExecutionEventSubscriberRegistry> event_subscribers_;
  std::vector<ExecutionEvent> recent_events_;
  // Journal commit and callback queue insertion share this lock so callbacks
  // observe the same total ordering as recent_events_. Callback execution is
  // outside both this lock and mutex_.
  mutable std::mutex event_dispatch_mutex_;
  std::condition_variable event_callbacks_idle_;
  std::deque<ExecutionEvent> pending_event_callbacks_;
  bool dispatching_events_ = false;
  std::thread worker_;
  std::atomic<bool> cancel_requested_{false};
  std::shared_ptr<CancellationToken> cancellation_;
  CancellationCapability cancellation_capability_;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback_;
  bool alignment_feedback_published_ = false;
  uint64_t next_job_id_ = 1;
  uint64_t next_event_sequence_ = 1;
  uint64_t terminal_event_completed_job_id_ = 0;
  uint64_t config_revision_ = 1;
  uint64_t committed_runtime_revision_ = 0;
};

}  // namespace open_lmm
