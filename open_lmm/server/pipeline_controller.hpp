#pragma once

#include <open_lmm/common/result.hpp>
#include <open_lmm/server/artifact_repository.hpp>
#include <open_lmm/server/stage_runner.hpp>

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace open_lmm {

struct ExecutionEventSubscriberRegistry;

enum class JobState : uint8_t {
  kQueued, kWaitingForDependency, kRunning, kCancelling,
  kWaitingForAlignmentFeedback, kSucceeded, kFailed, kCancelled
};
enum class EventType : uint8_t {
  kJobQueued, kJobStarted, kStageStarted, kNodeStarted,
  kProgressUpdated, kArtifactCommitted, kArtifactInvalidated,
  kNodeFailed, kStageCompleted, kStageFailed,
  kCancellationRequested, kAlignmentFeedbackRequested,
  kAlignmentProposalAccepted, kAlignmentProposalRejected,
  kAlignmentFeedbackCancelled, kJobCompleted, kJobCancelled
};

struct ExecutionEvent {
  uint64_t job_id = 0;
  EventType type = EventType::kJobStarted;
  std::optional<StageId> stage;
  std::string message;
  uint64_t sequence = 0;
  std::optional<NodeId> node;
  std::optional<AgentId> agent;
  uint64_t progress_current = 0;
  uint64_t progress_total = 0;
  std::optional<Error> error;
  std::optional<CancellationTelemetry> cancellation;
};
struct JobSnapshot {
  uint64_t id = 0;
  JobState state = JobState::kQueued;
  std::optional<StageId> active_stage;
  std::string message;
  CancellationTelemetry cancellation;
};
struct PipelineSnapshot {
  std::optional<JobSnapshot> job;
  uint64_t config_revision = 0;
  std::vector<AgentId> agents;
  std::vector<ArtifactMetadata> artifacts;
  std::vector<ExecutionEvent> recent_events;
};

class ExecutionEventSubscription {
 public:
  ExecutionEventSubscription() = default;
  explicit ExecutionEventSubscription(std::function<void()> unsubscribe);
  ~ExecutionEventSubscription();
  ExecutionEventSubscription(ExecutionEventSubscription&& other) noexcept;
  ExecutionEventSubscription& operator=(ExecutionEventSubscription&& other) noexcept;
  ExecutionEventSubscription(const ExecutionEventSubscription&) = delete;
  ExecutionEventSubscription& operator=(const ExecutionEventSubscription&) = delete;
  void Reset();
 private:
  std::function<void()> unsubscribe_;
};

class PipelineController {
 public:
  explicit PipelineController(std::shared_ptr<StageRunner> runner);
  ~PipelineController();
  PipelineController(const PipelineController&) = delete;
  PipelineController& operator=(const PipelineController&) = delete;

  Result<uint64_t> SubmitRunAll();
  Result<uint64_t> SubmitStage(StageId stage);
  Result<uint64_t> SubmitNode(NodeId node, AgentId agent);
  Result<uint64_t> SubmitOptimizeThrough(AgentId target_agent);
  Result<void> ApplyConfig(ConfigDomain domain, uint64_t revision);
  Result<void> ReplaceRunner(std::shared_ptr<StageRunner> runner);
  [[nodiscard]] std::vector<NodeDescriptor> NodeDescriptors() const;
  Result<void> Cancel(uint64_t job_id);
  Result<void> Wait(uint64_t job_id);
  [[nodiscard]] PipelineSnapshot Snapshot() const;
  [[nodiscard]] Result<VisualizationSnapshot> GetVisualizationSnapshot(
      const AgentId& agent) const;
  [[nodiscard]] std::optional<AlignmentFeedbackSnapshot>
  GetAlignmentFeedbackSnapshot() const;
  Result<void> RespondToAlignment(uint64_t job_id,
                                  AlignmentResponse response);
  void SetAlignmentFeedbackEnabled(bool enabled);
  [[nodiscard]] ExecutionEventSubscription SubscribeEvents(
      std::function<void(const ExecutionEvent&)> callback);

 private:
  using Work = std::function<Result<void>(uint64_t)>;
  Result<uint64_t> submit(Work work);
  Result<void> runOneStage(uint64_t job_id, StageId stage);
  bool cancellationRequested() const;
  void emit(ExecutionEvent event);
  void finish(uint64_t job_id, const Result<void>& result);
  bool synchronizeCommittedSession(
      const std::shared_ptr<StageRunner>& runner);

  std::shared_ptr<StageRunner> runner_;
  // Serializes commands that may replace the worker, runner, or cancellation
  // token. The state mutex alone cannot protect the check-then-start sequence.
  mutable std::mutex command_mutex_;
  ArtifactRepository artifacts_;
  mutable std::mutex mutex_;
  std::condition_variable completed_;
  std::optional<JobSnapshot> job_;
  std::vector<AgentId> agents_;
  bool maintenance_in_progress_ = false;
  std::shared_ptr<ExecutionEventSubscriberRegistry> event_subscribers_;
  std::vector<ExecutionEvent> recent_events_;
  std::thread worker_;
  std::atomic<bool> cancel_requested_{false};
  std::atomic<bool> runner_manages_artifacts_{false};
  std::shared_ptr<CancellationToken> cancellation_;
  CancellationCapability cancellation_capability_;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback_;
  uint64_t next_job_id_ = 1;
  uint64_t next_event_sequence_ = 1;
  uint64_t terminal_event_completed_job_id_ = 0;
  uint64_t config_revision_ = 1;
};

}  // namespace open_lmm
