#pragma once

#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/common/runtime_contracts.hpp>

#include <cstdint>
#include <functional>
#include <filesystem>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm {

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
  std::vector<AgentId> affected_agents;
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
  uint64_t session_revision = 0;
  uint64_t config_revision = 0;
  std::vector<AgentId> agents;
  std::vector<ArtifactMetadata> artifacts;
  std::vector<ExecutionEvent> recent_events;
};

struct RuntimeSessionSnapshot {
  SessionId id;
  std::string label;
  RuntimeSessionState state = RuntimeSessionState::kCreating;
  std::filesystem::path output_directory;
  PipelineSnapshot pipeline;
};

struct SessionExecutionEvent {
  SessionId session_id;
  ExecutionEvent event;
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

struct RuntimeSessionReplacement {
  SessionId session_id;
  ExecutionEventSubscription event_subscription;
};

}  // namespace open_lmm
