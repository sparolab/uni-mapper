#pragma once

#include <open_lmm/common/runtime_api.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/runtime_contracts.hpp>
#include <open_lmm/server/bootstrap/bootstrap_config.hpp>
#include <open_lmm/server/pipeline_controller.hpp>
#include <open_lmm/server/resource_governor.hpp>

#include <filesystem>
#include <condition_variable>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm {

class PendingOutputSet;

// Owns exactly one externally visible runtime. A root/DataLoad replacement
// builds one private candidate and atomically swaps it for active_ only after
// config-file installation succeeds.
class RuntimeService {
 public:
  using PortFactory = std::function<Result<std::shared_ptr<StageRuntimePort>>(
      const BootstrapConfigSnapshot&, const std::filesystem::path&)>;

  explicit RuntimeService(std::size_t max_agent_tasks = 2,
                          PortFactory port_factory = {});
  explicit RuntimeService(ResourceBudget budget, PortFactory port_factory = {});
  ~RuntimeService();
  RuntimeService(const RuntimeService&) = delete;
  RuntimeService& operator=(const RuntimeService&) = delete;

  Result<void> Open(const BootstrapRequest& request);
  Result<void> Open(const BootstrapRequest& request,
                    const ConfigCandidate& root_candidate);
  Result<RuntimeReplaceReceipt> ReplaceRootConfig(
      const BootstrapRequest& request, const ConfigCandidate& root_candidate,
      const ExpectedRevision& expected);

  Result<JobHandle> Submit(const ExecutionRequest& request);
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
  [[nodiscard]] bool IsInEventCallback() const;
  [[nodiscard]] const ResourceGovernor& Governor() const noexcept {
    return *governor_;
  }

 private:
  enum class LifecycleState { kClosed, kOpening, kReady, kReplacing, kClosing };
  struct RuntimeInstance;
  struct SubscriberRegistry;
  struct SubscriberSlot;
  struct OperationLease;
  struct PublicJob;

  Result<std::shared_ptr<RuntimeInstance>> BuildInstance(
      const BootstrapRequest& request, BootstrapConfigSnapshot bootstrap,
      uint64_t epoch, uint64_t initial_runtime_revision = 1,
      uint64_t initial_config_revision = 1) const;
  Result<std::shared_ptr<RuntimeInstance>> BuildInstance(
      const BootstrapRequest& request, const ConfigCandidate& root_candidate,
      uint64_t epoch, uint64_t initial_runtime_revision = 1,
      uint64_t initial_config_revision = 1) const;
  Result<std::shared_ptr<RuntimeInstance>> Active() const;
  Result<OperationLease> AcquireOperation(bool require_ready) const;
  Result<PublicJob> ResolveJob(JobHandle job) const;
  Result<void> AttachEventSource(const std::shared_ptr<RuntimeInstance>& instance);
  Result<void> StageRootConfig(const RuntimeInstance& candidate,
                               PendingOutputSet& pending) const;
  void FinishTransitionLocked(uint64_t generation, LifecycleState next);
  void DispatchEvent(const std::shared_ptr<RuntimeInstance>& instance,
                     const ExecutionEvent& event);
  uint64_t MapPublicJobLocked(uint64_t epoch, JobId local_job);
  void MarkTerminalPublicJobLocked(uint64_t handle);
  void ClearPublicJobsLocked();
  static bool IsActive(JobState state);
  static RuntimeStatus DeriveState(
      const RuntimeInstance& instance, const PipelineSnapshot& pipeline);
  static std::string GenerateOutputNamespace();

  mutable std::mutex mutex_;
  std::condition_variable lifecycle_changed_;
  std::shared_ptr<RuntimeInstance> active_;
  LifecycleState lifecycle_ = LifecycleState::kClosed;
  uint64_t transition_generation_ = 0;
  std::shared_ptr<CancellationToken> transition_cancellation_;
  uint64_t epoch_ = 0;
  uint64_t next_public_job_ = 1;
  std::optional<std::pair<uint64_t, uint64_t>> pending_public_job_;
  std::map<std::pair<uint64_t, JobId>, uint64_t> public_job_ids_;
  std::map<uint64_t, PublicJob> public_jobs_;
  std::deque<uint64_t> terminal_public_jobs_;
  uint64_t next_subscriber_id_ = 1;
  std::shared_ptr<SubscriberRegistry> subscribers_;
  bool feedback_enabled_ = false;
  uint64_t next_public_event_sequence_ = 1;
  std::deque<ExecutionEvent> recent_public_events_;
  std::shared_ptr<ResourceGovernor> governor_;
  PortFactory port_factory_;
  mutable std::mutex submit_mutex_;
};

}  // namespace open_lmm
