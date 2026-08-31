#include "runtime_service.hpp"

#include <runtime/composition/map_server.hpp>
#include <storage/transactions/output_repository.hpp>
#include <foundation/logging/logging.hpp>

#include <algorithm>
#include <condition_variable>
#include <exception>
#include <filesystem>
#include <random>
#include <sstream>
#include <thread>
#include <utility>

namespace open_lmm {
namespace fs = std::filesystem;
namespace {

struct SubscriberCallbackScope {
  explicit SubscriberCallbackScope(const void* slot) : previous(active_) {
    active_ = slot;
  }
  ~SubscriberCallbackScope() { active_ = previous; }
  static thread_local const void* active_;
  const void* previous;
};
thread_local const void* SubscriberCallbackScope::active_ = nullptr;

ResourceBudget SingleRuntimeBudget(std::size_t max_agent_tasks) {
  ResourceBudget budget;
  budget.max_agent_tasks = std::max<std::size_t>(1, max_agent_tasks);
  budget.max_cpu_threads = std::max<std::size_t>(1, max_agent_tasks);
  return budget;
}

struct UnpublishedOutputRollback {
  const fs::path* path = nullptr;
  bool published = false;

  ~UnpublishedOutputRollback() { RollbackNow(); }

  void RollbackNow() noexcept {
    if (published || !path) return;
    std::error_code ignored;
    fs::remove_all(*path, ignored);
    path = nullptr;
  }

  void Publish() noexcept { published = true; }
};

}  // namespace

struct RuntimeService::SubscriberSlot {
  std::mutex mutex;
  std::condition_variable idle;
  bool active = true;
  std::size_t callbacks_in_flight = 0;
  std::function<void(const ExecutionEvent&)> callback;
};

struct RuntimeService::SubscriberRegistry {
  std::mutex mutex;
  bool closing = false;
  std::map<uint64_t, std::shared_ptr<SubscriberSlot>> slots;
};

struct RuntimeService::RuntimeInstance {
  explicit RuntimeInstance(uint64_t runtime_epoch) : epoch(runtime_epoch) {}

  uint64_t epoch;
  std::string label;
  std::shared_ptr<const BootstrapConfigSnapshot> bootstrap_config;
  fs::path output_directory;
  std::shared_ptr<StageRuntimePort> port;
  std::shared_ptr<PipelineController> controller;
  ExecutionEventSubscription event_source;
  mutable std::mutex mutex;
  std::condition_variable idle;
  std::size_t operations_in_progress = 0;
  RuntimeStatus state = RuntimeStatus::kCreating;
  bool closing = false;
};

struct RuntimeService::OperationLease {
  OperationLease() = default;
  OperationLease(std::shared_ptr<RuntimeInstance> owner, uint64_t captured_epoch)
      : instance(std::move(owner)), epoch(captured_epoch) {}
  OperationLease(OperationLease&& other) noexcept
      : instance(std::move(other.instance)), epoch(other.epoch) {}
  OperationLease& operator=(OperationLease&&) = delete;
  OperationLease(const OperationLease&) = delete;
  OperationLease& operator=(const OperationLease&) = delete;
  ~OperationLease() {
    if (!instance) return;
    std::lock_guard lock(instance->mutex);
    --instance->operations_in_progress;
    instance->idle.notify_all();
  }
  std::shared_ptr<RuntimeInstance> instance;
  uint64_t epoch = 0;
};

struct RuntimeService::PublicJob {
  uint64_t epoch = 0;
  JobId local_job = 0;
};

RuntimeService::RuntimeService(std::size_t max_agent_tasks,
                               PortFactory port_factory)
    : RuntimeService(SingleRuntimeBudget(max_agent_tasks),
                     std::move(port_factory)) {}

RuntimeService::RuntimeService(ResourceBudget budget, PortFactory port_factory)
    : subscribers_(std::make_shared<SubscriberRegistry>()),
      governor_(std::make_shared<ResourceGovernor>(budget)),
      port_factory_(std::move(port_factory)) {
  if (!port_factory_) {
    port_factory_ = [governor = governor_](
                        const BootstrapConfigSnapshot& bootstrap,
                        const fs::path& output_directory)
        -> Result<std::shared_ptr<StageRuntimePort>> {
      auto port = std::make_shared<MapServer>(bootstrap, output_directory,
                                              governor);
      auto ready = port->ValidateReady();
      return ready ? Result<std::shared_ptr<StageRuntimePort>>::Ok(std::move(port))
                   : Result<std::shared_ptr<StageRuntimePort>>::Failure(
                         ready.GetError());
    };
  }
}

RuntimeService::~RuntimeService() {
  (void)Close(CloseMode::kCancelAndWait);
  const auto subscribers = std::move(subscribers_);
  if (!subscribers) return;
  std::lock_guard lock(subscribers->mutex);
  subscribers->closing = true;
  subscribers->slots.clear();
}

std::string RuntimeService::GenerateOutputNamespace() {
  std::random_device source;
  std::ostringstream output;
  output << std::hex;
  for (std::size_t index = 0; index < 4; ++index) output << source();
  return "runtime-" + output.str();
}

bool RuntimeService::IsActive(JobState state) {
  return state == JobState::kQueued || state == JobState::kWaitingForDependency ||
         state == JobState::kRunning || state == JobState::kCancelling ||
         state == JobState::kWaitingForAlignmentFeedback;
}

RuntimeStatus RuntimeService::DeriveState(
    const RuntimeInstance& instance, const PipelineSnapshot& pipeline,
    const std::optional<Error>& fatal_runtime_error) {
  if (instance.closing) return RuntimeStatus::kClosing;
  if (fatal_runtime_error) return RuntimeStatus::kFailedFatal;
  if (!pipeline.job) return RuntimeStatus::kReady;
  switch (pipeline.job->state) {
    case JobState::kQueued:
    case JobState::kWaitingForDependency:
    case JobState::kRunning:
    case JobState::kWaitingForAlignmentFeedback:
      return RuntimeStatus::kRunning;
    case JobState::kCancelling: return RuntimeStatus::kCancelling;
    case JobState::kSucceeded:
    case JobState::kCancelled: return RuntimeStatus::kReady;
    case JobState::kFailed:
      for (auto event = pipeline.recent_events.rbegin();
           event != pipeline.recent_events.rend(); ++event) {
        if (event->job_id != pipeline.job->id || !event->error) continue;
        return event->error->severity == Error::Severity::kFatalRuntime
                   ? RuntimeStatus::kFailedFatal
                   : RuntimeStatus::kFailedRecoverable;
      }
      return RuntimeStatus::kFailedRecoverable;
  }
  return RuntimeStatus::kFailedFatal;
}

Result<std::shared_ptr<RuntimeService::RuntimeInstance>>
RuntimeService::BuildInstance(const BootstrapRequest& request,
                              BootstrapConfigSnapshot bootstrap,
                              uint64_t epoch, uint64_t initial_runtime_revision,
                              uint64_t initial_config_revision) const {
  if (request.config_directory.empty()) {
    return Result<std::shared_ptr<RuntimeInstance>>::Failure(
        Error::InvalidArgument("runtime config directory must be non-empty"));
  }
  auto snapshot = std::make_shared<const BootstrapConfigSnapshot>(
      std::move(bootstrap));
  const fs::path output_root = request.output_root.value_or(snapshot->OutputRoot());
  if (output_root.empty()) {
    return Result<std::shared_ptr<RuntimeInstance>>::Failure(
        Error::InvalidArgument("runtime output root must be non-empty"));
  }
  const fs::path output_directory = output_root / GenerateOutputNamespace();
  struct OutputRollback {
    fs::path path;
    bool committed = false;
    ~OutputRollback() {
      if (committed || path.filename().string().rfind("runtime-", 0) != 0) return;
      std::error_code ignored;
      fs::remove_all(path, ignored);
    }
  } rollback{output_directory};
  auto port = port_factory_(*snapshot, output_directory);
  if (!port) return Result<std::shared_ptr<RuntimeInstance>>::Failure(port.GetError());
  auto initialized = port.Value()->InitializeRuntimeRevisions(
      initial_runtime_revision, initial_config_revision);
  if (!initialized) {
    return Result<std::shared_ptr<RuntimeInstance>>::Failure(
        initialized.GetError());
  }
  auto instance = std::make_shared<RuntimeInstance>(epoch);
  instance->label = request.label;
  instance->bootstrap_config = std::move(snapshot);
  instance->output_directory = output_directory;
  instance->port = std::move(port).Value();
  instance->controller = std::make_shared<PipelineController>(instance->port);
  bool feedback_enabled = false;
  {
    std::lock_guard lock(mutex_);
    feedback_enabled = feedback_enabled_;
  }
  instance->controller->SetAlignmentFeedbackEnabled(feedback_enabled);
  instance->state = RuntimeStatus::kReady;
  rollback.committed = true;
  return Result<std::shared_ptr<RuntimeInstance>>::Ok(std::move(instance));
}

Result<std::shared_ptr<RuntimeService::RuntimeInstance>>
RuntimeService::BuildInstance(const BootstrapRequest& request,
                              const ConfigCandidate& root_candidate,
                              uint64_t epoch, uint64_t initial_runtime_revision,
                              uint64_t initial_config_revision) const {
  if (root_candidate.domain != ConfigDomain::kGlobal ||
      root_candidate.selected_document || root_candidate.document_json.empty()) {
    return Result<std::shared_ptr<RuntimeInstance>>::Failure(Error::InvalidArgument(
        "root replacement requires a global in-memory canonical candidate"));
  }
  auto loaded = LoadBootstrapConfigCandidate(request.config_directory,
                                              root_candidate.document_json);
  return loaded ? BuildInstance(request, std::move(loaded).Value(), epoch,
                                initial_runtime_revision,
                                initial_config_revision)
                : Result<std::shared_ptr<RuntimeInstance>>::Failure(
                      loaded.GetError());
}

Result<std::shared_ptr<RuntimeService::RuntimeInstance>> RuntimeService::Active()
    const {
  std::lock_guard lock(mutex_);
  if (!active_) {
    return Result<std::shared_ptr<RuntimeInstance>>::Failure(
        Error::InvalidArgument("single runtime is not open"));
  }
  return Result<std::shared_ptr<RuntimeInstance>>::Ok(active_);
}

Result<RuntimeService::OperationLease> RuntimeService::AcquireOperation(
    bool require_ready) const {
  // The service lock must remain held until the operation count is acquired.
  // Otherwise replacement can observe zero in-flight operations in the gap.
  std::unique_lock runtime_lock(mutex_);
  if (lifecycle_ != LifecycleState::kReady || !active_) {
    return Result<OperationLease>::Failure(Error::InvalidArgument(
        lifecycle_ == LifecycleState::kReplacing
            ? "runtime replacement is in progress"
            : "single runtime is not ready for operations"));
  }
  const auto instance = active_;
  std::unique_lock instance_lock(instance->mutex);
  if (lifecycle_ != LifecycleState::kReady || active_ != instance) {
    return Result<OperationLease>::Failure(
        Error::InvalidArgument("runtime changed while acquiring operation"));
  }
  const auto pipeline = instance->controller->Snapshot();
  const auto fatal_runtime_error = instance->controller->FatalRuntimeError();
  instance->state = DeriveState(*instance, pipeline, fatal_runtime_error);
  if (instance->closing ||
      (require_ready && instance->state != RuntimeStatus::kReady &&
       instance->state != RuntimeStatus::kFailedRecoverable)) {
    if (fatal_runtime_error) {
      return Result<OperationLease>::Failure(*fatal_runtime_error);
    }
    return Result<OperationLease>::Failure(
        Error::InvalidArgument("runtime is not ready for this operation"));
  }
  ++instance->operations_in_progress;
  return Result<OperationLease>::Ok(OperationLease(instance, instance->epoch));
}

void RuntimeService::FinishTransitionLocked(uint64_t generation,
                                            LifecycleState next) {
  if (transition_generation_ != generation) return;
  lifecycle_ = next;
  transition_cancellation_.reset();
  lifecycle_changed_.notify_all();
}

Result<void> RuntimeService::AttachEventSource(
    const std::shared_ptr<RuntimeInstance>& instance) {
  auto source = instance->controller->SubscribeEvents(
      [this, weak = std::weak_ptr<RuntimeInstance>(instance)](
          const ExecutionEvent& event) {
        if (auto locked = weak.lock()) DispatchEvent(locked, event);
      });
  instance->event_source = std::move(source);
  return Result<void>::Ok();
}

Result<void> RuntimeService::Open(const BootstrapRequest& request) {
  uint64_t generation = 0;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(mutex_);
    if (lifecycle_ != LifecycleState::kClosed || active_) {
      return Result<void>::Failure(Error::InvalidArgument(
          "single runtime is already open or a transition is in progress"));
    }
    generation = ++transition_generation_;
    lifecycle_ = LifecycleState::kOpening;
    cancellation = std::make_shared<CancellationToken>();
    transition_cancellation_ = cancellation;
  }
  const auto fail = [&](const Error& error) {
    std::lock_guard lock(mutex_);
    FinishTransitionLocked(generation, LifecycleState::kClosed);
    return Result<void>::Failure(error);
  };
  auto loaded = LoadBootstrapConfig(request.config_directory);
  if (!loaded) return fail(loaded.GetError());
  if (cancellation->IsCancellationRequested()) {
    return fail(Error::Cancelled("runtime opening was cancelled"));
  }
  const uint64_t next_epoch = [&] {
    std::lock_guard lock(mutex_);
    return epoch_ + 1;
  }();
  auto built = BuildInstance(request, std::move(loaded).Value(), next_epoch);
  if (!built) return fail(built.GetError());
  const auto candidate = built.Value();
  // shared_ptr copy and pointer guard construction are noexcept. BuildInstance
  // can therefore hand off its inner rollback owner without an allocation gap.
  UnpublishedOutputRollback candidate_output{&candidate->output_directory};
  const auto fail_candidate = [&](const Error& error) {
    candidate->event_source.Reset();
    candidate_output.RollbackNow();
    return fail(error);
  };
  auto attached = AttachEventSource(candidate);
  if (!attached) return fail_candidate(attached.GetError());
  std::unique_lock lock(mutex_);
  if (transition_generation_ != generation || lifecycle_ != LifecycleState::kOpening ||
      cancellation->IsCancellationRequested() || active_) {
    lock.unlock();
    return fail_candidate(Error::Cancelled("runtime opening was cancelled"));
  }
  active_ = candidate;
  epoch_ = next_epoch;
  recent_public_events_.clear();
  candidate_output.Publish();
  FinishTransitionLocked(generation, LifecycleState::kReady);
  const auto published = active_;
  lock.unlock();
  try {
    if (const auto recovery = published->controller->FatalRuntimeError()) {
      ExecutionEvent event{0, EventType::kArtifactInvalidated, std::nullopt,
                           recovery->Message()};
      event.error = recovery;
      DispatchEvent(published, event);
    }
  } catch (...) {
    LogWarning("runtime open recovery event publication failed");
  }
  return Result<void>::Ok();
}

Result<void> RuntimeService::Open(const BootstrapRequest& request,
                                  const ConfigCandidate& root_candidate) {
  uint64_t generation = 0;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(mutex_);
    if (lifecycle_ != LifecycleState::kClosed || active_) {
      return Result<void>::Failure(Error::InvalidArgument(
          "single runtime is already open or a transition is in progress"));
    }
    generation = ++transition_generation_;
    lifecycle_ = LifecycleState::kOpening;
    cancellation = std::make_shared<CancellationToken>();
    transition_cancellation_ = cancellation;
  }
  const auto fail = [&](const Error& error) {
    std::lock_guard lock(mutex_);
    FinishTransitionLocked(generation, LifecycleState::kClosed);
    return Result<void>::Failure(error);
  };
  const uint64_t next_epoch = [&] {
    std::lock_guard lock(mutex_);
    return epoch_ + 1;
  }();
  auto built = BuildInstance(request, root_candidate, next_epoch);
  if (!built) return fail(built.GetError());
  const auto candidate = built.Value();
  // See the file-backed Open overload: ownership transfer after BuildInstance
  // contains no fallible path copy.
  UnpublishedOutputRollback candidate_output{&candidate->output_directory};
  const auto fail_candidate = [&](const Error& error) {
    candidate->event_source.Reset();
    candidate_output.RollbackNow();
    return fail(error);
  };
  if (cancellation->IsCancellationRequested()) {
    return fail_candidate(Error::Cancelled("runtime opening was cancelled"));
  }
  auto attached = AttachEventSource(candidate);
  if (!attached) return fail_candidate(attached.GetError());
  std::unique_lock lock(mutex_);
  if (transition_generation_ != generation || lifecycle_ != LifecycleState::kOpening ||
      cancellation->IsCancellationRequested() || active_) {
    lock.unlock();
    return fail_candidate(Error::Cancelled("runtime opening was cancelled"));
  }
  active_ = candidate;
  epoch_ = next_epoch;
  recent_public_events_.clear();
  candidate_output.Publish();
  FinishTransitionLocked(generation, LifecycleState::kReady);
  const auto published = active_;
  lock.unlock();
  try {
    if (const auto recovery = published->controller->FatalRuntimeError()) {
      ExecutionEvent event{0, EventType::kArtifactInvalidated, std::nullopt,
                           recovery->Message()};
      event.error = recovery;
      DispatchEvent(published, event);
    }
  } catch (...) {
    LogWarning("runtime open recovery event publication failed");
  }
  return Result<void>::Ok();
}

Result<void> RuntimeService::StageRootConfig(
    const RuntimeInstance& candidate, PendingOutputSet& pending) const {
  return StageConfigFile(candidate.bootstrap_config->ConfigDirectory() /
                             "config.json",
                         candidate.bootstrap_config->Root().CanonicalJson(),
                         pending);
}

/*
 * Root replacement reserves kReplacing before candidate preparation.  The
 * staged file is committed only while that reservation is still authoritative;
 * after Commit() succeeds the pointer swap below contains no fallible work.
 */
Result<RuntimeReplaceReceipt> RuntimeService::ReplaceRootConfig(
    const BootstrapRequest& request, const ConfigCandidate& root_candidate,
    const ExpectedRevision& expected) {
  std::shared_ptr<RuntimeInstance> previous;
  uint64_t generation = 0;
  uint64_t next_epoch = 0;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::unique_lock lock(mutex_);
    if (lifecycle_ != LifecycleState::kReady || !active_) {
      return Result<RuntimeReplaceReceipt>::Failure(Error::InvalidArgument(
          "single runtime is not ready for root replacement"));
    }
    previous = active_;
    if (previous->controller->IsInEventCallback()) {
      return Result<RuntimeReplaceReceipt>::Failure(Error::InvalidArgument(
          "cannot replace runtime from its event callback"));
    }
    std::unique_lock instance_lock(previous->mutex);
    const auto pipeline = previous->controller->Snapshot();
    previous->state = DeriveState(*previous, pipeline);
    if (previous->operations_in_progress != 0 ||
        (pipeline.job && IsActive(pipeline.job->state))) {
      return Result<RuntimeReplaceReceipt>::Failure(Error::InvalidArgument(
          "cannot replace runtime while a job or operation is active"));
    }
    if (pipeline.runtime_revision != expected.runtime_revision ||
        pipeline.config_revision != expected.config_revision) {
      return Result<RuntimeReplaceReceipt>::Failure(Error::InvalidArgument(
          "root replacement expected revision does not match committed runtime"));
    }
    generation = ++transition_generation_;
    next_epoch = epoch_ + 1;
    lifecycle_ = LifecycleState::kReplacing;
    cancellation = std::make_shared<CancellationToken>();
    transition_cancellation_ = cancellation;
  }
  const auto abort = [&](const Error& error) {
    std::lock_guard lock(mutex_);
    if (transition_generation_ == generation) {
      // Close may have requested cancellation while preparation ran.  The
      // previous runtime remains active and Close will continue from kReady.
      FinishTransitionLocked(generation, LifecycleState::kReady);
    }
    return Result<RuntimeReplaceReceipt>::Failure(error);
  };
  auto callbacks_idle = previous->controller->WaitForEventCallbacks();
  if (!callbacks_idle) return abort(callbacks_idle.GetError());
  if (cancellation->IsCancellationRequested()) {
    return abort(Error::Cancelled("runtime replacement was cancelled"));
  }
  auto built = BuildInstance(request, root_candidate, next_epoch,
                             expected.runtime_revision + 1,
                             expected.config_revision + 1);
  if (!built) return abort(built.GetError());
  const auto candidate = built.Value();
  UnpublishedOutputRollback candidate_output{&candidate->output_directory};
  const auto abort_candidate = [&](const Error& error) {
    candidate->event_source.Reset();
    candidate_output.RollbackNow();
    return abort(error);
  };
  if (cancellation->IsCancellationRequested()) {
    return abort_candidate(
        Error::Cancelled("runtime replacement was cancelled"));
  }
  const auto candidate_snapshot = candidate->controller->Snapshot();
  if (candidate_snapshot.runtime_revision != expected.runtime_revision + 1 ||
      candidate_snapshot.config_revision != expected.config_revision + 1) {
    return abort_candidate(Error::InvalidArgument(
        "replacement candidate did not preserve monotonic revisions"));
  }
  std::string previous_recovery_message;
  std::string previous_recovery_pointer;
  try {
    const auto candidate_authority = candidate->port->Snapshot();
    if (candidate_authority.recovery_required) {
      previous_recovery_message =
          candidate_authority.recovery_required->Message();
      previous_recovery_pointer = "/previous_recovery_required";
    }
  } catch (const std::exception& error) {
    return abort_candidate(Error::InvalidArgument(
        std::string("replacement candidate query failed before commit: ") +
        error.what()));
  } catch (...) {
    return abort_candidate(Error::InvalidArgument(
        "replacement candidate query failed before commit"));
  }
  auto attached = AttachEventSource(candidate);
  if (!attached) return abort_candidate(attached.GetError());
  PendingOutputSet pending;
  auto staged = StageRootConfig(*candidate, pending);
  if (!staged) {
    return abort_candidate(staged.GetError());
  }
  // Reserve the recovery payload before the file commit. After Commit()
  // succeeds, health publication and the active pointer swap only move
  // preallocated shared ownership.
  auto root_recovery = std::make_shared<Error>(
      Error::IoError("root config recovery outcome"));
  std::shared_ptr<const Error> committed_recovery;
  {
    std::unique_lock lock(mutex_);
    if (transition_generation_ != generation || lifecycle_ != LifecycleState::kReplacing ||
        cancellation->IsCancellationRequested() || active_ != previous) {
      lock.unlock();
      return abort_candidate(
          Error::Cancelled("runtime replacement was cancelled"));
    }
    auto committed = pending.Commit();
    if (!committed) {
      lock.unlock();
      return abort_candidate(committed.GetError());
    }
    if (committed.Value().recovery_required) {
      *root_recovery =
          std::move(*committed.Value().recovery_required);
      root_recovery->MarkFatalRuntime().WithRuntimeRevision(
          expected.runtime_revision + 1);
      if (!previous_recovery_message.empty()) {
        // Preserve a bootstrap cleanup fault when the root file-set also
        // needs recovery. Moving the precomputed message is allocation-free
        // after the root commit and keeps both manifests in structured Error
        // data instead of silently overwriting the first outcome.
        root_recovery->context.actual =
            std::move(previous_recovery_message);
        root_recovery->context.json_pointer =
            std::move(previous_recovery_pointer);
      }
      committed_recovery = root_recovery;
      // The internal hook is noexcept, non-blocking, and callback-free. Keep
      // the lifecycle lock from file commit through health publication and
      // active pointer swap so no query can observe new disk with old runtime.
      candidate->port->RecordRecoveryRequired(committed_recovery);
      candidate->controller->RecordRecoveryRequired(committed_recovery);
    }
    active_ = candidate;
    epoch_ = next_epoch;
    ClearPublicJobsLocked();
    recent_public_events_.clear();
    candidate_output.Publish();
    FinishTransitionLocked(generation, LifecycleState::kReady);
  }
  previous->event_source.Reset();
  try {
    if (const auto recovery = candidate->controller->FatalRuntimeError()) {
      ExecutionEvent event{0, EventType::kArtifactInvalidated, std::nullopt,
                           recovery->Message()};
      event.error = recovery;
      DispatchEvent(candidate, event);
    }
  } catch (...) {
    LogWarning("root replacement recovery event publication failed");
  }
  return Result<RuntimeReplaceReceipt>::Ok(
      {expected.runtime_revision, expected.config_revision,
       expected.runtime_revision + 1, expected.config_revision + 1});
}

uint64_t RuntimeService::MapPublicJobLocked(uint64_t epoch, JobId local_job) {
  const auto key = std::make_pair(epoch, local_job);
  if (const auto found = public_job_ids_.find(key); found != public_job_ids_.end()) {
    return found->second;
  }
  uint64_t handle = next_public_job_++;
  if (pending_public_job_ && pending_public_job_->first == epoch) {
    handle = pending_public_job_->second;
  }
  public_job_ids_.emplace(key, handle);
  public_jobs_.insert_or_assign(handle, PublicJob{epoch, local_job});
  return handle;
}

void RuntimeService::MarkTerminalPublicJobLocked(uint64_t handle) {
  constexpr size_t kMaxRetainedTerminalJobs = 256;
  if (std::find(terminal_public_jobs_.begin(), terminal_public_jobs_.end(),
                handle) == terminal_public_jobs_.end()) {
    terminal_public_jobs_.push_back(handle);
  }
  while (terminal_public_jobs_.size() > kMaxRetainedTerminalJobs) {
    const uint64_t expired = terminal_public_jobs_.front();
    terminal_public_jobs_.pop_front();
    const auto job = public_jobs_.find(expired);
    if (job == public_jobs_.end()) continue;
    public_job_ids_.erase(std::make_pair(job->second.epoch, job->second.local_job));
    public_jobs_.erase(job);
  }
}

void RuntimeService::ClearPublicJobsLocked() {
  public_job_ids_.clear();
  public_jobs_.clear();
  terminal_public_jobs_.clear();
  pending_public_job_.reset();
}

Result<JobHandle> RuntimeService::Submit(const ExecutionRequest& request) {
  std::lock_guard submit_lock(submit_mutex_);
  auto lease = AcquireOperation(true);
  if (!lease) return Result<JobHandle>::Failure(lease.GetError());
  uint64_t handle = 0;
  {
    std::lock_guard lock(mutex_);
    handle = next_public_job_++;
    pending_public_job_ = std::make_pair(lease.Value().epoch, handle);
  }
  Result<JobId> submitted = Result<JobId>::Failure(
      Error::InvalidArgument("invalid execution request"));
  auto& controller = lease.Value().instance->controller;
  switch (request.kind) {
    case ExecutionRequestKind::kRunAll: submitted = controller->SubmitRunAll(); break;
    case ExecutionRequestKind::kStage:
      if (request.stage) submitted = controller->SubmitStage(*request.stage);
      break;
    case ExecutionRequestKind::kNode:
      if (request.node) submitted = controller->SubmitNode(*request.node, request.agent);
      break;
    case ExecutionRequestKind::kOptimizeThrough:
      if (request.agent) submitted = controller->SubmitOptimizeThrough(*request.agent);
      break;
  }
  {
    std::lock_guard lock(mutex_);
    if (submitted) {
      const auto key = std::make_pair(lease.Value().epoch, submitted.Value());
      public_job_ids_.insert_or_assign(key, handle);
      public_jobs_.insert_or_assign(handle,
                                    PublicJob{lease.Value().epoch, submitted.Value()});
    }
    if (pending_public_job_ && pending_public_job_->first == lease.Value().epoch &&
        pending_public_job_->second == handle) {
      pending_public_job_.reset();
    }
  }
  return submitted ? Result<JobHandle>::Ok({handle})
                   : Result<JobHandle>::Failure(submitted.GetError());
}

Result<RuntimeService::PublicJob> RuntimeService::ResolveJob(JobHandle job)
    const {
  std::lock_guard lock(mutex_);
  const auto found = public_jobs_.find(job.value);
  if (found == public_jobs_.end()) {
    return Result<PublicJob>::Failure(
        Error::InvalidArgument("job handle is unknown or expired"));
  }
  if (!active_ || found->second.epoch != active_->epoch) {
    return Result<PublicJob>::Failure(Error::InvalidArgument(
        "job belongs to a retired runtime epoch"));
  }
  return Result<PublicJob>::Ok(found->second);
}

Result<void> RuntimeService::Cancel(JobHandle job) {
  auto mapped = ResolveJob(job);
  if (!mapped) return Result<void>::Failure(mapped.GetError());
  auto lease = AcquireOperation(false);
  if (!lease) return Result<void>::Failure(lease.GetError());
  if (lease.Value().epoch != mapped.Value().epoch) {
    return Result<void>::Failure(Error::InvalidArgument(
        "job belongs to a retired runtime epoch"));
  }
  return lease.Value().instance->controller->Cancel(mapped.Value().local_job);
}

Result<void> RuntimeService::Wait(JobHandle job) {
  auto mapped = ResolveJob(job);
  if (!mapped) return Result<void>::Failure(mapped.GetError());
  auto lease = AcquireOperation(false);
  if (!lease) return Result<void>::Failure(lease.GetError());
  if (lease.Value().epoch != mapped.Value().epoch) {
    return Result<void>::Failure(Error::InvalidArgument(
        "job belongs to a retired runtime epoch"));
  }
  return lease.Value().instance->controller->Wait(mapped.Value().local_job);
}

Result<RuntimeSnapshot> RuntimeService::Snapshot() const {
  auto active = Active();
  if (!active) return Result<RuntimeSnapshot>::Failure(active.GetError());
  auto instance = active.Value();
  auto pipeline = instance->controller->Snapshot();
  const auto fatal_runtime_error = instance->controller->FatalRuntimeError();
  {
    std::lock_guard instance_lock(instance->mutex);
    instance->state = DeriveState(*instance, pipeline, fatal_runtime_error);
  }
  {
    std::lock_guard lock(mutex_);
    if (active_ != instance) {
      return Result<RuntimeSnapshot>::Failure(Error::InvalidArgument(
          "runtime changed during snapshot"));
    }
    if (pipeline.job) {
      const auto found = public_job_ids_.find(
          std::make_pair(instance->epoch,
                         static_cast<JobId>(pipeline.job->id)));
      if (found != public_job_ids_.end()) pipeline.job->id = found->second;
    }
    pipeline.recent_events.assign(recent_public_events_.begin(),
                                  recent_public_events_.end());
  }
  return Result<RuntimeSnapshot>::Ok(
      {instance->label, instance->state, instance->output_directory,
       std::move(pipeline)});
}

Result<std::vector<NodeDescriptor>> RuntimeService::NodeDescriptors() const {
  auto lease = AcquireOperation(false);
  return lease ? Result<std::vector<NodeDescriptor>>::Ok(
                     lease.Value().instance->controller->NodeDescriptors())
               : Result<std::vector<NodeDescriptor>>::Failure(lease.GetError());
}

Result<CommittedConfigDocuments> RuntimeService::ConfigDocuments() const {
  auto lease = AcquireOperation(false);
  return lease ? lease.Value().instance->port->ConfigDocuments()
               : Result<CommittedConfigDocuments>::Failure(lease.GetError());
}

Result<ConfigCandidateCatalog> RuntimeService::ConfigCandidates() const {
  auto lease = AcquireOperation(false);
  return lease ? lease.Value().instance->port->ConfigCandidates()
               : Result<ConfigCandidateCatalog>::Failure(lease.GetError());
}

Result<std::vector<std::string>> RuntimeService::RecentLogs(
    std::size_t max_lines) const {
  if (max_lines == 0 || max_lines > 512) {
    return Result<std::vector<std::string>>::Failure(
        Error::InvalidArgument("max_lines must be between 1 and 512"));
  }
  auto lease = AcquireOperation(false);
  return lease ? Result<std::vector<std::string>>::Ok(
                     RecentRuntimeLogs(max_lines))
               : Result<std::vector<std::string>>::Failure(lease.GetError());
}

Result<VisualizationSnapshot> RuntimeService::Visualization(
    const AgentId& agent) const {
  return Visualization(VisualizationQuery{agent});
}

Result<VisualizationSnapshot> RuntimeService::Visualization(
    const VisualizationQuery& query) const {
  auto lease = AcquireOperation(false);
  return lease ? lease.Value().instance->controller->GetVisualizationSnapshot(query)
               : Result<VisualizationSnapshot>::Failure(lease.GetError());
}

Result<std::optional<AlignmentFeedbackSnapshot>> RuntimeService::AlignmentFeedback()
    const {
  auto lease = AcquireOperation(false);
  return lease ? Result<std::optional<AlignmentFeedbackSnapshot>>::Ok(
                     lease.Value().instance->controller->GetAlignmentFeedbackSnapshot())
               : Result<std::optional<AlignmentFeedbackSnapshot>>::Failure(
                     lease.GetError());
}

Result<void> RuntimeService::RespondToAlignment(JobHandle job,
                                                AlignmentResponse response) {
  auto mapped = ResolveJob(job);
  if (!mapped) return Result<void>::Failure(mapped.GetError());
  auto lease = AcquireOperation(false);
  if (!lease) return Result<void>::Failure(lease.GetError());
  if (lease.Value().epoch != mapped.Value().epoch) {
    return Result<void>::Failure(Error::InvalidArgument(
        "job belongs to a retired runtime epoch"));
  }
  return lease.Value().instance->controller->RespondToAlignment(
      mapped.Value().local_job, std::move(response));
}

Result<void> RuntimeService::SetAlignmentFeedbackEnabled(bool enabled) {
  auto lease = AcquireOperation(false);
  if (!lease) return Result<void>::Failure(lease.GetError());
  {
    std::lock_guard lock(mutex_);
    if (lifecycle_ != LifecycleState::kReady || active_ != lease.Value().instance) {
      return Result<void>::Failure(
          Error::InvalidArgument("runtime changed while setting feedback availability"));
    }
    feedback_enabled_ = enabled;
  }
  lease.Value().instance->controller->SetAlignmentFeedbackEnabled(enabled);
  return Result<void>::Ok();
}

Result<ConfigApplyReceipt> RuntimeService::ApplyConfig(
    const ConfigCandidate& candidate, const ExpectedRevision& expected) {
  auto lease = AcquireOperation(true);
  if (!lease) return Result<ConfigApplyReceipt>::Failure(lease.GetError());
  return lease.Value().instance->controller->ApplyConfig(candidate, expected);
}

void RuntimeService::DispatchEvent(const std::shared_ptr<RuntimeInstance>& instance,
                                   const ExecutionEvent& source) {
  std::vector<std::shared_ptr<SubscriberSlot>> targets;
  ExecutionEvent event = source;
  {
    std::lock_guard lock(mutex_);
    if (active_ != instance) return;
    if (event.job_id != 0) {
      event.job_id = MapPublicJobLocked(instance->epoch, event.job_id);
      if (event.type == EventType::kJobCompleted ||
          event.type == EventType::kJobCancelled) {
        MarkTerminalPublicJobLocked(event.job_id);
      }
    }
    event.sequence = next_public_event_sequence_++;
    recent_public_events_.push_back(event);
    if (recent_public_events_.size() > 256) recent_public_events_.pop_front();
  }
  const auto subscribers = subscribers_;
  if (!subscribers) return;
  {
    std::lock_guard lock(subscribers->mutex);
    if (subscribers->closing) return;
    for (const auto& [id, slot] : subscribers->slots) {
      (void)id;
      targets.push_back(slot);
    }
  }
  for (const auto& slot : targets) {
    std::function<void(const ExecutionEvent&)> callback;
    {
      std::lock_guard lock(slot->mutex);
      if (!slot->active) continue;
      ++slot->callbacks_in_flight;
      callback = slot->callback;
    }
    {
      SubscriberCallbackScope scope(slot.get());
      try {
        callback(event);
      } catch (const std::exception& error) {
        LogWarning(std::string("runtime event observer failed: ") + error.what());
      } catch (...) {
        LogWarning("runtime event observer failed with unknown exception");
      }
    }
    {
      std::lock_guard lock(slot->mutex);
      --slot->callbacks_in_flight;
      if (slot->callbacks_in_flight == 0) slot->idle.notify_all();
    }
  }
}

Result<ExecutionEventSubscription> RuntimeService::SubscribeEvents(
    std::function<void(const ExecutionEvent&)> callback) {
  if (!callback) {
    return Result<ExecutionEventSubscription>::Failure(
        Error::InvalidArgument("event callback must not be empty"));
  }
  auto active = Active();
  if (!active) return Result<ExecutionEventSubscription>::Failure(active.GetError());
  auto slot = std::make_shared<SubscriberSlot>();
  slot->callback = std::move(callback);
  const auto subscribers = subscribers_;
  if (!subscribers) {
    return Result<ExecutionEventSubscription>::Failure(
        Error::InvalidArgument("runtime event subscriptions are closed"));
  }
  uint64_t id = 0;
  {
    std::lock_guard lock(mutex_);
    if (active_ != active.Value()) {
      return Result<ExecutionEventSubscription>::Failure(Error::InvalidArgument(
          "runtime changed while subscribing"));
    }
    std::lock_guard subscribers_lock(subscribers->mutex);
    if (subscribers->closing) {
      return Result<ExecutionEventSubscription>::Failure(
          Error::InvalidArgument("runtime event subscriptions are closed"));
    }
    id = next_subscriber_id_++;
    subscribers->slots.emplace(id, slot);
  }
  return Result<ExecutionEventSubscription>::Ok(ExecutionEventSubscription(
      [weak_registry = std::weak_ptr<SubscriberRegistry>(subscribers),
       weak = std::weak_ptr<SubscriberSlot>(slot), id] {
        auto slot = weak.lock();
        if (!slot) return;
        if (auto registry = weak_registry.lock()) {
          std::lock_guard lock(registry->mutex);
          registry->slots.erase(id);
        }
        std::unique_lock lock(slot->mutex);
        slot->active = false;
        if (SubscriberCallbackScope::active_ != slot.get()) {
          slot->idle.wait(lock, [&] { return slot->callbacks_in_flight == 0; });
        }
      }));
}

Result<void> RuntimeService::Close(CloseMode mode) {
  std::shared_ptr<RuntimeInstance> instance;
  std::optional<JobSnapshot> active_job;
  for (;;) {
    std::unique_lock runtime_lock(mutex_);
    if (lifecycle_ == LifecycleState::kClosed && !active_) {
      return Result<void>::Ok();
    }
    if (lifecycle_ == LifecycleState::kOpening) {
      if (transition_cancellation_) transition_cancellation_->Request();
      lifecycle_ = LifecycleState::kClosing;
      lifecycle_changed_.notify_all();
      lifecycle_changed_.wait(runtime_lock, [&] {
        return lifecycle_ != LifecycleState::kClosing;
      });
      continue;
    }
    if (lifecycle_ == LifecycleState::kReplacing) {
      if (transition_cancellation_) transition_cancellation_->Request();
      lifecycle_changed_.wait(runtime_lock, [&] {
        return lifecycle_ != LifecycleState::kReplacing;
      });
      continue;
    }
    if (lifecycle_ == LifecycleState::kClosing) {
      lifecycle_changed_.wait(runtime_lock, [&] {
        return lifecycle_ != LifecycleState::kClosing;
      });
      continue;
    }
    if (lifecycle_ != LifecycleState::kReady || !active_) {
      return Result<void>::Failure(
          Error::InvalidArgument("runtime lifecycle is not closable"));
    }
    instance = active_;
    if (instance->controller->IsInEventCallback()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "cannot close runtime from its event callback"));
    }
    std::unique_lock instance_lock(instance->mutex);
    const auto pipeline = instance->controller->Snapshot();
    instance->state = DeriveState(*instance, pipeline);
    if (pipeline.job && IsActive(pipeline.job->state)) active_job = pipeline.job;
    if (active_job && mode == CloseMode::kRejectIfRunning) {
      return Result<void>::Failure(Error::InvalidArgument("runtime has an active job"));
    }
    lifecycle_ = LifecycleState::kClosing;
    lifecycle_changed_.notify_all();
    instance->closing = true;
    instance->state = RuntimeStatus::kClosing;
    break;
  }
  // Cancel first. Waiting for in-flight Wait() before cancellation would
  // deadlock a job waiting for interactive alignment feedback.
  if (active_job) {
    (void)instance->controller->Cancel(active_job->id);
    (void)instance->controller->Wait(active_job->id);
  }
  {
    std::unique_lock lock(instance->mutex);
    instance->idle.wait(lock, [&] { return instance->operations_in_progress == 0; });
  }
  auto callbacks = instance->controller->WaitForEventCallbacks();
  if (!callbacks) {
    std::lock_guard lock(mutex_);
    if (active_ == instance) {
      std::lock_guard instance_lock(instance->mutex);
      instance->closing = false;
      lifecycle_ = LifecycleState::kReady;
      lifecycle_changed_.notify_all();
    }
    return callbacks;
  }
  instance->event_source.Reset();
  {
    std::lock_guard lock(mutex_);
    if (active_ == instance) {
      active_.reset();
      ClearPublicJobsLocked();
    }
    lifecycle_ = LifecycleState::kClosed;
    transition_cancellation_.reset();
    lifecycle_changed_.notify_all();
  }
  return Result<void>::Ok();
}

bool RuntimeService::IsOpen() const {
  std::lock_guard lock(mutex_);
  return static_cast<bool>(active_);
}

bool RuntimeService::IsInEventCallback() const {
  auto active = Active();
  return active && active.Value()->controller->IsInEventCallback();
}

RuntimeServiceDiagnostics RuntimeService::Diagnostics() const {
  RuntimeServiceDiagnostics diagnostics;
  std::shared_ptr<RuntimeInstance> instance;
  std::shared_ptr<SubscriberRegistry> subscribers;
  {
    std::lock_guard lock(mutex_);
    diagnostics.lifecycle_state = lifecycle_;
    diagnostics.active_epoch = active_ ? active_->epoch : epoch_;
    diagnostics.public_job_count = public_jobs_.size();
    diagnostics.terminal_job_count = terminal_public_jobs_.size();
    diagnostics.recent_event_count = recent_public_events_.size();
    instance = active_;
    subscribers = subscribers_;
  }
  if (instance) {
    std::lock_guard lock(instance->mutex);
    diagnostics.operation_lease_count = instance->operations_in_progress;
  }
  if (subscribers) {
    std::lock_guard registry_lock(subscribers->mutex);
    diagnostics.subscriber_count = subscribers->slots.size();
    for (const auto& [id, slot] : subscribers->slots) {
      (void)id;
      std::lock_guard slot_lock(slot->mutex);
      diagnostics.callbacks_in_flight += slot->callbacks_in_flight;
    }
  }
  return diagnostics;
}

void RuntimeService::WaitForLifecycleForDiagnostics(
    RuntimeLifecycleState expected) {
  std::unique_lock lock(mutex_);
  lifecycle_changed_.wait(lock, [&] { return lifecycle_ == expected; });
}

}  // namespace open_lmm
