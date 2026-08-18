#include "runtime_service.hpp"

#include <open_lmm/server/map_server.hpp>
#include <open_lmm/server/output_repository.hpp>
#include <open_lmm/utils/logging.hpp>

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

}  // namespace

struct RuntimeService::SubscriberSlot {
  std::mutex mutex;
  std::condition_variable idle;
  bool active = true;
  std::size_t callbacks_in_flight = 0;
  std::function<void(const ExecutionEvent&)> callback;
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
    : governor_(std::make_shared<ResourceGovernor>(budget)),
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

RuntimeService::~RuntimeService() { (void)Close(CloseMode::kCancelAndWait); }

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
    const RuntimeInstance& instance, const PipelineSnapshot& pipeline) {
  if (instance.closing) return RuntimeStatus::kClosing;
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
  instance->controller->SetAlignmentFeedbackEnabled(true);
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
  auto active = Active();
  if (!active) return Result<OperationLease>::Failure(active.GetError());
  const auto instance = active.Value();
  {
    std::lock_guard runtime_lock(mutex_);
    if (replacement_in_progress_ || active_ != instance) {
      return Result<OperationLease>::Failure(
          Error::InvalidArgument("runtime replacement is in progress"));
    }
  }
  std::lock_guard instance_lock(instance->mutex);
  const auto pipeline = instance->controller->Snapshot();
  instance->state = DeriveState(*instance, pipeline);
  if (instance->closing ||
      (require_ready && instance->state != RuntimeStatus::kReady &&
       instance->state != RuntimeStatus::kFailedRecoverable)) {
    return Result<OperationLease>::Failure(
        Error::InvalidArgument("runtime is not ready for this operation"));
  }
  ++instance->operations_in_progress;
  return Result<OperationLease>::Ok(OperationLease(instance, instance->epoch));
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
  auto loaded = LoadBootstrapConfig(request.config_directory);
  if (!loaded) return Result<void>::Failure(loaded.GetError());
  std::unique_lock lock(mutex_);
  if (active_ || replacement_in_progress_) {
    return Result<void>::Failure(Error::InvalidArgument(
        "single runtime is already open or being replaced"));
  }
  const uint64_t next_epoch = epoch_ + 1;
  lock.unlock();
  auto built = BuildInstance(request, std::move(loaded).Value(), next_epoch);
  if (!built) return Result<void>::Failure(built.GetError());
  auto attached = AttachEventSource(built.Value());
  if (!attached) return attached;
  lock.lock();
  if (active_ || replacement_in_progress_) {
    lock.unlock();
    built.Value()->event_source.Reset();
    return Result<void>::Failure(Error::InvalidArgument(
        "single runtime changed while opening"));
  }
  active_ = built.Value();
  epoch_ = next_epoch;
  return Result<void>::Ok();
}

Result<void> RuntimeService::Open(const BootstrapRequest& request,
                                  const ConfigCandidate& root_candidate) {
  std::unique_lock lock(mutex_);
  if (active_ || replacement_in_progress_) {
    return Result<void>::Failure(Error::InvalidArgument(
        "single runtime is already open or being replaced"));
  }
  const uint64_t next_epoch = epoch_ + 1;
  lock.unlock();
  auto built = BuildInstance(request, root_candidate, next_epoch);
  if (!built) return Result<void>::Failure(built.GetError());
  auto attached = AttachEventSource(built.Value());
  if (!attached) return attached;
  lock.lock();
  if (active_ || replacement_in_progress_) {
    lock.unlock();
    built.Value()->event_source.Reset();
    return Result<void>::Failure(Error::InvalidArgument(
        "single runtime changed while opening"));
  }
  active_ = built.Value();
  epoch_ = next_epoch;
  return Result<void>::Ok();
}

Result<void> RuntimeService::InstallRootConfig(
    const RuntimeInstance& candidate) const {
  OutputRepository outputs;
  auto pending = outputs.Begin();
  auto staged = StageConfigFile(candidate.bootstrap_config->ConfigDirectory() /
                                    "config.json",
                                candidate.bootstrap_config->Root().CanonicalJson(),
                                pending);
  if (!staged) return Result<void>::Failure(staged.GetError());
  return pending.Commit();
}

Result<RuntimeReplaceReceipt> RuntimeService::ReplaceRootConfig(
    const BootstrapRequest& request, const ConfigCandidate& root_candidate,
    const ExpectedRevision& expected) {
  auto current = Active();
  if (!current) {
    return Result<RuntimeReplaceReceipt>::Failure(current.GetError());
  }
  const auto previous = current.Value();
  if (previous->controller->IsInEventCallback()) {
    return Result<RuntimeReplaceReceipt>::Failure(Error::InvalidArgument(
        "cannot replace runtime from its event callback"));
  }
  {
    std::lock_guard lock(mutex_);
    if (replacement_in_progress_ || active_ != previous) {
      return Result<RuntimeReplaceReceipt>::Failure(Error::InvalidArgument(
          "runtime replacement is already in progress"));
    }
    std::lock_guard instance_lock(previous->mutex);
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
    replacement_in_progress_ = true;
  }
  struct ReplacementGuard {
    RuntimeService& service;
    bool committed = false;
    ~ReplacementGuard() {
      if (committed) return;
      std::lock_guard lock(service.mutex_);
      service.replacement_in_progress_ = false;
    }
  } guard{*this};
  auto callbacks_idle = previous->controller->WaitForEventCallbacks();
  if (!callbacks_idle) {
    return Result<RuntimeReplaceReceipt>::Failure(callbacks_idle.GetError());
  }
  uint64_t next_epoch = 0;
  {
    std::lock_guard lock(mutex_);
    next_epoch = epoch_ + 1;
  }
  auto built = BuildInstance(request, root_candidate, next_epoch,
                             expected.runtime_revision + 1,
                             expected.config_revision + 1);
  if (!built) return Result<RuntimeReplaceReceipt>::Failure(built.GetError());
  auto candidate = built.Value();
  struct CandidateOutputRollback {
    fs::path path;
    bool committed = false;
    ~CandidateOutputRollback() {
      if (committed || path.filename().string().rfind("runtime-", 0) != 0) return;
      std::error_code ignored;
      fs::remove_all(path, ignored);
    }
  } candidate_output{candidate->output_directory};
  auto attached = AttachEventSource(candidate);
  if (!attached) return Result<RuntimeReplaceReceipt>::Failure(attached.GetError());
  auto installed = InstallRootConfig(*candidate);
  if (!installed) {
    candidate->event_source.Reset();
    return Result<RuntimeReplaceReceipt>::Failure(installed.GetError());
  }
  {
    std::lock_guard lock(mutex_);
    if (active_ != previous) {
      candidate->event_source.Reset();
      return Result<RuntimeReplaceReceipt>::Failure(Error::InvalidArgument(
          "runtime changed while replacement was prepared"));
    }
    const auto candidate_snapshot = candidate->controller->Snapshot();
    if (candidate_snapshot.runtime_revision != expected.runtime_revision + 1 ||
        candidate_snapshot.config_revision != expected.config_revision + 1) {
      candidate->event_source.Reset();
      return Result<RuntimeReplaceReceipt>::Failure(Error::InvalidArgument(
          "replacement candidate did not preserve monotonic revisions"));
    }
    active_ = candidate;
    epoch_ = next_epoch;
    replacement_in_progress_ = false;
  }
  previous->event_source.Reset();
  guard.committed = true;
  candidate_output.committed = true;
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
  if (found == public_jobs_.end() || !active_ ||
      found->second.epoch != active_->epoch) {
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
  {
    std::lock_guard instance_lock(instance->mutex);
    instance->state = DeriveState(*instance, pipeline);
  }
  {
    std::lock_guard lock(mutex_);
    if (active_ != instance) {
      return Result<RuntimeSnapshot>::Failure(Error::InvalidArgument(
          "runtime changed during snapshot"));
    }
    const auto translate = [&](uint64_t& local) {
      const auto found = public_job_ids_.find(
          std::make_pair(instance->epoch, static_cast<JobId>(local)));
      if (found != public_job_ids_.end()) local = found->second;
    };
    if (pipeline.job) translate(pipeline.job->id);
    for (auto& event : pipeline.recent_events) translate(event.job_id);
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

Result<VisualizationSnapshot> RuntimeService::Visualization(
    const AgentId& agent) const {
  auto lease = AcquireOperation(false);
  return lease ? lease.Value().instance->controller->GetVisualizationSnapshot(agent)
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
    }
    for (const auto& [id, slot] : subscribers_) {
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
  uint64_t id = 0;
  {
    std::lock_guard lock(mutex_);
    if (active_ != active.Value()) {
      return Result<ExecutionEventSubscription>::Failure(Error::InvalidArgument(
          "runtime changed while subscribing"));
    }
    id = next_subscriber_id_++;
    subscribers_.emplace(id, slot);
  }
  return Result<ExecutionEventSubscription>::Ok(ExecutionEventSubscription(
      [this, weak = std::weak_ptr<SubscriberSlot>(slot), id] {
        auto slot = weak.lock();
        if (!slot) return;
        {
          std::lock_guard lock(mutex_);
          subscribers_.erase(id);
        }
        std::unique_lock lock(slot->mutex);
        slot->active = false;
        if (SubscriberCallbackScope::active_ != slot.get()) {
          slot->idle.wait(lock, [&] { return slot->callbacks_in_flight == 0; });
        }
      }));
}

Result<void> RuntimeService::Close(CloseMode mode) {
  auto active = Active();
  if (!active) return Result<void>::Ok();
  auto instance = active.Value();
  if (instance->controller->IsInEventCallback()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "cannot close runtime from its event callback"));
  }
  std::optional<JobSnapshot> active_job;
  {
    std::lock_guard runtime_lock(mutex_);
    if (replacement_in_progress_ || active_ != instance) {
      return Result<void>::Failure(Error::InvalidArgument(
          "runtime replacement is in progress"));
    }
    std::lock_guard instance_lock(instance->mutex);
    const auto pipeline = instance->controller->Snapshot();
    instance->state = DeriveState(*instance, pipeline);
    if (pipeline.job && IsActive(pipeline.job->state)) active_job = pipeline.job;
    if (active_job && mode == CloseMode::kRejectIfRunning) {
      return Result<void>::Failure(Error::InvalidArgument("runtime has an active job"));
    }
    instance->closing = true;
    instance->state = RuntimeStatus::kClosing;
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
  if (!callbacks) return callbacks;
  instance->event_source.Reset();
  {
    std::lock_guard lock(mutex_);
    if (active_ == instance) active_.reset();
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

}  // namespace open_lmm
