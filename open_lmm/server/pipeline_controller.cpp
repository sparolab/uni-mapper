#include "pipeline_controller.hpp"

#include <algorithm>
#include <array>
#include <open_lmm/common/profiling.hpp>
#include <exception>
#include <map>

namespace open_lmm {

namespace {

std::string StageName(StageId stage) {
  switch (stage) {
    case StageId::kDataLoad: return "data_load";
    case StageId::kAlignment: return "alignment";
    case StageId::kMapUpdate: return "map_update";
    case StageId::kSave: return "save";
  }
  return "unknown";
}

Error AddExecutionContext(
    Error error, const std::shared_ptr<StageRunner>& runner, StageId stage,
    std::optional<NodeId> node = std::nullopt,
    std::optional<AgentId> agent = std::nullopt) {
  std::string node_name;
  if (node) node_name = std::string(DescribeNode(*node).name);
  error.WithExecution(StageName(stage), std::move(node_name), agent);
  try {
    if (runner) {
      auto session = runner->SessionSnapshot();
      if (session) error.WithSessionRevision(session->revision);
    }
  } catch (...) {
    // Preserve the original execution error if diagnostic snapshotting fails.
  }
  return error;
}

ExecutionEvent FailureEvent(uint64_t job_id, EventType type, StageId stage,
                            const Error& error,
                            std::optional<NodeId> node = std::nullopt,
                            std::optional<AgentId> agent = std::nullopt) {
  ExecutionEvent event{job_id, type, stage, error.Message(), 0, node, agent};
  event.error = error;
  return event;
}

}  // namespace

struct ExecutionEventSubscriberSlot {
  std::mutex mutex;
  std::condition_variable idle;
  bool active = true;
  std::size_t callbacks_in_flight = 0;
  std::function<void(const ExecutionEvent&)> callback;
};

struct ExecutionEventSubscriberRegistry {
  std::mutex mutex;
  uint64_t next_id = 1;
  std::map<uint64_t, std::shared_ptr<ExecutionEventSubscriberSlot>> callbacks;
};

thread_local const ExecutionEventSubscriberSlot* active_event_subscriber =
    nullptr;
thread_local const PipelineController* active_event_controller = nullptr;

ExecutionEventSubscription::ExecutionEventSubscription(
    std::function<void()> unsubscribe)
    : unsubscribe_(std::move(unsubscribe)) {}

ExecutionEventSubscription::~ExecutionEventSubscription() { Reset(); }

ExecutionEventSubscription::ExecutionEventSubscription(
    ExecutionEventSubscription&& other) noexcept
    : unsubscribe_(std::move(other.unsubscribe_)) {}

ExecutionEventSubscription& ExecutionEventSubscription::operator=(
    ExecutionEventSubscription&& other) noexcept {
  if (this != &other) {
    Reset();
    unsubscribe_ = std::move(other.unsubscribe_);
  }
  return *this;
}

void ExecutionEventSubscription::Reset() {
  if (!unsubscribe_) return;
  auto unsubscribe = std::move(unsubscribe_);
  unsubscribe();
}

PipelineController::PipelineController(std::shared_ptr<StageRunner> runner)
    : runner_(std::move(runner)),
      event_subscribers_(std::make_shared<ExecutionEventSubscriberRegistry>()),
      alignment_feedback_(std::make_shared<AlignmentFeedbackBroker>()) {
  alignment_feedback_->SetNotification(
      [this](const AlignmentFeedbackSnapshot& snapshot) {
        uint64_t job_id = 0;
        {
          std::lock_guard lock(mutex_);
          if (!job_) return;
          job_id = job_->id;
          job_->state = JobState::kWaitingForAlignmentFeedback;
          job_->message = "waiting for map alignment feedback";
        }
        emit({job_id, EventType::kAlignmentFeedbackRequested,
              StageId::kAlignment, "map alignment feedback requested", 0,
              NodeId::kLoopDetect, snapshot.proposal.source_agent});
      });
  if (runner_) {
    cancellation_capability_ = runner_->CancellationMetadata();
    runner_->SetAlignmentFeedbackBroker(alignment_feedback_);
    agents_ = runner_->AgentIds();
    artifacts_.RegisterAgents(agents_);
    synchronizeCommittedSession(runner_);
  }
}

bool PipelineController::synchronizeCommittedSession(
    const std::shared_ptr<StageRunner>& runner) {
  if (!runner) return false;
  auto session = runner->SessionSnapshot();
  if (!session) {
    runner_manages_artifacts_ = false;
    return false;
  }
  artifacts_.Reset(session->ordered_agents);
  artifacts_.Restore(session->artifacts);
  {
    std::lock_guard lock(mutex_);
    agents_ = session->ordered_agents;
    config_revision_ = session->config_revision;
  }
  runner_manages_artifacts_ = true;
  return true;
}

PipelineController::~PipelineController() {
  cancel_requested_ = true;
  if (cancellation_) cancellation_->Request();
  if (alignment_feedback_) alignment_feedback_->Cancel();
  if (worker_.joinable()) worker_.join();
}

Result<uint64_t> PipelineController::submit(Work work) {
  if (IsInEventCallback()) {
    return Result<uint64_t>::Failure(Error::InvalidArgument(
        "cannot submit a pipeline job from its event callback"));
  }
  std::thread finished_worker;
  std::shared_ptr<StageRunner> runner;
  uint64_t id;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard command_lock(command_mutex_);
    std::lock_guard state_lock(mutex_);
    if (!runner_) {
      return Result<uint64_t>::Failure(
          Error::InvalidArgument("pipeline stage runner is null"));
    }
    if (maintenance_in_progress_) {
      return Result<uint64_t>::Failure(
          Error::InvalidArgument("pipeline maintenance is in progress"));
    }
    if (job_ && (job_->state == JobState::kQueued ||
                 job_->state == JobState::kRunning ||
                 job_->state == JobState::kWaitingForAlignmentFeedback ||
                 job_->state == JobState::kCancelling)) {
      return Result<uint64_t>::Failure(
          Error::InvalidArgument("another pipeline job is already running"));
    }
    if (worker_.joinable()) finished_worker = std::move(worker_);
    runner = runner_;
    id = next_job_id_++;
    cancellation =
        std::make_shared<CancellationToken>(cancellation_capability_);
    job_ = JobSnapshot{id, JobState::kQueued, std::nullopt, {},
                       CancellationTelemetry{cancellation_capability_}};
    terminal_event_completed_job_id_ = 0;
    cancel_requested_ = false;
    cancellation_ = cancellation;
  }

  // Joining and StageRunner calls are deliberately outside both controller
  // mutexes. The queued job reserves the runner against replacement while the
  // external calls are in flight.
  if (finished_worker.joinable()) finished_worker.join();
  Result<void> setup = Result<void>::Ok();
  try {
    runner->SetCancellationToken(cancellation);
    runner->SetAlignmentFeedbackBroker(alignment_feedback_);
  } catch (const std::exception& error) {
    setup = Result<void>::Failure(Error::InvalidArgument(error.what()));
  } catch (...) {
    setup = Result<void>::Failure(
        Error::InvalidArgument("unknown runner setup exception"));
  }
  if (!setup) {
    commitTerminal(id, setup);
    return Result<uint64_t>::Failure(setup.GetError());
  }
  emit({id, EventType::kJobQueued, std::nullopt, {}});
  auto start_worker = std::make_shared<std::atomic<bool>>(false);
  auto work_result = std::make_shared<std::optional<Result<void>>>();
  std::thread execution_worker(
      [this, id, work = std::move(work), start_worker, cancellation,
       work_result]() mutable {
    while (!start_worker->load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
    OPEN_LMM_THREAD_NAME("open_lmm.pipeline");
    OPEN_LMM_ZONE_N("PipelineController.Job");
    OPEN_LMM_PLOT("job.id", id);
    {
      std::lock_guard lock(mutex_);
      if (job_ && job_->id == id && job_->state == JobState::kQueued) {
        job_->state = JobState::kRunning;
      }
    }
    emit({id, EventType::kJobStarted, std::nullopt, {}});
    Result<void> result = Result<void>::Ok();
    try {
      result = work(id);
    } catch (const std::exception& e) {
      result = Result<void>::Failure(Error::InvalidArgument(e.what()));
    } catch (...) {
      result = Result<void>::Failure(
          Error::InvalidArgument("unknown pipeline exception"));
    }
    if (result && cancellation->IsCancellationRequested()) {
      result = Result<void>::Failure(
          Error::Cancelled("observed after runner operation returned"));
    }
    work_result->emplace(std::move(result));
  });
  std::thread lifecycle_worker(
      [this, id, cancellation, work_result,
       execution_worker = std::move(execution_worker)]() mutable {
        execution_worker.join();
        cancellation->Complete();
        {
          std::lock_guard lock(mutex_);
          if (job_ && job_->id == id) {
            job_->cancellation = cancellation->Telemetry();
          }
        }
        commitTerminal(id, **work_result);
      });
  {
    std::lock_guard command_lock(command_mutex_);
    worker_ = std::move(lifecycle_worker);
  }
  start_worker->store(true, std::memory_order_release);
  return Result<uint64_t>::Ok(id);
}

Result<uint64_t> PipelineController::SubmitRunAll() {
  return submit([this](uint64_t id) {
    for (StageId stage : PipelineStages()) {
      if (cancellationRequested()) {
        return Result<void>::Failure(
            Error::Cancelled("stopped at a stage boundary"));
      }
      auto result = runOneStage(id, stage);
      if (!result) return result;
    }
    return Result<void>::Ok();
  });
}

Result<uint64_t> PipelineController::SubmitStage(StageId stage) {
  return submit([this, stage](uint64_t id) {
    if (cancellationRequested()) {
      return Result<void>::Failure(Error::Cancelled("before stage start"));
    }
    return runOneStage(id, stage);
  });
}

Result<uint64_t> PipelineController::SubmitNode(NodeId node, AgentId agent) {
  auto valid = artifacts_.ValidateNode(node, agent);
  if (!valid) return Result<uint64_t>::Failure(valid.GetError());
  return submit([this, node, agent](uint64_t id) {
    if (cancellationRequested()) {
      return Result<void>::Failure(Error::Cancelled("before node start"));
    }
    const auto& descriptor = DescribeNode(node);
    const bool managed = runner_manages_artifacts_.load();
    const auto artifact_checkpoint = artifacts_.Snapshot();
    if (!managed) artifacts_.BeginNode(node, agent);
    emit({id, EventType::kNodeStarted, descriptor.stage, {}, 0, node, agent});
    auto result = runner_->RunNode(node, agent);
    if (result && cancellationRequested()) {
      result = Result<void>::Failure(
          Error::Cancelled("after node runner returned"));
    }
    if (!result) {
      const Error error = AddExecutionContext(
          result.GetError(), runner_, descriptor.stage, node, agent);
      result = Result<void>::Failure(error);
      if (result.GetError().code == Error::Code::kCancelled) {
        if (!managed) artifacts_.Restore(artifact_checkpoint);
        return result;
      }
      if (!managed) {
        artifacts_.FailNode(node, agent, result.GetError().Message());
      }
      emit(FailureEvent(id, EventType::kNodeFailed, descriptor.stage,
                        result.GetError(), node, agent));
      return result;
    }
    if (managed) {
      synchronizeCommittedSession(runner_);
    } else {
      artifacts_.CompleteNode(node, agent);
    }
    emit({id, EventType::kArtifactInvalidated, descriptor.stage,
          "downstream artifacts invalidated", 0, node, agent});
    emit({id, EventType::kArtifactCommitted, descriptor.stage, {},
          0, node, agent});
    const auto total = ProgressTotal(node, agents_, agent);
    emit({id, EventType::kProgressUpdated, descriptor.stage, {},
          0, node, agent, total, total});
    return Result<void>::Ok();
  });
}

Result<void> PipelineController::ApplyConfig(ConfigDomain domain,
                                             uint64_t revision) {
  std::shared_ptr<StageRunner> runner;
  {
    std::lock_guard command_lock(command_mutex_);
    std::lock_guard lock(mutex_);
    if (maintenance_in_progress_) {
      return Result<void>::Failure(
          Error::InvalidArgument("pipeline maintenance is in progress"));
    }
    if (job_ && (job_->state == JobState::kQueued ||
                 job_->state == JobState::kRunning ||
                 job_->state == JobState::kWaitingForAlignmentFeedback ||
                 job_->state == JobState::kCancelling)) {
      return Result<void>::Failure(
          Error::InvalidArgument("cannot apply config while a job is running"));
    }
    if (revision <= config_revision_) {
      return Result<void>::Failure(
          Error::InvalidArgument("config revision must increase"));
    }
    maintenance_in_progress_ = true;
    runner = runner_;
  }
  Result<void> reconfigured = Result<void>::Failure(
      Error::InvalidArgument("pipeline stage runner is null"));
  try {
    if (runner) reconfigured = runner->Reconfigure(domain, revision);
  } catch (const std::exception& error) {
    reconfigured = Result<void>::Failure(Error::InvalidArgument(error.what()));
  } catch (...) {
    reconfigured = Result<void>::Failure(
        Error::InvalidArgument("unknown runner reconfigure exception"));
  }
  {
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
    if (reconfigured) config_revision_ = revision;
  }
  if (!reconfigured) return reconfigured;
  if (runner_manages_artifacts_.load()) {
    synchronizeCommittedSession(runner);
  } else {
    artifacts_.ApplyConfig(domain, revision);
  }
  emit({0, EventType::kArtifactInvalidated, std::nullopt,
        "config applied"});
  return Result<void>::Ok();
}

Result<void> PipelineController::ReplaceRunner(
    std::shared_ptr<StageRunner> runner) {
  if (IsInEventCallback()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "cannot replace the pipeline runner from its event callback"));
  }
  if (!runner) return Result<void>::Failure(
      Error::InvalidArgument("pipeline stage runner is null"));
  std::thread finished_worker;
  {
    std::lock_guard command_lock(command_mutex_);
    std::lock_guard lock(mutex_);
    if (maintenance_in_progress_) {
      return Result<void>::Failure(
          Error::InvalidArgument("pipeline maintenance is in progress"));
    }
    if (job_ && (job_->state == JobState::kQueued ||
                 job_->state == JobState::kRunning ||
                 job_->state == JobState::kWaitingForAlignmentFeedback ||
                 job_->state == JobState::kCancelling)) {
      return Result<void>::Failure(
          Error::InvalidArgument("cannot create a session while a job is running"));
    }
    maintenance_in_progress_ = true;
    if (worker_.joinable()) finished_worker = std::move(worker_);
  }
  if (finished_worker.joinable()) finished_worker.join();
  std::vector<AgentId> agents;
  std::optional<CommittedSessionSnapshot> session;
  CancellationCapability cancellation_capability;
  try {
    agents = runner->AgentIds();
    cancellation_capability = runner->CancellationMetadata();
    runner->SetAlignmentFeedbackBroker(alignment_feedback_);
    session = runner->SessionSnapshot();
  } catch (const std::exception& error) {
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
    return Result<void>::Failure(Error::InvalidArgument(error.what()));
  } catch (...) {
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
    return Result<void>::Failure(
        Error::InvalidArgument("unknown runner replacement exception"));
  }
  {
    std::lock_guard lock(mutex_);
    runner_ = std::move(runner);
    agents_ = agents;
    cancellation_capability_ = std::move(cancellation_capability);
    job_.reset();
    terminal_event_completed_job_id_ = 0;
    cancellation_.reset();
    cancel_requested_ = false;
    ++config_revision_;
    maintenance_in_progress_ = false;
  }
  artifacts_.Reset(agents);
  if (session) {
    artifacts_.Restore(session->artifacts);
    {
      std::lock_guard lock(mutex_);
      agents_ = session->ordered_agents;
      config_revision_ = session->config_revision;
    }
    runner_manages_artifacts_ = true;
  } else {
    runner_manages_artifacts_ = false;
  }
  emit({0, EventType::kArtifactInvalidated, std::nullopt,
        "new pipeline session created"});
  return Result<void>::Ok();
}

std::vector<NodeDescriptor> PipelineController::NodeDescriptors() const {
  std::vector<NodeDescriptor> result;
  result.reserve(PipelineNodes().size());
  for (NodeId node : PipelineNodes()) {
    result.push_back(DescribeNode(node));
  }
  return result;
}

Result<uint64_t> PipelineController::SubmitOptimizeThrough(AgentId target_agent) {
  return submit([this, target_agent](uint64_t id) {
    const bool managed = runner_manages_artifacts_.load();
    const auto artifact_checkpoint = artifacts_.Snapshot();
    {
      std::lock_guard lock(mutex_);
      job_->active_stage = StageId::kAlignment;
    }
    emit({id, EventType::kStageStarted, StageId::kAlignment,
          "optimizer replay"});
    auto result = runner_->RunOptimizeThrough(target_agent);
    if (result && cancellationRequested()) {
      result = Result<void>::Failure(
          Error::Cancelled("after optimizer replay returned"));
    }
    if (!result) {
      const Error error = AddExecutionContext(
          result.GetError(), runner_, StageId::kAlignment,
          NodeId::kOptimize, target_agent);
      result = Result<void>::Failure(error);
      if (result.GetError().code == Error::Code::kCancelled) {
        if (!managed) artifacts_.Restore(artifact_checkpoint);
        return result;
      }
      if (!managed) {
        artifacts_.FailStage(StageId::kAlignment, result.GetError().Message());
      }
      emit(FailureEvent(id, EventType::kStageFailed, StageId::kAlignment,
                        result.GetError(), NodeId::kOptimize, target_agent));
      return result;
    }
    std::vector<AgentId> agents;
    {
      std::lock_guard lock(mutex_);
      agents = agents_;
    }
    if (managed) {
      synchronizeCommittedSession(runner_);
    } else {
      artifacts_.CompleteOptimizeThrough(target_agent, agents);
    }
    emit({id, EventType::kStageCompleted, StageId::kAlignment,
          "optimizer replay"});
    return Result<void>::Ok();
  });
}

Result<void> PipelineController::runOneStage(uint64_t job_id, StageId stage) {
  OPEN_LMM_ZONE_N("PipelineController.Stage");
  OPEN_LMM_PLOT("job.id", job_id);
  OPEN_LMM_PLOT("stage.id", static_cast<int>(stage));
  const auto artifact_checkpoint = artifacts_.Snapshot();
  const bool managed = runner_manages_artifacts_.load();
  if (!managed) artifacts_.BeginStage(stage);
  {
    std::lock_guard lock(mutex_);
    job_->active_stage = stage;
  }
  emit({job_id, EventType::kStageStarted, stage, {}});
  auto result = runner_->RunStage(stage);
  if (result && cancellationRequested()) {
    result = Result<void>::Failure(
        Error::Cancelled("after stage runner returned"));
  }
  if (!result) {
    const Error error = AddExecutionContext(result.GetError(), runner_, stage);
    result = Result<void>::Failure(error);
    if (result.GetError().code == Error::Code::kCancelled) {
      if (!managed) artifacts_.Restore(artifact_checkpoint);
      return result;
    }
    if (!managed) {
      artifacts_.FailStage(stage, result.GetError().Message());
    }
    emit(FailureEvent(job_id, EventType::kStageFailed, stage,
                      result.GetError()));
    return result;
  }
  if (managed) {
    synchronizeCommittedSession(runner_);
  } else {
    artifacts_.CompleteStage(stage);
  }
  emit({job_id, EventType::kStageCompleted, stage, {}});
  return Result<void>::Ok();
}

Result<void> PipelineController::Cancel(uint64_t job_id) {
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback;
  ExecutionEvent requested{
      job_id, EventType::kCancellationRequested, std::nullopt,
      "cancellation requested; waiting for the next safe point"};
  bool drain_callbacks = false;
  {
    std::lock_guard dispatch_lock(event_dispatch_mutex_);
    {
      std::lock_guard lock(mutex_);
      if (!job_ || job_->id != job_id) {
        return Result<void>::Failure(Error::InvalidArgument("unknown job id"));
      }
      if (job_->state != JobState::kQueued &&
          job_->state != JobState::kRunning &&
          job_->state != JobState::kWaitingForDependency &&
          job_->state != JobState::kWaitingForAlignmentFeedback) {
        return Result<void>::Failure(
            Error::InvalidArgument("job is not running"));
      }
      cancel_requested_ = true;
      job_->state = JobState::kCancelling;
      cancellation = cancellation_;
      alignment_feedback = alignment_feedback_;
      if (cancellation) {
        cancellation->Request();
        job_->cancellation = cancellation->Telemetry();
        requested.cancellation = job_->cancellation;
      }
      requested.sequence = next_event_sequence_++;
      recent_events_.push_back(requested);
      if (recent_events_.size() > 256) {
        recent_events_.erase(recent_events_.begin());
      }
    }
    pending_event_callbacks_.push_back(requested);
    if (!dispatching_events_) {
      dispatching_events_ = true;
      drain_callbacks = true;
    }
  }
  if (alignment_feedback) alignment_feedback->Cancel();
  if (drain_callbacks) drainEventCallbacks();
  return Result<void>::Ok();
}

bool PipelineController::cancellationRequested() const {
  if (!cancel_requested_.load(std::memory_order_acquire)) return false;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(mutex_);
    cancellation = cancellation_;
  }
  return !cancellation || cancellation->IsCancellationRequested();
}

void PipelineController::commitTerminal(uint64_t job_id,
                                        const Result<void>& result) {
  JobState state = JobState::kSucceeded;
  std::string message;
  if (!result) {
    message = result.GetError().Message();
    state = result.GetError().code == Error::Code::kCancelled
                ? JobState::kCancelled : JobState::kFailed;
  }
  ExecutionEvent terminal{
      job_id, state == JobState::kCancelled ? EventType::kJobCancelled
                                             : EventType::kJobCompleted,
      std::nullopt, message};
  if (!result) terminal.error = result.GetError();
  bool drain_callbacks = false;
  {
    std::lock_guard dispatch_lock(event_dispatch_mutex_);
    {
      std::lock_guard lock(mutex_);
      if (!job_ || job_->id != job_id ||
          terminal_event_completed_job_id_ == job_id) {
        return;
      }
      terminal.cancellation = job_->cancellation;
      if (cancellation_) {
        terminal.cancellation = cancellation_->Telemetry();
      }

      // The journal entry is committed before terminal state/watermark. A
      // waiter or callback can therefore never observe terminal state without
      // the corresponding event already being present in the snapshot.
      terminal.sequence = next_event_sequence_++;
      recent_events_.push_back(terminal);
      if (recent_events_.size() > 256) {
        recent_events_.erase(recent_events_.begin());
      }

      job_->state = state;
      job_->active_stage.reset();
      job_->message = message;
      if (terminal.cancellation) job_->cancellation = *terminal.cancellation;
      terminal_event_completed_job_id_ = job_id;
    }
    pending_event_callbacks_.push_back(terminal);
    if (!dispatching_events_) {
      dispatching_events_ = true;
      drain_callbacks = true;
    }
  }
  completed_.notify_all();
  if (drain_callbacks) drainEventCallbacks();
}

Result<void> PipelineController::Wait(uint64_t job_id) {
  std::unique_lock lock(mutex_);
  if (!job_ || job_->id != job_id) {
    return Result<void>::Failure(Error::InvalidArgument("unknown job id"));
  }
  completed_.wait(lock, [this, job_id] {
    return !job_ || job_->id != job_id ||
           ((job_->state == JobState::kSucceeded ||
             job_->state == JobState::kFailed ||
             job_->state == JobState::kCancelled) &&
            terminal_event_completed_job_id_ == job_id);
  });
  if (!job_ || job_->id != job_id) {
    return Result<void>::Failure(Error::InvalidArgument("job was replaced"));
  }
  if (job_->state == JobState::kSucceeded) return Result<void>::Ok();
  if (job_->state == JobState::kCancelled) {
    return Result<void>::Failure(Error::Cancelled(job_->message));
  }
  return Result<void>::Failure(Error::InvalidArgument(job_->message));
}

Result<void> PipelineController::WaitForEventCallbacks() {
  if (IsInEventCallback()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "cannot wait for event callbacks from an event callback"));
  }
  std::unique_lock lock(event_dispatch_mutex_);
  event_callbacks_idle_.wait(lock, [this] {
    return !dispatching_events_ && pending_event_callbacks_.empty();
  });
  return Result<void>::Ok();
}

bool PipelineController::IsInEventCallback() const noexcept {
  return active_event_controller == this;
}

PipelineSnapshot PipelineController::Snapshot() const {
  PipelineSnapshot snapshot;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(mutex_);
    snapshot.job = job_;
    cancellation = cancellation_;
    snapshot.config_revision = config_revision_;
    snapshot.agents = agents_;
    snapshot.recent_events = recent_events_;
  }
  if (snapshot.job && cancellation) {
    snapshot.job->cancellation = cancellation->Telemetry();
  }
  snapshot.artifacts = artifacts_.Snapshot();
  return snapshot;
}

Result<VisualizationSnapshot> PipelineController::GetVisualizationSnapshot(
    const AgentId& agent) const {
  std::shared_ptr<StageRunner> runner;
  {
    std::lock_guard lock(mutex_);
    runner = runner_;
  }
  if (!runner) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("stage runner is not available"));
  }
  auto result = runner->CreateVisualizationSnapshot(agent);
  if (!result) return result;
  return result;
}

std::optional<AlignmentFeedbackSnapshot>
PipelineController::GetAlignmentFeedbackSnapshot() const {
  {
    std::lock_guard lock(mutex_);
    if (!job_ || job_->state != JobState::kWaitingForAlignmentFeedback) {
      return std::nullopt;
    }
  }
  return alignment_feedback_ ? alignment_feedback_->Snapshot() : std::nullopt;
}

Result<void> PipelineController::RespondToAlignment(
    uint64_t job_id, AlignmentResponse response) {
  AlignmentDecision decision = response.decision;
  {
    std::lock_guard lock(mutex_);
    if (!job_ || job_->id != job_id) {
      return Result<void>::Failure(Error::InvalidArgument("unknown job id"));
    }
    if (job_->state != JobState::kWaitingForAlignmentFeedback) {
      return Result<void>::Failure(
          Error::InvalidArgument("job is not waiting for alignment feedback"));
    }
    job_->state = JobState::kRunning;
    job_->message.clear();
  }
  auto result = alignment_feedback_->Respond(std::move(response));
  if (!result) {
    std::lock_guard lock(mutex_);
    if (job_ && job_->id == job_id) {
      job_->state = JobState::kWaitingForAlignmentFeedback;
      job_->message = "waiting for map alignment feedback";
    }
    return result;
  }
  EventType type = EventType::kAlignmentProposalRejected;
  if (decision == AlignmentDecision::kAccept ||
      decision == AlignmentDecision::kManual) {
    type = EventType::kAlignmentProposalAccepted;
  } else if (decision == AlignmentDecision::kCancel) {
    type = EventType::kAlignmentFeedbackCancelled;
  }
  emit({job_id, type, StageId::kAlignment, "alignment feedback received"});
  return Result<void>::Ok();
}

void PipelineController::SetAlignmentFeedbackEnabled(bool enabled) {
  if (alignment_feedback_) alignment_feedback_->SetEnabled(enabled);
}

ExecutionEventSubscription PipelineController::SubscribeEvents(
    std::function<void(const ExecutionEvent&)> callback) {
  if (!callback) return {};
  uint64_t id;
  auto slot = std::make_shared<ExecutionEventSubscriberSlot>();
  slot->callback = std::move(callback);
  {
    std::lock_guard lock(event_subscribers_->mutex);
    id = event_subscribers_->next_id++;
    event_subscribers_->callbacks.emplace(id, slot);
  }
  std::weak_ptr<ExecutionEventSubscriberRegistry> weak = event_subscribers_;
  return ExecutionEventSubscription([weak, slot, id] {
    auto registry = weak.lock();
    if (registry) {
      std::lock_guard lock(registry->mutex);
      registry->callbacks.erase(id);
    }
    std::unique_lock slot_lock(slot->mutex);
    slot->active = false;
    if (active_event_subscriber != slot.get()) {
      slot->idle.wait(slot_lock,
                      [&slot] { return slot->callbacks_in_flight == 0; });
    }
  });
}

void PipelineController::emit(ExecutionEvent event) {
  bool drain_callbacks = false;
  {
    std::lock_guard dispatch_lock(event_dispatch_mutex_);
    {
      std::lock_guard lock(mutex_);
      event.sequence = next_event_sequence_++;
      recent_events_.push_back(event);
      if (recent_events_.size() > 256) {
        recent_events_.erase(recent_events_.begin());
      }
    }
    pending_event_callbacks_.push_back(std::move(event));
    if (!dispatching_events_) {
      dispatching_events_ = true;
      drain_callbacks = true;
    }
  }
  if (drain_callbacks) drainEventCallbacks();
}

void PipelineController::drainEventCallbacks() {
  for (;;) {
    ExecutionEvent event;
    {
      std::lock_guard lock(event_dispatch_mutex_);
      if (pending_event_callbacks_.empty()) {
        dispatching_events_ = false;
        event_callbacks_idle_.notify_all();
        return;
      }
      event = std::move(pending_event_callbacks_.front());
      pending_event_callbacks_.pop_front();
    }

    std::vector<std::shared_ptr<ExecutionEventSubscriberSlot>> subscribers;
    {
      std::lock_guard lock(event_subscribers_->mutex);
      subscribers.reserve(event_subscribers_->callbacks.size());
      for (const auto& [id, subscriber] : event_subscribers_->callbacks) {
        (void)id;
        subscribers.push_back(subscriber);
      }
    }
    for (const auto& subscriber : subscribers) {
      std::function<void(const ExecutionEvent&)> callback;
      {
        std::lock_guard lock(subscriber->mutex);
        if (!subscriber->active) continue;
        ++subscriber->callbacks_in_flight;
        callback = subscriber->callback;
      }
      const auto* previous = active_event_subscriber;
      const auto* previous_controller = active_event_controller;
      active_event_subscriber = subscriber.get();
      active_event_controller = this;
      try {
        callback(event);
      } catch (...) {
        // Event observers are isolated from the pipeline worker. Errors in an
        // observer must not change the committed execution result.
      }
      active_event_subscriber = previous;
      active_event_controller = previous_controller;
      {
        std::lock_guard lock(subscriber->mutex);
        --subscriber->callbacks_in_flight;
        if (subscriber->callbacks_in_flight == 0) {
          subscriber->idle.notify_all();
        }
      }
    }
  }
}

}  // namespace open_lmm
