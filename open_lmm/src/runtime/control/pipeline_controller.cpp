#include "pipeline_controller.hpp"

#include <algorithm>
#include <array>
#include <foundation/diagnostics/profiling.hpp>
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
    Error error, const std::shared_ptr<RuntimeQueryPort>& query_port,
    StageId stage,
    std::optional<NodeId> node = std::nullopt,
    std::optional<AgentId> agent = std::nullopt) {
  std::string node_name;
  if (node) node_name = std::string(DescribeNode(*node).name);
  error.WithExecution(StageName(stage), std::move(node_name), agent);
  try {
    if (query_port) {
      error.WithRuntimeRevision(query_port->Snapshot().revision);
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

bool SameRecovery(const std::shared_ptr<const Error>& lhs,
                  const std::shared_ptr<const Error>& rhs) {
  if (static_cast<bool>(lhs) != static_cast<bool>(rhs)) return false;
  if (!lhs) return true;
  return lhs->code == rhs->code && lhs->severity == rhs->severity &&
         lhs->message == rhs->message &&
         lhs->context.runtime_revision == rhs->context.runtime_revision &&
         lhs->context.stage == rhs->context.stage &&
         lhs->context.node == rhs->context.node &&
         lhs->context.agent == rhs->context.agent &&
         lhs->context.plugin == rhs->context.plugin &&
         lhs->context.config == rhs->context.config &&
         lhs->context.json_pointer == rhs->context.json_pointer &&
         lhs->context.expected == rhs->context.expected &&
         lhs->context.actual == rhs->context.actual &&
         lhs->context.schema_version == rhs->context.schema_version;
}

void AttachRecovery(ExecutionEvent& event,
                    const std::shared_ptr<const Error>& recovery) {
  if (!recovery) return;
  event.message = recovery->Message();
  event.error = *recovery;
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

PipelineController::PipelineController(std::shared_ptr<StageRuntimePort> port)
    : PipelineController(port, port) {}

PipelineController::PipelineController(
    std::shared_ptr<StageCommandPort> command_port,
    std::shared_ptr<RuntimeQueryPort> query_port)
    : command_port_(std::move(command_port)),
      query_port_(std::move(query_port)),
      event_subscribers_(std::make_shared<ExecutionEventSubscriberRegistry>()),
      alignment_feedback_(std::make_shared<AlignmentFeedbackBroker>()) {
  alignment_feedback_->SetNotification(
      [this](const AlignmentFeedbackSnapshot& snapshot) {
        uint64_t job_id = 0;
        EventType event_type = EventType::kAlignmentFeedbackRequested;
        std::string event_message = "map alignment feedback requested";
        {
          std::lock_guard lock(mutex_);
          if (!job_) return;
          job_id = job_->id;
          alignment_feedback_published_ = true;
          if (snapshot.review_state != AlignmentReviewState::kActive) {
            job_->message = snapshot.terminal_message;
            event_type = snapshot.review_state ==
                                 AlignmentReviewState::kCancelled
                             ? EventType::kAlignmentFeedbackCancelled
                             : EventType::kAlignmentProposalRejected;
            event_message = snapshot.terminal_message;
          } else if (snapshot.attempt_status.state ==
              AlignmentAttemptState::kRunning) {
            job_->state = JobState::kRunning;
            job_->message = snapshot.attempt_status.message;
          } else {
            job_->state = JobState::kWaitingForAlignmentFeedback;
            job_->message = "waiting for map alignment feedback";
          }
        }
        emit({job_id, event_type,
              StageId::kAlignment, std::move(event_message), 0,
              NodeId::kLoopDetect, snapshot.proposal.source_agent});
      });
  if (command_port_ && query_port_) {
    cancellation_capability_ = command_port_->CancellationMetadata();
    synchronizeCommittedRuntime(query_port_);
  }
}

void PipelineController::synchronizeCommittedRuntime(
    const std::shared_ptr<RuntimeQueryPort>& query_port) {
  if (!query_port) return;
  CommittedRuntimeSnapshot runtime;
  try {
    runtime = query_port->Snapshot();
  } catch (const std::exception&) {
    return;
  } catch (...) {
    return;
  }
  {
    std::lock_guard lock(mutex_);
    committed_runtime_ = runtime;
    agents_ = runtime.ordered_agents;
    config_revision_ = runtime.config_revision;
    committed_runtime_revision_ = runtime.revision;
  }
}

Result<void> PipelineController::acceptExecutionReceipt(
    const ExecutionReceipt& receipt, uint64_t sent_base_revision,
    const std::shared_ptr<RuntimeQueryPort>& query_port) {
  auto fail_protocol = [this](std::string message,
                              std::optional<uint64_t> observed_revision =
                                  std::nullopt) {
    Error error = Error::InvalidArgument(std::move(message));
    error.MarkFatalRuntime();
    if (observed_revision) error.WithRuntimeRevision(*observed_revision);
    {
      std::lock_guard lock(mutex_);
      protocol_failure_ = error;
      return Result<void>::Failure(*effectiveFatalRuntimeErrorLocked());
    }
  };
  if (!query_port) {
    return fail_protocol(
        "runtime query port is unavailable after successful execution");
  }
  CommittedRuntimeSnapshot runtime;
  try {
    runtime = query_port->Snapshot();
  } catch (const std::exception& error) {
    return fail_protocol(std::string(
        "runtime query snapshot failed after successful execution: ") +
                         error.what());
  } catch (...) {
    return fail_protocol(
        "runtime query snapshot failed after successful execution");
  }
  // Snapshot publication is the committed authority.  Publish it locally even
  // when the receipt is malformed so a post-commit protocol failure cannot
  // leave the controller displaying the pre-command artifacts/config.
  {
    std::lock_guard lock(mutex_);
    committed_runtime_ = runtime;
    agents_ = runtime.ordered_agents;
    config_revision_ = runtime.config_revision;
    committed_runtime_revision_ = runtime.revision;
  }
  if (!SameRecovery(receipt.recovery_required,
                    runtime.recovery_required)) {
    return fail_protocol(
        "execution receipt recovery health does not match query authority",
        runtime.revision);
  }
  if (receipt.base_revision != sent_base_revision) {
    return fail_protocol(
        "execution receipt base revision does not match the command context",
        runtime.revision);
  }
  if (receipt.committed_revision <= receipt.base_revision) {
    return fail_protocol(
        "successful execution did not advance the committed revision",
        runtime.revision);
  }
  if (runtime.revision != receipt.committed_revision) {
    return fail_protocol(
        "query snapshot revision does not match the execution receipt",
        runtime.revision);
  }
  return Result<void>::Ok();
}

Error PipelineController::reconcileExecutionFailure(
    Error error, uint64_t sent_base_revision,
    const std::shared_ptr<RuntimeQueryPort>& query_port) {
  auto fail_protocol = [this](std::string message,
                              std::optional<uint64_t> observed_revision =
                                  std::nullopt) {
    Error protocol_error = Error::InvalidArgument(std::move(message));
    protocol_error.MarkFatalRuntime();
    if (observed_revision) {
      protocol_error.WithRuntimeRevision(*observed_revision);
    }
    {
      std::lock_guard lock(mutex_);
      protocol_failure_ = protocol_error;
      return *effectiveFatalRuntimeErrorLocked();
    }
  };

  if (!query_port) {
    return fail_protocol(
        "runtime query port is unavailable after failed execution; "
        "commit authority is unknown");
  }

  CommittedRuntimeSnapshot runtime;
  try {
    runtime = query_port->Snapshot();
  } catch (const std::exception& snapshot_error) {
    return fail_protocol(
        std::string("runtime query snapshot failed after failed execution; ") +
        "commit authority is unknown: " + snapshot_error.what());
  } catch (...) {
    return fail_protocol(
        "runtime query snapshot failed after failed execution; "
        "commit authority is unknown");
  }

  if (runtime.revision == sent_base_revision) return error;
  if (runtime.revision < sent_base_revision) {
    return fail_protocol(
        "runtime query revision regressed after failed execution; "
        "commit authority is unknown",
        runtime.revision);
  }

  {
    std::lock_guard lock(mutex_);
    committed_runtime_ = runtime;
    agents_ = runtime.ordered_agents;
    config_revision_ = runtime.config_revision;
    committed_runtime_revision_ = runtime.revision;
  }
  return fail_protocol(
      "execution reported failure after authoritative runtime advanced: " +
          error.Message(),
      runtime.revision);
}

Result<ExecutionReceipt> PipelineController::executeCommand(
    const std::shared_ptr<StageCommandPort>& command_port,
    const ExecutionCommand& command, const ExecutionContext& context) {
  if (!command_port) {
    return Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument("pipeline command port is unavailable"));
  }
  try {
    return command_port->Execute(command, context);
  } catch (const std::exception& error) {
    return Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument(error.what()));
  } catch (...) {
    return Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument("unknown command-port execution exception"));
  }
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
  uint64_t id;
  std::shared_ptr<CancellationToken> cancellation;
  ExecutionContext execution_context;
  {
    std::lock_guard command_lock(command_mutex_);
    std::lock_guard state_lock(mutex_);
    if (!command_port_ || !query_port_) {
      return Result<uint64_t>::Failure(
          Error::InvalidArgument("pipeline execution ports are unavailable"));
    }
    if (const auto fatal = effectiveFatalRuntimeErrorLocked()) {
      return Result<uint64_t>::Failure(*fatal);
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
    id = next_job_id_++;
    cancellation =
        std::make_shared<CancellationToken>(cancellation_capability_);
    job_ = JobSnapshot{id, JobState::kQueued, std::nullopt, {},
                       CancellationTelemetry{cancellation_capability_}};
    terminal_event_completed_job_id_ = 0;
    alignment_feedback_published_ = false;
    cancel_requested_ = false;
    cancellation_ = cancellation;
    execution_context = {
        cancellation, alignment_feedback_, committed_runtime_revision_};
  }

  // Joining is deliberately outside both controller mutexes. The queued job
  // reserves the ports against replacement while the external call is active.
  if (finished_worker.joinable()) finished_worker.join();
  emit({id, EventType::kJobQueued, std::nullopt, {}});
  auto start_worker = std::make_shared<std::atomic<bool>>(false);
  auto work_result = std::make_shared<std::optional<Result<void>>>();
  std::thread execution_worker(
      [this, id, work = std::move(work), start_worker, cancellation,
       execution_context = std::move(execution_context), work_result]() mutable {
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
      result = work(id, execution_context);
    } catch (const std::exception& e) {
      result = Result<void>::Failure(Error::InvalidArgument(e.what()));
    } catch (...) {
      result = Result<void>::Failure(
          Error::InvalidArgument("unknown pipeline exception"));
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
  return submit([this](uint64_t id, const ExecutionContext& context) {
    for (StageId stage : PipelineStages()) {
      if (cancellationRequested()) {
        return Result<void>::Failure(
            Error::Cancelled("stopped at a stage boundary"));
      }
      auto result = runOneStage(id, stage, context);
      if (!result) return result;
      if (const auto fatal = FatalRuntimeError()) {
        return Result<void>::Failure(*fatal);
      }
    }
    return Result<void>::Ok();
  });
}

Result<uint64_t> PipelineController::SubmitStage(StageId stage) {
  return submit([this, stage](uint64_t id,
                             const ExecutionContext& context) {
    if (cancellationRequested()) {
      return Result<void>::Failure(Error::Cancelled("before stage start"));
    }
    return runOneStage(id, stage, context);
  });
}

Result<uint64_t> PipelineController::SubmitNode(
    NodeId node, std::optional<AgentId> agent) {
  return submit([this, node, agent](uint64_t id,
                                   const ExecutionContext& context) {
    if (cancellationRequested()) {
      return Result<void>::Failure(Error::Cancelled("before node start"));
    }
    const auto& descriptor = DescribeNode(node);
    emit({id, EventType::kNodeStarted, descriptor.stage, {}, 0, node, agent});
    ExecutionContext command_context = context;
    {
      std::lock_guard lock(mutex_);
      command_context.base_revision = committed_runtime_.revision;
    }
    command_context = withProgress(command_context, id, descriptor.stage,
                                   node, agent);
    auto result = executeCommand(command_port_,
                                 ExecutionCommand::Node(node, agent),
                                 command_context);
    if (!result) {
      const Error error = AddExecutionContext(reconcileExecutionFailure(
          result.GetError(), command_context.base_revision, query_port_),
          query_port_, descriptor.stage, node, agent);
      emit(FailureEvent(id, EventType::kNodeFailed, descriptor.stage,
                        error, node, agent));
      return Result<void>::Failure(error);
    }
    auto accepted = acceptExecutionReceipt(
        result.Value(), command_context.base_revision, query_port_);
    if (!accepted) {
      const Error error = AddExecutionContext(
          accepted.GetError(), query_port_, descriptor.stage, node, agent);
      emit(FailureEvent(id, EventType::kNodeFailed, descriptor.stage,
                        error, node, agent));
      return Result<void>::Failure(error);
    }
    ExecutionEvent invalidated{
        id, EventType::kArtifactInvalidated, descriptor.stage,
        "downstream artifacts invalidated", 0, node, agent};
    invalidated.affected_agents = result.Value().affected_agents;
    emit(std::move(invalidated));
    ExecutionEvent committed{
        id, EventType::kArtifactCommitted, descriptor.stage, {},
        0, node, agent};
    committed.affected_agents = result.Value().affected_agents;
    AttachRecovery(committed, result.Value().recovery_required);
    emit(std::move(committed));
    const auto total = ProgressTotal(node, agents_, agent);
    emit({id, EventType::kProgressUpdated, descriptor.stage, {},
          0, node, agent, total, total});
    return Result<void>::Ok();
  });
}

Result<void> PipelineController::ApplyConfig(ConfigDomain domain,
                                             uint64_t revision) {
  std::shared_ptr<StageCommandPort> command_port;
  std::shared_ptr<RuntimeQueryPort> query_port;
  uint64_t base_revision = 0;
  {
    std::lock_guard command_lock(command_mutex_);
    std::lock_guard lock(mutex_);
    if (maintenance_in_progress_) {
      return Result<void>::Failure(
          Error::InvalidArgument("pipeline maintenance is in progress"));
    }
    if (const auto fatal = effectiveFatalRuntimeErrorLocked()) {
      return Result<void>::Failure(*fatal);
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
    command_port = command_port_;
    query_port = query_port_;
    base_revision = committed_runtime_revision_;
  }
  Result<ExecutionReceipt> reconfigured =
      Result<ExecutionReceipt>::Failure(
          Error::InvalidArgument("pipeline execution ports are unavailable"));
  try {
    if (command_port && query_port) {
      reconfigured = command_port->Execute(
          ExecutionCommand::Reconfigure(domain, revision),
          {std::make_shared<CancellationToken>(), alignment_feedback_,
           base_revision});
    }
  } catch (const std::exception& error) {
    reconfigured = Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument(error.what()));
  } catch (...) {
    reconfigured = Result<ExecutionReceipt>::Failure(
        Error::InvalidArgument("unknown command-port reconfigure exception"));
  }
  if (!reconfigured) {
    const Error error = reconcileExecutionFailure(
        reconfigured.GetError(), base_revision, query_port);
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
    return Result<void>::Failure(error);
  }
  auto accepted = acceptExecutionReceipt(
      reconfigured.Value(), base_revision, query_port);
  {
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
  }
  if (!accepted) return accepted;
  ExecutionEvent configured{0, EventType::kArtifactInvalidated, std::nullopt,
                            "config applied"};
  AttachRecovery(configured, reconfigured.Value().recovery_required);
  emit(std::move(configured));
  return Result<void>::Ok();
}

Result<ConfigApplyReceipt> PipelineController::ApplyConfig(
    const ConfigCandidate& candidate, const ExpectedRevision& expected) {
  std::shared_ptr<StageCommandPort> command_port;
  std::shared_ptr<RuntimeQueryPort> query_port;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback;
  {
    std::lock_guard command_lock(command_mutex_);
    std::lock_guard lock(mutex_);
    if (maintenance_in_progress_) {
      return Result<ConfigApplyReceipt>::Failure(
          Error::InvalidArgument("pipeline maintenance is in progress"));
    }
    if (const auto fatal = effectiveFatalRuntimeErrorLocked()) {
      return Result<ConfigApplyReceipt>::Failure(*fatal);
    }
    if (job_ && (job_->state == JobState::kQueued ||
                 job_->state == JobState::kRunning ||
                 job_->state == JobState::kWaitingForAlignmentFeedback ||
                 job_->state == JobState::kCancelling)) {
      return Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument(
          "cannot apply config while a job is running"));
    }
    if (expected.runtime_revision != committed_runtime_.revision ||
        expected.config_revision != committed_runtime_.config_revision) {
      return Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument(
          "config expected revision does not match committed runtime"));
    }
    maintenance_in_progress_ = true;
    command_port = command_port_;
    query_port = query_port_;
    alignment_feedback = alignment_feedback_;
  }

  Result<ConfigCommandReceipt> applied = Result<ConfigCommandReceipt>::Failure(
      Error::InvalidArgument("pipeline execution ports are unavailable"));
  try {
    if (command_port && query_port) {
      applied = command_port->ApplyConfig(
          candidate, expected,
          {std::make_shared<CancellationToken>(), alignment_feedback,
           expected.runtime_revision});
    }
  } catch (const std::exception& error) {
    applied = Result<ConfigCommandReceipt>::Failure(
        Error::InvalidArgument(error.what()));
  } catch (...) {
    applied = Result<ConfigCommandReceipt>::Failure(Error::InvalidArgument(
        "unknown command-port config transaction exception"));
  }
  if (!applied) {
    const Error error = reconcileExecutionFailure(
        applied.GetError(), expected.runtime_revision, query_port);
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
    return Result<ConfigApplyReceipt>::Failure(error);
  }

  const auto& command_receipt = applied.Value();
  const auto& receipt = command_receipt.receipt;
  auto accepted = acceptExecutionReceipt(
      {receipt.base_runtime_revision, receipt.runtime_revision,
       receipt.affected_agents, {}, command_receipt.recovery_required},
      expected.runtime_revision, query_port);
  Error config_protocol_error = Error::InvalidArgument("");
  bool config_protocol_failed = false;
  {
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
    if (accepted &&
        (receipt.previous_config_revision != expected.config_revision ||
         receipt.config_revision <= receipt.previous_config_revision ||
         committed_runtime_.config_revision != receipt.config_revision)) {
      config_protocol_error = Error::InvalidArgument(
          "config transaction receipt does not match committed runtime");
      config_protocol_error.MarkFatalRuntime().WithRuntimeRevision(
          committed_runtime_.revision);
      protocol_failure_ = config_protocol_error;
      config_protocol_error = *effectiveFatalRuntimeErrorLocked();
      config_protocol_failed = true;
    }
  }
  if (!accepted) {
    return Result<ConfigApplyReceipt>::Failure(accepted.GetError());
  }
  if (config_protocol_failed) {
    return Result<ConfigApplyReceipt>::Failure(
        std::move(config_protocol_error));
  }
  ExecutionEvent invalidated{0, EventType::kArtifactInvalidated,
                             std::nullopt, "config applied"};
  invalidated.affected_agents = receipt.affected_agents;
  AttachRecovery(invalidated, command_receipt.recovery_required);
  emit(std::move(invalidated));
  return Result<ConfigApplyReceipt>::Ok(receipt);
}

Result<void> PipelineController::ReplacePorts(
    std::shared_ptr<StageCommandPort> command_port,
    std::shared_ptr<RuntimeQueryPort> query_port) {
  if (IsInEventCallback()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "cannot replace pipeline ports from an event callback"));
  }
  if (!command_port || !query_port) {
    return Result<void>::Failure(
        Error::InvalidArgument("pipeline execution ports are unavailable"));
  }
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
          Error::InvalidArgument("cannot replace a runtime while a job is running"));
    }
    maintenance_in_progress_ = true;
    if (worker_.joinable()) finished_worker = std::move(worker_);
  }
  if (finished_worker.joinable()) finished_worker.join();
  CommittedRuntimeSnapshot runtime;
  CancellationCapability cancellation_capability;
  try {
    cancellation_capability = command_port->CancellationMetadata();
    runtime = query_port->Snapshot();
  } catch (const std::exception& error) {
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
    return Result<void>::Failure(Error::InvalidArgument(error.what()));
  } catch (...) {
    std::lock_guard lock(mutex_);
    maintenance_in_progress_ = false;
    return Result<void>::Failure(
        Error::InvalidArgument("unknown port replacement exception"));
  }
  {
    std::lock_guard lock(mutex_);
    command_port_ = std::move(command_port);
    query_port_ = std::move(query_port);
    agents_ = runtime.ordered_agents;
    cancellation_capability_ = std::move(cancellation_capability);
    job_.reset();
    alignment_feedback_published_ = false;
    terminal_event_completed_job_id_ = 0;
    cancellation_.reset();
    cancel_requested_ = false;
    config_revision_ = runtime.config_revision;
    committed_runtime_revision_ = runtime.revision;
    committed_runtime_ = runtime;
    protocol_failure_.reset();
    maintenance_in_progress_ = false;
  }
  emit({0, EventType::kArtifactInvalidated, std::nullopt,
        "new pipeline runtime initialized"});
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
  return submit([this, target_agent](uint64_t id,
                                     const ExecutionContext& context) {
    {
      std::lock_guard lock(mutex_);
      job_->active_stage = StageId::kAlignment;
    }
    emit({id, EventType::kStageStarted, StageId::kAlignment,
          "optimizer replay"});
    ExecutionContext command_context = context;
    {
      std::lock_guard lock(mutex_);
      command_context.base_revision = committed_runtime_.revision;
    }
    command_context = withProgress(command_context, id, StageId::kAlignment,
                                   NodeId::kOptimize, target_agent);
    auto result = executeCommand(command_port_,
                                 ExecutionCommand::OptimizeThrough(target_agent),
                                 command_context);
    if (!result) {
      const Error error = AddExecutionContext(reconcileExecutionFailure(
          result.GetError(), command_context.base_revision, query_port_),
          query_port_, StageId::kAlignment, NodeId::kOptimize, target_agent);
      emit(FailureEvent(id, EventType::kStageFailed, StageId::kAlignment,
                        error, NodeId::kOptimize, target_agent));
      return Result<void>::Failure(error);
    }
    auto accepted = acceptExecutionReceipt(
        result.Value(), command_context.base_revision, query_port_);
    if (!accepted) {
      const Error error = AddExecutionContext(
          accepted.GetError(), query_port_, StageId::kAlignment,
          NodeId::kOptimize, target_agent);
      emit(FailureEvent(id, EventType::kStageFailed, StageId::kAlignment,
                        error, NodeId::kOptimize, target_agent));
      return Result<void>::Failure(error);
    }
    ExecutionEvent completed{id, EventType::kStageCompleted,
                             StageId::kAlignment, "optimizer replay"};
    completed.affected_agents = result.Value().affected_agents;
    AttachRecovery(completed, result.Value().recovery_required);
    emit(std::move(completed));
    return Result<void>::Ok();
  });
}

Result<void> PipelineController::runOneStage(
    uint64_t job_id, StageId stage, const ExecutionContext& context) {
  OPEN_LMM_ZONE_N("PipelineController.Stage");
  OPEN_LMM_PLOT("job.id", job_id);
  OPEN_LMM_PLOT("stage.id", static_cast<int>(stage));
  {
    std::lock_guard lock(mutex_);
    job_->active_stage = stage;
  }
  emit({job_id, EventType::kStageStarted, stage, {}});
  ExecutionContext command_context = context;
  {
    std::lock_guard lock(mutex_);
    command_context.base_revision = committed_runtime_.revision;
  }
  command_context = withProgress(command_context, job_id, stage, std::nullopt);
  auto result = executeCommand(command_port_, ExecutionCommand::Stage(stage),
                               command_context);
  if (!result) {
    const Error error = AddExecutionContext(reconcileExecutionFailure(
        result.GetError(), command_context.base_revision, query_port_),
        query_port_, stage);
    emit(FailureEvent(job_id, EventType::kStageFailed, stage,
                      error));
    return Result<void>::Failure(error);
  }
  auto accepted = acceptExecutionReceipt(
      result.Value(), command_context.base_revision, query_port_);
  if (!accepted) {
    const Error error = AddExecutionContext(
        accepted.GetError(), query_port_, stage);
    emit(FailureEvent(job_id, EventType::kStageFailed, stage, error));
    return Result<void>::Failure(error);
  }
  for (const AgentId& excluded : result.Value().excluded_agents) {
    ExecutionEvent event{
        job_id, EventType::kAlignmentAgentExcluded, StageId::kAlignment,
        "agent excluded from this Alignment result", 0,
        NodeId::kLoopDetect, excluded};
    event.affected_agents = {excluded};
    emit(std::move(event));
  }
  ExecutionEvent completed{job_id, EventType::kStageCompleted, stage, {}};
  completed.affected_agents = result.Value().affected_agents;
  AttachRecovery(completed, result.Value().recovery_required);
  emit(std::move(completed));
  return Result<void>::Ok();
}

ExecutionContext PipelineController::withProgress(
    const ExecutionContext& context, uint64_t job_id, StageId stage,
    std::optional<NodeId> node, std::optional<AgentId> fallback_agent) {
  ExecutionContext result = context;
  result.progress =
      [this, job_id, stage, node,
       fallback_agent = std::move(fallback_agent)](
          const AlgorithmProgress& progress) {
        ExecutionEvent event;
        event.job_id = job_id;
        event.type = EventType::kProgressUpdated;
        event.stage = stage;
        event.node = node;
        event.agent = progress.agent.IsValid()
                          ? std::optional<AgentId>(progress.agent)
                          : fallback_agent;
        event.message = std::string(DescribeAlgorithmProgressPhase(
            progress.phase));
        event.progress_current = progress.current;
        event.progress_total = progress.total.value_or(0);
        event.algorithm_progress = progress;
        emit(std::move(event));
      };
  return result;
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
    snapshot.runtime_revision = committed_runtime_.revision;
    snapshot.config_revision = committed_runtime_.config_revision;
    snapshot.agents = committed_runtime_.ordered_agents;
    snapshot.artifacts = committed_runtime_.artifacts;
    snapshot.recent_events = recent_events_;
  }
  if (snapshot.job && cancellation) {
    snapshot.job->cancellation = cancellation->Telemetry();
  }
  return snapshot;
}

std::optional<Error> PipelineController::FatalRuntimeError() const {
  std::lock_guard lock(mutex_);
  return effectiveFatalRuntimeErrorLocked();
}

std::optional<Error> PipelineController::effectiveFatalRuntimeErrorLocked()
    const {
  if (!protocol_failure_) {
    return committed_runtime_.recovery_required
               ? std::optional<Error>(*committed_runtime_.recovery_required)
               : std::nullopt;
  }
  if (!committed_runtime_.recovery_required) return protocol_failure_;

  // Recovery is the actionable committed-state condition: its code, manifest
  // path, and structured context must remain observable even when the same
  // command also violates the controller/port protocol. Keep both canonical
  // causes separately latched and synthesize their public projection on read.
  Error combined = *committed_runtime_.recovery_required;
  combined.MarkFatalRuntime();
  combined.message += "; additional runtime protocol failure: ";
  combined.message += protocol_failure_->Message();
  return combined;
}

void PipelineController::RecordRecoveryRequired(
    std::shared_ptr<const Error> recovery_required) noexcept {
  std::lock_guard lock(mutex_);
  committed_runtime_.recovery_required = std::move(recovery_required);
}

Result<VisualizationSnapshot> PipelineController::GetVisualizationSnapshot(
    const AgentId& agent) const {
  return GetVisualizationSnapshot(VisualizationQuery{agent});
}

Result<VisualizationSnapshot> PipelineController::GetVisualizationSnapshot(
    const VisualizationQuery& query) const {
  std::shared_ptr<RuntimeQueryPort> query_port;
  {
    std::lock_guard lock(mutex_);
    query_port = query_port_;
  }
  if (!query_port) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("runtime query port is not available"));
  }
  auto result = query_port->Visualization(query);
  if (!result) return result;
  return result;
}

std::optional<AlignmentFeedbackSnapshot>
PipelineController::GetAlignmentFeedbackSnapshot() const {
  JobState job_state = JobState::kQueued;
  {
    std::lock_guard lock(mutex_);
    if (!job_ || !alignment_feedback_published_) {
      return std::nullopt;
    }
    job_state = job_->state;
  }
  auto snapshot =
      alignment_feedback_ ? alignment_feedback_->Snapshot() : std::nullopt;
  if (!snapshot) return std::nullopt;
  if (snapshot->review_state != AlignmentReviewState::kActive) {
    return snapshot;
  }
  const bool attempt_is_running =
      snapshot->attempt_status.state == AlignmentAttemptState::kRunning;
  const bool controller_state_matches =
      attempt_is_running ? job_state == JobState::kRunning
                         : job_state == JobState::kWaitingForAlignmentFeedback;
  return controller_state_matches ? snapshot : std::nullopt;
}

Result<void> PipelineController::RespondToAlignment(
    uint64_t job_id, AlignmentResponse response) {
  AlignmentDecision decision = response.decision;
  JobState previous_state = JobState::kQueued;
  std::string previous_message;
  {
    std::lock_guard lock(mutex_);
    if (!job_ || job_->id != job_id) {
      return Result<void>::Failure(Error::InvalidArgument("unknown job id"));
    }
    if (job_->state != JobState::kWaitingForAlignmentFeedback &&
        !(job_->state == JobState::kRunning &&
          decision == AlignmentDecision::kCancel)) {
      return Result<void>::Failure(
          Error::InvalidArgument("job is not waiting for alignment feedback"));
    }
    previous_state = job_->state;
    previous_message = job_->message;
    job_->state = JobState::kRunning;
    job_->message.clear();
  }
  auto result = alignment_feedback_->Respond(std::move(response));
  if (!result) {
    std::lock_guard lock(mutex_);
    if (job_ && job_->id == job_id) {
      job_->state = previous_state;
      job_->message = std::move(previous_message);
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
