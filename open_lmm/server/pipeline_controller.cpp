#include "pipeline_controller.hpp"

#include <algorithm>
#include <array>
#include <open_lmm/common/profiling.hpp>
#include <exception>
#include <map>

namespace open_lmm {

struct ExecutionEventSubscriberSlot {
  std::recursive_mutex mutex;
  bool active = true;
  std::function<void(const ExecutionEvent&)> callback;
};

struct ExecutionEventSubscriberRegistry {
  std::mutex mutex;
  uint64_t next_id = 1;
  std::map<uint64_t, std::shared_ptr<ExecutionEventSubscriberSlot>> callbacks;
};

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
    runner_->SetAlignmentFeedbackBroker(alignment_feedback_);
    artifacts_.RegisterAgents(runner_->AgentIds());
  }
}

PipelineController::~PipelineController() {
  cancel_requested_ = true;
  if (cancellation_) cancellation_->Request();
  if (alignment_feedback_) alignment_feedback_->Cancel();
  if (worker_.joinable()) worker_.join();
}

Result<uint64_t> PipelineController::submit(Work work) {
  if (!runner_) {
    return Result<uint64_t>::Failure(
        Error::InvalidArgument("pipeline stage runner is null"));
  }
  {
    std::lock_guard lock(mutex_);
    if (job_ && (job_->state == JobState::kQueued ||
                 job_->state == JobState::kRunning ||
                 job_->state == JobState::kWaitingForAlignmentFeedback ||
                 job_->state == JobState::kCancelling)) {
      return Result<uint64_t>::Failure(
          Error::InvalidArgument("another pipeline job is already running"));
    }
  }
  if (worker_.joinable()) worker_.join();

  uint64_t id;
  {
    std::lock_guard lock(mutex_);
    id = next_job_id_++;
    job_ = JobSnapshot{id, JobState::kQueued, std::nullopt, {}};
    cancel_requested_ = false;
  }
  cancellation_ = std::make_shared<CancellationToken>();
  runner_->SetCancellationToken(cancellation_);
  runner_->SetAlignmentFeedbackBroker(alignment_feedback_);
  emit({id, EventType::kJobQueued, std::nullopt, {}});
  worker_ = std::thread([this, id, work = std::move(work)]() mutable {
    OPEN_LMM_THREAD_NAME("open_lmm.pipeline");
    OPEN_LMM_ZONE_N("PipelineController.Job");
    OPEN_LMM_PLOT("job.id", id);
    {
      std::lock_guard lock(mutex_);
      job_->state = JobState::kRunning;
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
    finish(id, result);
  });
  return Result<uint64_t>::Ok(id);
}

Result<uint64_t> PipelineController::SubmitRunAll() {
  return submit([this](uint64_t id) {
    for (StageId stage : {StageId::kDataLoad, StageId::kAlignment,
                          StageId::kMapUpdate, StageId::kSave}) {
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

Result<uint64_t> PipelineController::SubmitNode(NodeId node, char agent) {
  auto valid = artifacts_.ValidateNode(node, agent);
  if (!valid) return Result<uint64_t>::Failure(valid.GetError());
  return submit([this, node, agent](uint64_t id) {
    if (cancellationRequested()) {
      return Result<void>::Failure(Error::Cancelled("before node start"));
    }
    const auto& descriptor = DescribeNode(node);
    const auto artifact_checkpoint = artifacts_.Snapshot();
    artifacts_.BeginNode(node, agent);
    emit({id, EventType::kArtifactInvalidated, descriptor.stage,
          "downstream artifacts invalidated", 0, node, agent});
    emit({id, EventType::kNodeStarted, descriptor.stage, {}, 0, node, agent});
    auto result = runner_->RunNode(node, agent);
    if (!result) {
      if (result.GetError().code == Error::Code::kCancelled) {
        artifacts_.Restore(artifact_checkpoint);
        return result;
      }
      artifacts_.FailNode(node, agent, result.GetError().Message());
      emit({id, EventType::kNodeFailed, descriptor.stage,
            result.GetError().Message(), 0, node, agent});
      return result;
    }
    artifacts_.CompleteNode(node, agent);
    emit({id, EventType::kArtifactCommitted, descriptor.stage, {},
          0, node, agent});
    emit({id, EventType::kProgressUpdated, descriptor.stage, {},
          0, node, agent, 1, 1});
    return Result<void>::Ok();
  });
}

Result<void> PipelineController::ApplyConfig(ConfigDomain domain,
                                             uint64_t revision) {
  {
    std::lock_guard lock(mutex_);
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
    config_revision_ = revision;
  }
  artifacts_.ApplyConfig(domain, revision);
  emit({0, EventType::kArtifactInvalidated, std::nullopt,
        "config applied"});
  return Result<void>::Ok();
}

Result<void> PipelineController::ReplaceRunner(
    std::shared_ptr<StageRunner> runner) {
  if (!runner) return Result<void>::Failure(
      Error::InvalidArgument("pipeline stage runner is null"));
  {
    std::lock_guard lock(mutex_);
    if (job_ && (job_->state == JobState::kQueued ||
                 job_->state == JobState::kRunning ||
                 job_->state == JobState::kWaitingForAlignmentFeedback ||
                 job_->state == JobState::kCancelling)) {
      return Result<void>::Failure(
          Error::InvalidArgument("cannot create a session while a job is running"));
    }
  }
  if (worker_.joinable()) worker_.join();
  const auto agents = runner->AgentIds();
  {
    std::lock_guard lock(mutex_);
    runner_ = std::move(runner);
    runner_->SetAlignmentFeedbackBroker(alignment_feedback_);
    job_.reset();
    cancellation_.reset();
    cancel_requested_ = false;
    ++config_revision_;
  }
  artifacts_.Reset(agents);
  emit({0, EventType::kArtifactInvalidated, std::nullopt,
        "new pipeline session created"});
  return Result<void>::Ok();
}

std::vector<NodeDescriptor> PipelineController::NodeDescriptors() const {
  std::vector<NodeDescriptor> result;
  for (NodeId node : {NodeId::kDataLoad, NodeId::kLoopDetect,
                      NodeId::kOptimize, NodeId::kMapUpdate,
                      NodeId::kPoseSave}) {
    result.push_back(DescribeNode(node));
  }
  return result;
}

Result<uint64_t> PipelineController::SubmitOptimizeThrough(char target_agent) {
  return submit([this, target_agent](uint64_t id) {
    const auto artifact_checkpoint = artifacts_.Snapshot();
    {
      std::lock_guard lock(mutex_);
      job_->active_stage = StageId::kAlignment;
    }
    emit({id, EventType::kStageStarted, StageId::kAlignment,
          "optimizer replay"});
    auto result = runner_->RunOptimizeThrough(target_agent);
    if (!result) {
      if (result.GetError().code == Error::Code::kCancelled) {
        artifacts_.Restore(artifact_checkpoint);
        return result;
      }
      artifacts_.FailStage(StageId::kAlignment, result.GetError().Message());
      emit({id, EventType::kStageFailed, StageId::kAlignment,
            result.GetError().Message()});
      return result;
    }
    artifacts_.CompleteOptimizeThrough(target_agent, runner_->AgentIds());
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
  artifacts_.BeginStage(stage);
  {
    std::lock_guard lock(mutex_);
    job_->active_stage = stage;
  }
  emit({job_id, EventType::kStageStarted, stage, {}});
  auto result = runner_->RunStage(stage);
  if (!result) {
    if (result.GetError().code == Error::Code::kCancelled) {
      artifacts_.Restore(artifact_checkpoint);
      return result;
    }
    artifacts_.FailStage(stage, result.GetError().Message());
    emit({job_id, EventType::kStageFailed, stage, result.GetError().Message()});
    return result;
  }
  artifacts_.CompleteStage(stage);
  emit({job_id, EventType::kStageCompleted, stage, {}});
  return Result<void>::Ok();
}

Result<void> PipelineController::Cancel(uint64_t job_id) {
  {
    std::lock_guard lock(mutex_);
    if (!job_ || job_->id != job_id) {
      return Result<void>::Failure(Error::InvalidArgument("unknown job id"));
    }
    if (job_->state != JobState::kQueued &&
        job_->state != JobState::kRunning &&
        job_->state != JobState::kWaitingForDependency &&
        job_->state != JobState::kWaitingForAlignmentFeedback) {
      return Result<void>::Failure(Error::InvalidArgument("job is not running"));
    }
    cancel_requested_ = true;
    job_->state = JobState::kCancelling;
    if (cancellation_) cancellation_->Request();
    if (alignment_feedback_) alignment_feedback_->Cancel();
  }
  emit({job_id, EventType::kCancellationRequested, std::nullopt,
        "cancellation requested; waiting for the next safe point"});
  return Result<void>::Ok();
}

bool PipelineController::cancellationRequested() const {
  return cancel_requested_.load() ||
         (cancellation_ && cancellation_->IsCancellationRequested());
}

void PipelineController::finish(uint64_t job_id, const Result<void>& result) {
  JobState state = JobState::kSucceeded;
  std::string message;
  if (!result) {
    message = result.GetError().Message();
    state = result.GetError().code == Error::Code::kCancelled
                ? JobState::kCancelled : JobState::kFailed;
  }
  {
    std::lock_guard lock(mutex_);
    job_->state = state;
    job_->active_stage.reset();
    job_->message = message;
  }
  emit({job_id, state == JobState::kCancelled ? EventType::kJobCancelled
                                               : EventType::kJobCompleted,
        std::nullopt, message});
  completed_.notify_all();
}

Result<void> PipelineController::Wait(uint64_t job_id) {
  std::unique_lock lock(mutex_);
  if (!job_ || job_->id != job_id) {
    return Result<void>::Failure(Error::InvalidArgument("unknown job id"));
  }
  completed_.wait(lock, [this, job_id] {
    return !job_ || job_->id != job_id ||
           job_->state == JobState::kSucceeded ||
           job_->state == JobState::kFailed ||
           job_->state == JobState::kCancelled;
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

PipelineSnapshot PipelineController::Snapshot() const {
  PipelineSnapshot snapshot;
  {
    std::lock_guard lock(mutex_);
    snapshot.job = job_;
    snapshot.config_revision = config_revision_;
    snapshot.agents = runner_ ? runner_->AgentIds() : std::vector<char>{};
    snapshot.recent_events = recent_events_;
  }
  snapshot.artifacts = artifacts_.Snapshot();
  return snapshot;
}

Result<VisualizationSnapshot> PipelineController::GetVisualizationSnapshot(
    char agent, std::size_t max_points) const {
  std::shared_ptr<StageRunner> runner;
  {
    std::lock_guard lock(mutex_);
    runner = runner_;
  }
  if (!runner) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("stage runner is not available"));
  }
  auto result = runner->CreateVisualizationSnapshot(agent, max_points);
  if (!result) return result;
  auto snapshot = std::move(result).Value();
  for (const auto& artifact : artifacts_.Snapshot()) {
    if (artifact.key.agent == agent && artifact.state == ArtifactState::kReady &&
        (artifact.key.type == ArtifactType::kOptimizedPoses ||
         artifact.key.type == ArtifactType::kGlobalMap)) {
      snapshot.revision = std::max(snapshot.revision, artifact.revision);
    }
  }
  return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
}

std::optional<AlignmentFeedbackSnapshot>
PipelineController::GetAlignmentFeedbackSnapshot() const {
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
    std::lock_guard slot_lock(slot->mutex);
    slot->active = false;
  });
}

void PipelineController::emit(ExecutionEvent event) {
  std::vector<std::shared_ptr<ExecutionEventSubscriberSlot>> subscribers;
  {
    std::lock_guard lock(mutex_);
    event.sequence = next_event_sequence_++;
    recent_events_.push_back(event);
    if (recent_events_.size() > 256) recent_events_.erase(recent_events_.begin());
  }
  {
    std::lock_guard lock(event_subscribers_->mutex);
    subscribers.reserve(event_subscribers_->callbacks.size());
    for (const auto& [id, subscriber] : event_subscribers_->callbacks) {
      (void)id;
      subscribers.push_back(subscriber);
    }
  }
  for (const auto& subscriber : subscribers) {
    std::lock_guard lock(subscriber->mutex);
    if (subscriber->active) subscriber->callback(event);
  }
}

}  // namespace open_lmm
