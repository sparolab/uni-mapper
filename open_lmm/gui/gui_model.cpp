#include <open_lmm/gui/gui_model.hpp>

#include <algorithm>

namespace open_lmm {
namespace {

void ClearProgress(GuiStageView& stage) {
  stage.progress_current = 0;
  stage.progress_total = 0;
  stage.agent_progress.clear();
  stage.latest_progress_agent.reset();
}

}  // namespace

GuiModel::GuiModel() {
  for (StageId stage : {StageId::kDataLoad, StageId::kAlignment,
                        StageId::kMapUpdate, StageId::kSave}) {
    stages_.emplace(stage, GuiStageView{});
  }
}

bool GuiModel::IsActive(JobState state) {
  return state == JobState::kQueued ||
         state == JobState::kWaitingForDependency ||
         state == JobState::kRunning ||
         state == JobState::kWaitingForAlignmentFeedback ||
         state == JobState::kCancelling;
}

void GuiModel::Synchronize(PipelineSnapshot snapshot) {
  job_ = std::move(snapshot.job);
  config_revision_ = snapshot.config_revision;
  runtime_revision_ = snapshot.runtime_revision;
  agents_ = std::move(snapshot.agents);
  artifacts_ = std::move(snapshot.artifacts);
  event_log_.clear();
  event_log_.reserve(snapshot.recent_events.size());
  last_sequence_ = 0;
  for (auto& [stage, view] : stages_) {
    (void)stage;
    view = {};
  }
  for (const auto& event : snapshot.recent_events) {
    last_sequence_ = event.sequence > 0 ? event.sequence - 1 : 0;
    Apply(event);
  }
}

bool GuiModel::Apply(const ExecutionEvent& event) {
  if (event.sequence <= last_sequence_) return true;
  if (last_sequence_ != 0 && event.sequence != last_sequence_ + 1) return false;
  last_sequence_ = event.sequence;
  event_log_.push_back(event);
  if (event_log_.size() > 256) event_log_.erase(event_log_.begin());

  if (!job_ || (event.job_id != 0 && job_->id != event.job_id)) {
    if (event.job_id != 0) job_ = JobSnapshot{event.job_id};
  }
  if (event.job_id != 0 && job_) {
    job_->message = event.message;
    job_->active_stage = event.stage;
    if (event.cancellation) job_->cancellation = *event.cancellation;
  }

  switch (event.type) {
    case EventType::kJobQueued:
      job_->state = JobState::kQueued;
      break;
    case EventType::kJobStarted:
      job_->state = JobState::kRunning;
      break;
    case EventType::kCancellationRequested:
      job_->state = JobState::kCancelling;
      if (event.stage) stages_[*event.stage].state = GuiStageState::kCancelling;
      break;
    case EventType::kAlignmentFeedbackRequested:
      job_->state = JobState::kWaitingForAlignmentFeedback;
      break;
    case EventType::kAlignmentProposalAccepted:
    case EventType::kAlignmentProposalRejected:
    case EventType::kAlignmentFeedbackCancelled:
      job_->state = JobState::kRunning;
      break;
    case EventType::kJobCancelled:
      job_->state = JobState::kCancelled;
      job_->active_stage.reset();
      for (auto& [stage_id, stage] : stages_) {
        (void)stage_id;
        ClearProgress(stage);
      }
      break;
    case EventType::kJobCompleted:
      job_->state = event.message.empty() ? JobState::kSucceeded
                                          : JobState::kFailed;
      job_->active_stage.reset();
      break;
    case EventType::kStageStarted:
      if (event.stage) stages_[*event.stage] = {GuiStageState::kRunning};
      break;
    case EventType::kProgressUpdated:
      if (event.stage) {
        auto& stage = stages_[*event.stage];
        stage.progress_current = event.progress_current;
        stage.progress_total = event.progress_total;
        if (event.algorithm_progress && event.agent) {
          stage.agent_progress[*event.agent] = *event.algorithm_progress;
          stage.latest_progress_agent = *event.agent;
        }
      }
      break;
    case EventType::kStageCompleted:
      if (event.stage) {
        auto& stage = stages_[*event.stage];
        stage.state = GuiStageState::kSucceeded;
        ClearProgress(stage);
      }
      break;
    case EventType::kStageFailed:
    case EventType::kNodeFailed:
      if (event.stage) {
        auto& stage = stages_[*event.stage];
        stage.state = GuiStageState::kFailed;
        stage.message = event.error ? event.error->Message() : event.message;
        ClearProgress(stage);
      }
      break;
    case EventType::kArtifactCommitted:
    case EventType::kArtifactInvalidated:
      // Artifact metadata is authoritative in PipelineSnapshot.
      return false;
    default:
      break;
  }
  return true;
}

bool GuiModel::CanSubmitCommand() const {
  return !job_ || !IsActive(job_->state);
}

bool GuiModel::CanCancel() const {
  return job_ && (job_->state == JobState::kQueued ||
                  job_->state == JobState::kWaitingForDependency ||
                  job_->state == JobState::kRunning ||
                  job_->state == JobState::kWaitingForAlignmentFeedback);
}

uint64_t GuiModel::LastSequence() const { return last_sequence_; }
uint64_t GuiModel::ConfigRevision() const { return config_revision_; }
uint64_t GuiModel::RuntimeRevision() const { return runtime_revision_; }
const std::vector<AgentId>& GuiModel::Agents() const { return agents_; }
const std::vector<ArtifactMetadata>& GuiModel::Artifacts() const {
  return artifacts_;
}
const std::optional<JobSnapshot>& GuiModel::Job() const { return job_; }
const GuiStageView& GuiModel::Stage(StageId stage) const {
  return stages_.at(stage);
}
const std::vector<ExecutionEvent>& GuiModel::EventLog() const {
  return event_log_;
}

}  // namespace open_lmm
