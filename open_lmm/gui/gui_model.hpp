#pragma once

#include <open_lmm/server/pipeline_controller.hpp>

#include <map>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm {

enum class GuiStageState : uint8_t {
  kIdle, kRunning, kSucceeded, kFailed, kCancelling, kCancelled
};

struct GuiStageView {
  GuiStageState state = GuiStageState::kIdle;
  uint64_t progress_current = 0;
  uint64_t progress_total = 0;
  std::string message;
  std::map<AgentId, AlgorithmProgress> agent_progress;
  std::optional<AgentId> latest_progress_agent;
};

class GuiModel {
 public:
  GuiModel();
  void Synchronize(PipelineSnapshot snapshot);
  // false means an event gap was detected and a full snapshot is required.
  bool Apply(const ExecutionEvent& event);

  [[nodiscard]] bool CanSubmitCommand() const;
  [[nodiscard]] bool CanCancel() const;
  [[nodiscard]] uint64_t LastSequence() const;
  [[nodiscard]] uint64_t ConfigRevision() const;
  [[nodiscard]] uint64_t RuntimeRevision() const;
  [[nodiscard]] const std::vector<AgentId>& Agents() const;
  [[nodiscard]] const std::vector<ArtifactMetadata>& Artifacts() const;
  [[nodiscard]] const std::optional<JobSnapshot>& Job() const;
  [[nodiscard]] const GuiStageView& Stage(StageId stage) const;
  [[nodiscard]] const std::vector<ExecutionEvent>& EventLog() const;

 private:
  static bool IsActive(JobState state);
  std::optional<JobSnapshot> job_;
  uint64_t config_revision_ = 0;
  uint64_t runtime_revision_ = 0;
  uint64_t last_sequence_ = 0;
  std::vector<AgentId> agents_;
  std::vector<ArtifactMetadata> artifacts_;
  std::map<StageId, GuiStageView> stages_;
  std::vector<ExecutionEvent> event_log_;
};

}  // namespace open_lmm
