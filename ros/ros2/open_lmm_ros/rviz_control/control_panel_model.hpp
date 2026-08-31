#pragma once

#include <open_lmm_ros/action/execute_pipeline.hpp>
#include <open_lmm_ros/msg/execution_event.hpp>
#include <open_lmm_ros/srv/get_runtime_status.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm_ros::control {

enum class Stage : uint8_t { kDataLoad, kAlignment, kMapUpdate, kSave };
enum class Node : uint8_t {
  kDataLoad,
  kLoopDetect,
  kOptimize,
  kMapUpdate,
  kPoseSave,
  kFallbackMapSave,
};

enum class Event : uint8_t {
  kJobQueued = 0,
  kJobStarted = 1,
  kCancellationRequested = 10,
  kJobCompleted = 16,
  kJobCancelled = 17,
};

static_assert(static_cast<uint8_t>(Stage::kDataLoad) == 0);
static_assert(static_cast<uint8_t>(Stage::kSave) == 3);
static_assert(static_cast<uint8_t>(Node::kDataLoad) == 0);
static_assert(static_cast<uint8_t>(Node::kFallbackMapSave) == 5);
static_assert(static_cast<uint8_t>(Event::kJobQueued) == 0);
static_assert(static_cast<uint8_t>(Event::kJobCompleted) == 16);
static_assert(static_cast<uint8_t>(Event::kJobCancelled) == 17);

[[nodiscard]] inline bool NodeRequiresAgent(Node node) {
  return node == Node::kDataLoad || node == Node::kLoopDetect ||
         node == Node::kOptimize || node == Node::kMapUpdate;
}

[[nodiscard]] inline std::optional<action::ExecutePipeline::Goal> StageGoal(
    Stage stage) {
  action::ExecutePipeline::Goal goal;
  goal.kind = action::ExecutePipeline::Goal::STAGE;
  goal.has_stage = true;
  goal.stage = static_cast<uint8_t>(stage);
  return goal;
}

[[nodiscard]] inline std::optional<action::ExecutePipeline::Goal> NodeGoal(
    Node node, const std::string& agent) {
  if (NodeRequiresAgent(node) && agent.empty()) return std::nullopt;
  action::ExecutePipeline::Goal goal;
  goal.kind = action::ExecutePipeline::Goal::NODE;
  goal.has_node = true;
  goal.node = static_cast<uint8_t>(node);
  goal.has_agent = NodeRequiresAgent(node);
  goal.agent = goal.has_agent ? agent : std::string{};
  return goal;
}

[[nodiscard]] inline std::optional<action::ExecutePipeline::Goal>
OptimizeThroughGoal(const std::string& agent) {
  if (agent.empty()) return std::nullopt;
  action::ExecutePipeline::Goal goal;
  goal.kind = action::ExecutePipeline::Goal::OPTIMIZE_THROUGH;
  goal.has_agent = true;
  goal.agent = agent;
  return goal;
}

class ControlPanelModel {
 public:
  void Disconnect(std::string message) {
    connected_ = false;
    can_submit_ = false;
    can_cancel_ = false;
    message_ = std::move(message);
  }

  void ApplyStatus(const srv::GetRuntimeStatus::Response& status) {
    connected_ = status.success;
    if (!status.success) {
      can_submit_ = false;
      can_cancel_ = false;
      message_ = status.error;
      return;
    }
    runtime_state_ = status.runtime_state;
    runtime_revision_ = status.runtime_revision;
    config_revision_ = status.config_revision;
    agents_ = status.agents;
    if (!status.has_job || !has_job_ || job_id_ != status.job_id) {
      progress_current_ = 0;
      progress_total_ = 0;
    }
    has_job_ = status.has_job;
    job_id_ = status.job_id;
    job_state_ = status.job_state;
    active_stage_ = status.has_active_stage
                        ? std::optional<uint8_t>(status.active_stage)
                        : std::nullopt;
    message_ = status.job_message;
    const bool active = status.has_job && status.job_state <= 4;
    can_submit_ = !active;
    can_cancel_ = active;
    // Status is the authoritative baseline. Events following this response
    // establish a new local sequence watermark, including after node restart.
    last_sequence_ = 0;
  }

  // False requests an authoritative status refresh after an event gap.
  bool ApplyEvent(const msg::ExecutionEvent& event) {
    if (event.sequence == 1 && last_sequence_ > 1) last_sequence_ = 0;
    if (event.sequence <= last_sequence_) return true;
    const bool contiguous = last_sequence_ == 0 ||
                            event.sequence == last_sequence_ + 1;
    last_sequence_ = event.sequence;
    if (event.job_id != 0) {
      has_job_ = true;
      job_id_ = event.job_id;
    }
    if (event.has_stage) active_stage_ = event.stage;
    progress_current_ = event.progress_current;
    progress_total_ = event.progress_total;
    message_ = event.has_error ? event.error : event.message;
    const auto type = static_cast<Event>(event.event_type);
    if (type == Event::kJobCompleted || type == Event::kJobCancelled) {
      can_submit_ = false;
      can_cancel_ = false;
      return false;
    }
    can_submit_ = false;
    can_cancel_ = true;
    return contiguous;
  }

  [[nodiscard]] bool Connected() const { return connected_; }
  [[nodiscard]] bool CanSubmit() const { return can_submit_; }
  [[nodiscard]] bool CanCancel() const { return can_cancel_; }
  [[nodiscard]] uint8_t RuntimeState() const { return runtime_state_; }
  [[nodiscard]] uint64_t RuntimeRevision() const { return runtime_revision_; }
  [[nodiscard]] uint64_t ConfigRevision() const { return config_revision_; }
  [[nodiscard]] bool HasJob() const { return has_job_; }
  [[nodiscard]] uint64_t JobId() const { return job_id_; }
  [[nodiscard]] uint8_t JobState() const { return job_state_; }
  [[nodiscard]] std::optional<uint8_t> ActiveStage() const {
    return active_stage_;
  }
  [[nodiscard]] uint64_t ProgressCurrent() const { return progress_current_; }
  [[nodiscard]] uint64_t ProgressTotal() const { return progress_total_; }
  [[nodiscard]] const std::string& Message() const { return message_; }
  [[nodiscard]] const std::vector<std::string>& Agents() const {
    return agents_;
  }

 private:
  bool connected_ = false;
  bool can_submit_ = false;
  bool can_cancel_ = false;
  uint8_t runtime_state_ = 0;
  uint64_t runtime_revision_ = 0;
  uint64_t config_revision_ = 0;
  bool has_job_ = false;
  uint64_t job_id_ = 0;
  uint8_t job_state_ = 0;
  std::optional<uint8_t> active_stage_;
  uint64_t progress_current_ = 0;
  uint64_t progress_total_ = 0;
  uint64_t last_sequence_ = 0;
  std::string message_;
  std::vector<std::string> agents_;
};

}  // namespace open_lmm_ros::control
