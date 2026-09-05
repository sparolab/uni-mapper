#pragma once

#include <open_lmm/common/runtime_api.hpp>
#include <rclcpp_action/types.hpp>

#include <optional>

namespace open_lmm {

enum class RosGoalPhase {
  kIdle,
  kAccepted,
  kPendingSubmit,
  kActiveJob,
  kTerminal,
};

struct RosCancelDisposition {
  bool accepted = false;
  std::optional<JobHandle> job;
};

struct RosJobPublication {
  bool matched = false;
  bool cancel_pending = false;
};

// Exact-goal action state. OpenLMMROS serializes every call with action_mutex_.
// Keeping UUID, phase, pending cancellation and the public job in one object
// prevents a cancel for an old ROS goal from reaching a newer runtime job.
class RosGoalCoordinator {
 public:
  bool TryAccept(const rclcpp_action::GoalUUID& uuid) {
    if (phase_ != RosGoalPhase::kIdle) return false;
    uuid_ = uuid;
    phase_ = RosGoalPhase::kAccepted;
    cancel_pending_ = false;
    job_.reset();
    return true;
  }

  bool BeginSubmit(const rclcpp_action::GoalUUID& uuid) {
    if (!Matches(uuid) || phase_ != RosGoalPhase::kAccepted) return false;
    phase_ = RosGoalPhase::kPendingSubmit;
    return true;
  }

  RosCancelDisposition RequestCancel(
      const rclcpp_action::GoalUUID& uuid) {
    if (!Matches(uuid)) return {};
    if (phase_ == RosGoalPhase::kAccepted ||
        phase_ == RosGoalPhase::kPendingSubmit) {
      cancel_pending_ = true;
      return {true, std::nullopt};
    }
    if (phase_ == RosGoalPhase::kActiveJob && job_) {
      cancel_pending_ = true;
      return {true, job_};
    }
    return {};
  }

  RosJobPublication PublishJob(const rclcpp_action::GoalUUID& uuid,
                               JobHandle job) {
    if (!Matches(uuid) || phase_ != RosGoalPhase::kPendingSubmit) return {};
    job_ = job;
    phase_ = RosGoalPhase::kActiveJob;
    return {true, cancel_pending_};
  }

  [[nodiscard]] bool CancelPending(
      const rclcpp_action::GoalUUID& uuid) const {
    return Matches(uuid) && cancel_pending_;
  }

  void RejectCancel(const rclcpp_action::GoalUUID& uuid) {
    if (Matches(uuid)) cancel_pending_ = false;
  }

  [[nodiscard]] std::optional<JobHandle> ActiveJob() const { return job_; }

  [[nodiscard]] bool Matches(const rclcpp_action::GoalUUID& uuid) const {
    return phase_ != RosGoalPhase::kIdle && uuid_ && *uuid_ == uuid;
  }

  bool MarkTerminal(const rclcpp_action::GoalUUID& uuid) {
    if (!Matches(uuid) || phase_ == RosGoalPhase::kTerminal) return false;
    phase_ = RosGoalPhase::kTerminal;
    job_.reset();
    cancel_pending_ = false;
    return true;
  }

  void ReleaseTerminal(const rclcpp_action::GoalUUID& uuid) {
    if (!Matches(uuid) || phase_ != RosGoalPhase::kTerminal) return;
    phase_ = RosGoalPhase::kIdle;
    uuid_.reset();
  }

  [[nodiscard]] RosGoalPhase Phase() const { return phase_; }

 private:
  RosGoalPhase phase_ = RosGoalPhase::kIdle;
  std::optional<rclcpp_action::GoalUUID> uuid_;
  std::optional<JobHandle> job_;
  bool cancel_pending_ = false;
};

}  // namespace open_lmm
