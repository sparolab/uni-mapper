#include "../../ros2/open_lmm_ros/runtime_adapter/goal_coordinator.hpp"
#include "../../ros2/open_lmm_ros/runtime_adapter/action_terminal.hpp"

#include <cstdlib>
#include <iostream>

int main() {
  rclcpp_action::GoalUUID first{};
  rclcpp_action::GoalUUID other{};
  first[0] = 1;
  other[0] = 2;
  open_lmm::RosGoalCoordinator goals;
  if (!goals.TryAccept(first) || goals.TryAccept(other)) {
    std::cerr << "goal admission did not preserve a single exact UUID\n";
    return EXIT_FAILURE;
  }
  if (goals.RequestCancel(other).accepted) {
    std::cerr << "cancel for another UUID reached the reserved goal\n";
    return EXIT_FAILURE;
  }
  const auto accepted_cancel = goals.RequestCancel(first);
  if (!accepted_cancel.accepted || accepted_cancel.job ||
      !goals.CancelPending(first)) {
    std::cerr << "accept-to-submit cancellation was not retained\n";
    return EXIT_FAILURE;
  }
  if (!goals.BeginSubmit(first)) {
    std::cerr << "accepted goal did not enter pending-submit phase\n";
    return EXIT_FAILURE;
  }
  const auto pending_cancel = goals.RequestCancel(first);
  if (!pending_cancel.accepted || pending_cancel.job) {
    std::cerr << "submit-to-job-publication cancellation was not retained\n";
    return EXIT_FAILURE;
  }
  const open_lmm::JobHandle job{41};
  const auto publication = goals.PublishJob(first, job);
  if (!publication.matched || !publication.cancel_pending) {
    std::cerr << "job publication did not atomically inherit cancellation\n";
    return EXIT_FAILURE;
  }
  const auto active_cancel = goals.RequestCancel(first);
  if (!active_cancel.accepted || !active_cancel.job ||
      active_cancel.job->value != job.value) {
    std::cerr << "active cancellation did not target the exact public job\n";
    return EXIT_FAILURE;
  }
  goals.MarkTerminal(other);
  if (goals.Phase() != open_lmm::RosGoalPhase::kActiveJob) {
    std::cerr << "wrong-goal terminal cleanup cleared the active goal\n";
    return EXIT_FAILURE;
  }
  if (!goals.MarkTerminal(first) ||
      goals.Phase() != open_lmm::RosGoalPhase::kTerminal ||
      goals.TryAccept(other)) {
    std::cerr << "terminal phase did not retain exact goal ownership\n";
    return EXIT_FAILURE;
  }
  goals.ReleaseTerminal(first);
  if (goals.Phase() != open_lmm::RosGoalPhase::kIdle ||
      !goals.TryAccept(other)) {
    std::cerr << "exact terminal cleanup did not reopen goal admission\n";
    return EXIT_FAILURE;
  }
  if (!goals.BeginSubmit(other) ||
      !goals.PublishJob(other, {42}).matched || goals.CancelPending(other) ||
      !goals.RequestCancel(other).accepted || !goals.CancelPending(other)) {
    std::cerr << "active-job cancellation did not retain the ROS request\n";
    return EXIT_FAILURE;
  }

  goals.MarkTerminal(first);
  goals.ReleaseTerminal(first);
  if (goals.Phase() != open_lmm::RosGoalPhase::kActiveJob ||
      !goals.ActiveJob() || goals.ActiveJob()->value != 42) {
    std::cerr << "retired worker cleanup cleared the next goal\n";
    return EXIT_FAILURE;
  }

  goals.RejectCancel(first);
  if (!goals.CancelPending(other)) {
    std::cerr << "stale rejection cleared the current cancellation\n";
    return EXIT_FAILURE;
  }
  goals.RejectCancel(other);
  if (goals.CancelPending(other)) {
    std::cerr << "rejected cancellation retained a pending transport transition\n";
    return EXIT_FAILURE;
  }

  using open_lmm::Error;
  using open_lmm::JobSnapshot;
  using open_lmm::JobState;
  using open_lmm::ResolveRosActionTerminal;
  using open_lmm::Result;
  using open_lmm::RosActionTerminal;
  const auto success = Result<void>::Ok();
  if (ResolveRosActionTerminal(
          success, 1, JobSnapshot{1, JobState::kCancelling}) !=
      RosActionTerminal::kSucceeded) {
    std::cerr << "late ROS cancel overrode a committed success receipt\n";
    return EXIT_FAILURE;
  }
  const auto cancelled =
      Result<void>::Failure(Error::Cancelled("before commit"));
  if (ResolveRosActionTerminal(
          cancelled, 1, JobSnapshot{1, JobState::kSucceeded}) !=
      RosActionTerminal::kSucceeded) {
    std::cerr << "authoritative committed success did not reconcile wait\n";
    return EXIT_FAILURE;
  }
  if (ResolveRosActionTerminal(
          cancelled, 1, JobSnapshot{1, JobState::kCancelled}) !=
      RosActionTerminal::kCanceled) {
    std::cerr << "authoritative cancellation was not preserved\n";
    return EXIT_FAILURE;
  }
  const auto failed =
      Result<void>::Failure(Error::InvalidArgument("execution failed"));
  if (ResolveRosActionTerminal(failed, 1,
                               JobSnapshot{1, JobState::kFailed}) !=
      RosActionTerminal::kAborted) {
    std::cerr << "authoritative failure was not aborted\n";
    return EXIT_FAILURE;
  }
  if (ResolveRosActionTerminal(
          cancelled, 1, JobSnapshot{2, JobState::kSucceeded}) !=
      RosActionTerminal::kCanceled) {
    std::cerr << "mismatched authoritative job reconciled the wrong goal\n";
    return EXIT_FAILURE;
  }
  if (ResolveRosActionTerminal(cancelled, 1, std::nullopt) !=
      RosActionTerminal::kCanceled) {
    std::cerr << "cancel error fallback was not canceled\n";
    return EXIT_FAILURE;
  }
  if (ResolveRosActionTerminal(failed, 1, std::nullopt) !=
      RosActionTerminal::kAborted) {
    std::cerr << "non-cancel error fallback was not aborted\n";
    return EXIT_FAILURE;
  }
  return EXIT_SUCCESS;
}
