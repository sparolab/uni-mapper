#include "../../ros2/open_lmm_ros/runtime_adapter/goal_admission.hpp"
#include "../../ros2/open_lmm_ros/runtime_adapter/action_terminal.hpp"

#include <atomic>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

int main() {
  open_lmm::GoalAdmissionGate gate;
  std::atomic<int> accepted{0};
  std::vector<std::thread> contenders;
  for (int index = 0; index < 32; ++index) {
    contenders.emplace_back([&] {
      if (gate.TryReserve()) accepted.fetch_add(1);
    });
  }
  for (auto& contender : contenders) contender.join();
  if (accepted.load() != 1 || !gate.IsReserved()) {
    std::cerr << "concurrent goal admission accepted more than one goal\n";
    return EXIT_FAILURE;
  }
  gate.Release();
  if (!gate.TryReserve()) {
    std::cerr << "terminal release did not reopen goal admission\n";
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
