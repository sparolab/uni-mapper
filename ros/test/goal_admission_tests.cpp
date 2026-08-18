#include "../ros2/open_lmm_ros/goal_admission.hpp"

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
  return EXIT_SUCCESS;
}
