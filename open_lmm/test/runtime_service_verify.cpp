#include <open_lmm/server/runtime_service.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

namespace {
using namespace open_lmm;
namespace fs = std::filesystem;

bool IsTerminal(JobState state) {
  return state == JobState::kSucceeded || state == JobState::kFailed ||
         state == JobState::kCancelled;
}

}  // namespace

int main(int argc, char** argv) {
  if (argc != 4) {
    std::cerr << "usage: open_lmm_runtime_service_verify "
                 "<config-one> <config-two> <output-root>\n";
    return 2;
  }
  RuntimeService service(2);
  auto one = service.CreateSession({argv[1], "dataset-one", fs::path(argv[3])});
  auto two = service.CreateSession({argv[2], "dataset-two", fs::path(argv[3])});
  if (!one || !two) {
    std::cerr << "session creation failed: "
              << (!one ? one.GetError().Message() : two.GetError().Message())
              << '\n';
    return 3;
  }
  auto one_job = service.Submit(one.Value(), {});
  auto two_job = service.Submit(two.Value(), {});
  if (!one_job || !two_job) {
    std::cerr << "concurrent submission failed\n";
    return 4;
  }

  const auto running_deadline = std::chrono::steady_clock::now() +
                                std::chrono::seconds(10);
  while (std::chrono::steady_clock::now() < running_deadline) {
    auto snapshot = service.Snapshot(one.Value());
    if (snapshot && snapshot.Value().pipeline.job &&
        snapshot.Value().pipeline.job->state == JobState::kRunning) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  if (!service.Cancel(one.Value(), one_job.Value())) {
    std::cerr << "session-one cancellation failed\n";
    return 5;
  }
  if (!service.CloseSession(one.Value(), CloseMode::kCancelAndWait)) {
    std::cerr << "session-one close failed\n";
    return 6;
  }

  RuntimeSessionSnapshot completed = service.Snapshot(two.Value()).Value();
  const auto completion_deadline = std::chrono::steady_clock::now() +
                                   std::chrono::minutes(2);
  while ((!completed.pipeline.job ||
          !IsTerminal(completed.pipeline.job->state)) &&
         std::chrono::steady_clock::now() < completion_deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
    auto snapshot = service.Snapshot(two.Value());
    if (!snapshot) return 7;
    completed = snapshot.Value();
  }
  if (!completed.pipeline.job ||
      completed.pipeline.job->state != JobState::kSucceeded) {
    std::cerr << "session-two did not succeed\n";
    return 8;
  }
  const fs::path output = completed.output_directory;
  if (!fs::is_regular_file(output / "agent_manifest.json") ||
      !fs::is_regular_file(output / "optimized_poses_test1.txt") ||
      !fs::is_regular_file(output / "optimized_poses_test2.txt")) {
    std::cerr << "session-two output namespace is incomplete: " << output
              << '\n';
    return 9;
  }
  if (!service.CloseSession(two.Value(), CloseMode::kRejectIfRunning)) {
    std::cerr << "session-two close failed\n";
    return 10;
  }
  std::cout << "runtime service dataset verification passed\n"
            << "  completed_output=" << output << '\n';
  return 0;
}
