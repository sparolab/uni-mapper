#include <runtime/service/runtime_service.hpp>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <thread>

namespace {
bool IsTerminal(open_lmm::JobState state) {
  return state == open_lmm::JobState::kSucceeded ||
         state == open_lmm::JobState::kFailed ||
         state == open_lmm::JobState::kCancelled;
}
}

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "usage: open_lmm_runtime_service_verify <config> <output-root>\n";
    return 2;
  }
  open_lmm::RuntimeService service(2);
  if (!service.Open({argv[1], "batch", std::filesystem::path(argv[2])})) return 3;
  auto job = service.Submit({});
  if (!job) return 4;
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::minutes(2);
  while (std::chrono::steady_clock::now() < deadline) {
    auto snapshot = service.Snapshot();
    if (!snapshot) return 5;
    if (snapshot.Value().pipeline.job &&
        IsTerminal(snapshot.Value().pipeline.job->state)) {
      if (snapshot.Value().pipeline.job->state != open_lmm::JobState::kSucceeded) return 6;
      std::cout << "runtime service single-runtime verification passed\n"
                << "  completed_output=" << snapshot.Value().output_directory << '\n';
      return service.Close().IsOk() ? 0 : 7;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  return 8;
}
