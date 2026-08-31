#include <iostream>
#include <mutex>
#include <string>

#include <open_lmm/server/runtime_client.hpp>
#include <foundation/logging/logging.hpp>

int main(int argc, char** argv) {
  open_lmm::InitializeLogging();
  if (argc == 2 && std::string(argv[1]) == "--help") {
    std::cout << "Usage: " << argv[0] << " <config_dir_path>\n";
    return 0;
  }
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <config_dir_path>" << std::endl;
    return 1;
  }

  open_lmm::RuntimeClient runtime(1);
  auto opened = runtime.Open({argv[1], "batch"});
  if (!opened) {
    open_lmm::LogError(opened.GetError().Message());
    return 1;
  }
  std::mutex progress_mutex;
  std::string active_progress_stream;
  auto progress_subscription = runtime.SubscribeEvents(
      [&progress_mutex, &active_progress_stream](
          const open_lmm::ExecutionEvent& event) {
        if (!event.algorithm_progress) return;
        const auto& progress = *event.algorithm_progress;
        std::lock_guard lock(progress_mutex);
        const std::string agent = progress.agent.IsValid()
                                      ? progress.agent.Value()
                                      : std::string("runtime");
        const std::string stream =
            agent + ":" + std::to_string(static_cast<int>(progress.phase));
        if (!active_progress_stream.empty() &&
            active_progress_stream != stream) {
          std::cerr << '\n';
        }
        active_progress_stream = stream;
        std::cerr << '\r' << '[' << agent << "] "
                  << open_lmm::FormatAlgorithmProgressStatus(progress);
        if (!progress.total || progress.current == *progress.total) {
          std::cerr << '\n';
          active_progress_stream.clear();
        } else {
          std::cerr << std::flush;
        }
      });
  if (!progress_subscription) {
    open_lmm::LogError(progress_subscription.GetError().Message());
    return 1;
  }
  auto submitted = runtime.SubmitRunAll();
  if (!submitted) {
    open_lmm::LogError(submitted.GetError().Message());
    return 1;
  }
  auto completed = runtime.Wait(submitted.Value());
  if (!completed) {
    open_lmm::LogError(completed.GetError().Message());
    return 1;
  }

  return 0;
}
