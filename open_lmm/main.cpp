#include <iostream>
#include <string>

#include <open_lmm/server/runtime_client.hpp>
#include <open_lmm/utils/logging.hpp>

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
