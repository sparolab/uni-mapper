#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

namespace open_lmm::test::benchmark {

struct RunnerOptions {
  std::string profile;
  std::string scenario;
  std::filesystem::path fixture_root;
  std::filesystem::path report_path;
  uint64_t iteration = 0;
  std::string git_commit;
  bool git_dirty = false;
  std::string compiler;
  std::string build_type;
  std::string sanitizer;
  std::string container_digest;
};

RunnerOptions ParseRunnerOptions(int argc, char** argv);

}  // namespace open_lmm::test::benchmark
