#include "benchmark_options.hpp"

#include <algorithm>
#include <cctype>
#include <set>
#include <stdexcept>
#include <string>

namespace open_lmm::test::benchmark {
namespace {

uint64_t ParsePositive(const std::string& text, const std::string& option) {
  std::size_t consumed = 0;
  uint64_t value = 0;
  try {
    value = std::stoull(text, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument(option + " requires a positive integer");
  }
  if (consumed != text.size() || value == 0) {
    throw std::invalid_argument(option + " requires a positive integer");
  }
  return value;
}

bool LowerHex(const std::string& text, std::size_t size) {
  return text.size() == size &&
         std::all_of(text.begin(), text.end(), [](unsigned char character) {
           return (character >= '0' && character <= '9') ||
                  (character >= 'a' && character <= 'f');
         });
}

}  // namespace

RunnerOptions ParseRunnerOptions(int argc, char** argv) {
  RunnerOptions options;
  std::set<std::string> seen;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    if (!seen.emplace(option).second) {
      throw std::invalid_argument("duplicate option: " + option);
    }
    auto value = [&]() -> std::string {
      if (++index >= argc) {
        throw std::invalid_argument("missing value for " + option);
      }
      return argv[index];
    };
    if (option == "--profile") options.profile = value();
    else if (option == "--scenario") options.scenario = value();
    else if (option == "--fixture-root") options.fixture_root = value();
    else if (option == "--report") options.report_path = value();
    else if (option == "--iteration") {
      options.iteration = ParsePositive(value(), option);
    } else if (option == "--git-commit") {
      options.git_commit = value();
    } else if (option == "--git-dirty") {
      options.git_dirty = true;
    } else if (option == "--compiler") {
      options.compiler = value();
    } else if (option == "--build-type") {
      options.build_type = value();
    } else if (option == "--sanitizer") {
      options.sanitizer = value();
    } else if (option == "--container-digest") {
      options.container_digest = value();
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }

  if (!std::set<std::string>{"contract", "pr", "nightly", "external",
                             "gpu"}
           .contains(options.profile)) {
    throw std::invalid_argument("unsupported benchmark profile");
  }
  if (!std::set<std::string>{"open", "data-load", "alignment",
                             "map-update-sequential", "map-update-parallel",
                             "save-fallback", "visualization-cold",
                             "visualization-warm", "full-pipeline",
                             "cancellation"}
           .contains(options.scenario)) {
    throw std::invalid_argument("unsupported benchmark scenario");
  }
  if (options.fixture_root.empty() ||
      !std::filesystem::is_directory(options.fixture_root)) {
    throw std::invalid_argument("fixture root must be an existing directory");
  }
  if (options.report_path.empty() || options.iteration == 0) {
    throw std::invalid_argument("report and iteration are required");
  }
  if (!LowerHex(options.git_commit, 40)) {
    throw std::invalid_argument(
        "git commit must be 40 lowercase hexadecimal digits");
  }
  if (options.profile != "contract" && options.git_dirty) {
    throw std::invalid_argument(
        "pr/nightly/external/gpu benchmark requires a clean worktree");
  }
  for (const auto& [name, text] :
       {std::pair{"compiler", options.compiler},
        std::pair{"build type", options.build_type},
        std::pair{"sanitizer", options.sanitizer}}) {
    if (text.empty()) throw std::invalid_argument(name + std::string(" is required"));
  }
  if (!options.container_digest.starts_with("sha256:") ||
      !LowerHex(options.container_digest.substr(7), 64)) {
    throw std::invalid_argument(
        "container digest must be a lowercase sha256 digest");
  }
  return options;
}

}  // namespace open_lmm::test::benchmark
