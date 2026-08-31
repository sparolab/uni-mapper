#include "support/benchmark/benchmark_options.hpp"
#include "support/benchmark/test_assert.hpp"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <unistd.h>

namespace {

using open_lmm::test::benchmark::Check;

open_lmm::test::benchmark::RunnerOptions Parse(
    std::vector<std::string> arguments) {
  std::vector<char*> argv;
  argv.reserve(arguments.size());
  for (auto& argument : arguments) argv.push_back(argument.data());
  return open_lmm::test::benchmark::ParseRunnerOptions(
      static_cast<int>(argv.size()), argv.data());
}

bool Rejects(std::vector<std::string> arguments) {
  try {
    static_cast<void>(Parse(std::move(arguments)));
    return false;
  } catch (const std::invalid_argument&) {
    return true;
  }
}

}  // namespace

int main() {
  const auto fixture = std::filesystem::temp_directory_path() /
                       ("open_lmm_benchmark_options_" +
                        std::to_string(static_cast<uint64_t>(getpid())) + "_" +
                        std::to_string(std::chrono::steady_clock::now()
                                           .time_since_epoch()
                                           .count()));
  std::filesystem::create_directory(fixture);
  const std::string commit(40, 'a');
  const std::string digest = "sha256:" + std::string(64, 'b');
  const std::vector<std::string> base = {
      "runner", "--profile", "contract", "--scenario", "open",
      "--fixture-root", fixture.string(), "--report", "new.json",
      "--iteration", "1", "--git-commit", commit, "--compiler", "gcc-12",
      "--build-type", "Release", "--sanitizer", "none",
      "--container-digest", digest};
  const auto parsed = Parse(base);
  Check(parsed.profile == "contract" && parsed.scenario == "open" &&
            parsed.iteration == 1 && !parsed.git_dirty,
        "complete runner metadata parses without inference");

  auto dirty_pr = base;
  dirty_pr[2] = "pr";
  dirty_pr.push_back("--git-dirty");
  Check(Rejects(dirty_pr), "required profile rejects a dirty worktree");

  auto duplicate = base;
  duplicate.insert(duplicate.end(), {"--iteration", "2"});
  Check(Rejects(duplicate), "duplicate runner option fails closed");

  auto zero = base;
  zero[10] = "0";
  Check(Rejects(zero), "zero iteration fails closed");

  auto uppercase_commit = base;
  uppercase_commit[12] = std::string(40, 'A');
  Check(Rejects(uppercase_commit),
        "noncanonical git commit metadata fails closed");

  auto unknown = base;
  unknown.insert(unknown.end(), {"--baseline-update", "yes"});
  Check(Rejects(unknown), "unreviewed baseline update mode is unavailable");

  std::error_code error;
  std::filesystem::remove(fixture, error);
  Check(!error, "options test fixture cleanup succeeds");
  std::cout << "benchmark options tests passed\n";
}
