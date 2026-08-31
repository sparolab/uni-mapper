#include "support/soak/owner_stress_support.hpp"

#include <storage/transactions/file_set_transaction.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#ifndef OPEN_LMM_SOAK_SANITIZER_NAME
#define OPEN_LMM_SOAK_SANITIZER_NAME "none"
#endif

namespace fs = std::filesystem;
namespace soak = open_lmm::test::soak;
using Json = nlohmann::json;

namespace {

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

void Write(const fs::path& path, const std::string& value) {
  std::ofstream output(path);
  if (!output || !(output << value))
    throw std::runtime_error("failed to write " + path.string());
}

std::string Read(const fs::path& path) {
  std::ifstream input(path);
  std::string value;
  input >> value;
  return value;
}

struct FileCounts {
  uint64_t finals = 0;
  uint64_t temporaries = 0;
  uint64_t backups = 0;
  uint64_t recovery = 0;
  uint64_t directories = 0;
};

FileCounts Count(const fs::path& root) {
  FileCounts counts;
  std::error_code error;
  for (fs::directory_iterator item(root, error), end;
       !error && item != end; item.increment(error)) {
    const std::string name = item->path().filename().string();
    if (item->is_directory()) ++counts.directories;
    if (name.ends_with(".tmp"))
      ++counts.temporaries;
    else if (name.ends_with(".open_lmm_backup"))
      ++counts.backups;
    else if (name.starts_with(".open_lmm_recovery_") &&
             name.ends_with(".json"))
      ++counts.recovery;
    else
      ++counts.finals;
  }
  if (error) throw std::runtime_error("failed to enumerate transaction files");
  return counts;
}

Json Owner(const FileCounts& counts) {
  Json owner = soak::EmptyOwnerMetrics();
  owner["output_final_files"] = counts.finals;
  owner["output_temporary_files"] = counts.temporaries;
  owner["output_backup_entries"] = counts.backups;
  owner["output_recovery_manifests"] = counts.recovery;
  owner["output_directory_count"] = counts.directories;
  return owner;
}

soak::ProcessSeries Run(const soak::RunOptions& options, Json& report) {
  soak::TemporaryDirectory temporary("open_lmm_transaction_soak");
  soak::ProcessSeries series;
  for (uint64_t iteration = 0; iteration < options.iterations; ++iteration) {
    const fs::path root = temporary.Path() / std::to_string(iteration);
    fs::create_directories(root);

    const fs::path original = root / "rollback-a.final";
    const fs::path replacement = root / "rollback-a.tmp";
    const fs::path second = root / "rollback-b.tmp";
    Write(original, "original");
    Write(replacement, "candidate");
    Write(second, "second");
    const auto rolled_back = open_lmm::CommitFileSet(
        {{replacement, original}, {second, root / "missing" / "b.final"}});
    Require(!rolled_back && Read(original) == "original" &&
                !fs::exists(original.string() + ".open_lmm_backup") &&
                !fs::exists(replacement) && fs::is_regular_file(second),
            "pre-commit failure did not restore the previous file set");

    fs::remove(original);
    fs::remove(second);
    const fs::path destination = root / "recovery.final";
    const fs::path recovery_temporary = root / "recovery.tmp";
    fs::create_directories(destination);
    Write(destination / "original", "original");
    Write(recovery_temporary, "committed");
    const auto recovery = open_lmm::CommitFileSet(
        {{recovery_temporary, destination}});
    Require(recovery && recovery.Value().recovery_required &&
                fs::is_regular_file(destination) &&
                Read(destination) == "committed",
            "post-commit cleanup fault did not retain committed authority");
    const FileCounts committed = Count(root);
    Require(committed.finals == 1 && committed.temporaries == 0 &&
                committed.backups == 1 && committed.recovery == 1 &&
                committed.directories == 1,
            "recovery file cardinality is not exact");
    soak::AppendOwnerSample(report, iteration, "recovery_committed",
                            soak::SampleProcessMetrics(), Owner(committed));

    fs::remove_all(root);
    fs::create_directories(root);
    const FileCounts idle = Count(root);
    Require(idle.finals == 0 && idle.temporaries == 0 && idle.backups == 0 &&
                idle.recovery == 0 && idle.directories == 0,
            "transaction fixture cleanup left owned files");
    const auto process = soak::SampleProcessMetrics();
    soak::AppendOwnerSample(report, iteration, "owner_idle", process,
                            Owner(idle));
    soak::AddProcessPoint(series, iteration, process);
    fs::remove_all(root);
  }
  return series;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = soak::ParseRunOptions(argc, argv);
    Json report = soak::InitialOwnerReport(
        options, "transaction-fault", OPEN_LMM_SOAK_SANITIZER_NAME);
    try {
      const auto series = Run(options, report);
      soak::FinishOwnerReport(options, series, report);
    } catch (const std::exception& error) {
      report["failures"].push_back(
          {{"iteration", nullptr},
           {"phase", "file_transaction"},
           {"message", error.what()}});
      report["result"] = "fail";
    }
    const auto validation = soak::ValidateSoakReport(report);
    if (!validation.Ok())
      throw std::runtime_error("invalid soak report:\n" +
                               validation.Summary());
    if (options.report) soak::WriteJsonExclusive(*options.report, report);
    const bool passed = report.at("result") == "pass";
    std::cout << "transaction fault stress=" << (passed ? "PASS" : "FAIL")
              << " iterations=" << options.iterations << '\n';
    return passed ? 0 : 1;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid soak request: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "soak infrastructure failure: " << error.what() << '\n';
    return 2;
  }
}
