#include "owner_stress_support.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdlib>
#include <stdexcept>
#include <utility>

#include <sys/utsname.h>
#include <unistd.h>

namespace open_lmm::test::soak {
namespace {

using Json = nlohmann::json;

uint64_t ParseUnsigned(const std::string& value, const std::string& option) {
  std::size_t consumed = 0;
  uint64_t parsed = 0;
  try {
    parsed = std::stoull(value, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument(option + " requires an unsigned integer");
  }
  if (consumed != value.size())
    throw std::invalid_argument(option + " requires an unsigned integer");
  return parsed;
}

Json MachineMetadata() {
  utsname value{};
  const bool has_uname = ::uname(&value) == 0;
  const long cpu_count = ::sysconf(_SC_NPROCESSORS_ONLN);
  const long pages = ::sysconf(_SC_PHYS_PAGES);
  const long page_size = ::sysconf(_SC_PAGESIZE);
  const char* digest = std::getenv("OPEN_LMM_CONTAINER_DIGEST");
  return {{"os", has_uname ? value.sysname : "unknown"},
          {"kernel", has_uname ? value.release : "unknown"},
          {"cpu_count", cpu_count > 0 ? Json(cpu_count) : Json(nullptr)},
          {"memory_bytes",
           pages > 0 && page_size > 0
               ? Json(static_cast<uint64_t>(pages) *
                      static_cast<uint64_t>(page_size))
               : Json(nullptr)},
          {"container_digest", digest ? Json(digest) : Json(nullptr)}};
}

}  // namespace

RunOptions ParseRunOptions(int argc, char** argv, int first_option) {
  RunOptions options;
  for (int index = first_option; index < argc; ++index) {
    const std::string option = argv[index];
    auto text = [&]() -> std::string {
      if (++index >= argc)
        throw std::invalid_argument("missing value for " + option);
      return argv[index];
    };
    if (option == "--iterations")
      options.iterations = ParseUnsigned(text(), option);
    else if (option == "--warmup")
      options.warmup = ParseUnsigned(text(), option);
    else if (option == "--seed")
      options.seed = ParseUnsigned(text(), option);
    else if (option == "--profile")
      options.profile = text();
    else if (option == "--git-commit")
      options.git_commit = text();
    else if (option == "--git-dirty")
      options.git_dirty = true;
    else if (option == "--report")
      options.report = std::filesystem::path(text());
    else
      throw std::invalid_argument("unknown option: " + option);
  }
  if (options.iterations < 5 || options.iterations > 5'000 ||
      options.warmup >= options.iterations ||
      options.iterations - options.warmup < 4) {
    throw std::invalid_argument(
        "iterations must be 5..5000 with four post-warmup samples");
  }
  if (options.git_commit.size() != 40 ||
      !std::all_of(options.git_commit.begin(), options.git_commit.end(),
                   [](unsigned char value) { return std::isxdigit(value); })) {
    throw std::invalid_argument(
        "git commit must contain 40 hexadecimal digits");
  }
  return options;
}

Json InitialOwnerReport(const RunOptions& options, std::string scenario,
                        std::string sanitizer) {
  return {{"schema_version", 1},
          {"run_id", scenario + "-" + options.profile},
          {"profile", options.profile},
          {"scenario", std::move(scenario)},
          {"iterations", options.iterations},
          {"warmup_iterations", options.warmup},
          {"seed", options.seed},
          {"git",
           {{"commit", options.git_commit}, {"dirty", options.git_dirty}}},
          {"build",
           {{"compiler", __VERSION__},
            {"sanitizer", std::move(sanitizer)},
            {"build_type", "configured"}}},
          {"machine", MachineMetadata()},
          {"samples", Json::array()},
          {"slopes", Json::object()},
          {"latencies", Json::object()},
          {"failures", Json::array()},
          {"result", "fail"}};
}

Json EmptyOwnerMetrics() {
  return {{"live_jobs", nullptr},
          {"live_subscriptions", nullptr},
          {"callbacks", nullptr},
          {"visualization_cache_entries", nullptr},
          {"visualization_cache_bytes", nullptr},
          {"output_final_files", nullptr},
          {"output_temporary_files", nullptr},
          {"output_backup_entries", nullptr},
          {"output_recovery_manifests", nullptr},
          {"output_directory_count", nullptr},
          {"loaded_plugin_mappings", nullptr}};
}

void AppendOwnerSample(Json& report, uint64_t iteration,
                       std::string checkpoint, const ProcessMetrics& process,
                       Json owner) {
  report["samples"].push_back(
      {{"iteration", iteration},
       {"checkpoint", std::move(checkpoint)},
       {"process", ProcessMetricsJson(process)},
       {"runtime_revision", nullptr},
       {"recent_event_count", nullptr},
       {"reserved_memory_bytes", nullptr},
       {"executor", nullptr},
       {"owner", std::move(owner)}});
}

void AddProcessPoint(ProcessSeries& series, uint64_t iteration,
                     const ProcessMetrics& process) {
  if (!process.rss_bytes || !process.thread_count || !process.fd_count ||
      !process.unavailable.empty()) {
    throw std::runtime_error("required Linux process metric unavailable");
  }
  series.rss.push_back({iteration, static_cast<double>(*process.rss_bytes)});
  series.threads.push_back(
      {iteration, static_cast<double>(*process.thread_count)});
  series.fds.push_back({iteration, static_cast<double>(*process.fd_count)});
}

void FinishOwnerReport(const RunOptions& options, const ProcessSeries& series,
                       Json& report) {
  const auto rss = AnalyzeSlope(series.rss, options.warmup);
  const auto threads = AnalyzeSlope(series.threads, options.warmup);
  const auto fds = AnalyzeSlope(series.fds, options.warmup);
  report["slopes"] = {{"rss_bytes", SlopeAnalysisJson(rss)},
                       {"thread_count", SlopeAnalysisJson(threads)},
                       {"fd_count", SlopeAnalysisJson(fds)}};
  const auto thread_baseline = series.threads.at(options.warmup).value;
  const auto fd_baseline = series.fds.at(options.warmup).value;
  if (threads.theil_sen_per_iteration != 0.0 ||
      fds.theil_sen_per_iteration != 0.0 ||
      series.threads.back().value != thread_baseline ||
      series.fds.back().value != fd_baseline) {
    throw std::runtime_error(
        "thread or fd count has non-zero slope or missed its final baseline");
  }
  report["result"] = "pass";
}

TemporaryDirectory::TemporaryDirectory(std::string prefix) {
  path_ = std::filesystem::temp_directory_path() /
          (std::move(prefix) + "_" + std::to_string(::getpid()) + "_" +
           std::to_string(std::chrono::steady_clock::now()
                              .time_since_epoch()
                              .count()));
  std::filesystem::create_directories(path_);
}

TemporaryDirectory::~TemporaryDirectory() {
  std::error_code ignored;
  std::filesystem::remove_all(path_, ignored);
}

}  // namespace open_lmm::test::soak
