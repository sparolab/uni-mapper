#include "support/soak/soak_metrics.hpp"
#include "support/soak/owner_stress_support.hpp"
#include "support/check.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <unistd.h>

namespace soak = open_lmm::test::soak;
namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace {

class TemporaryTree {
 public:
  TemporaryTree() {
    path_ = fs::temp_directory_path() /
            ("open_lmm_soak_metrics_" + std::to_string(::getpid()) + "_" +
             std::to_string(std::chrono::steady_clock::now()
                                .time_since_epoch()
                                .count()));
    fs::create_directories(path_);
  }
  ~TemporaryTree() {
    std::error_code ignored;
    fs::remove_all(path_, ignored);
  }
  const fs::path& Path() const { return path_; }

 private:
  fs::path path_;
};

std::vector<soak::MetricPoint> Series(double slope,
                                      double alternating_noise = 0.0) {
  std::vector<soak::MetricPoint> points;
  for (uint64_t iteration = 0; iteration < 20; ++iteration) {
    const double noise = iteration % 2 == 0 ? alternating_noise
                                             : -alternating_noise;
    points.push_back({iteration, 100.0 + slope * iteration + noise});
  }
  return points;
}

Json ValidReport() {
  return {{"schema_version", 1},
          {"run_id", "metrics-contract"},
          {"profile", "contract"},
          {"scenario", "sampler"},
          {"iterations", 20},
          {"warmup_iterations", 4},
          {"seed", 104729},
          {"git", {{"commit", std::string(40, '0')}, {"dirty", false}}},
          {"build",
           {{"compiler", "contract"},
            {"sanitizer", "none"},
            {"build_type", "test"}}},
          {"machine",
           {{"os", "test"},
            {"kernel", "test"},
            {"cpu_count", 1},
            {"memory_bytes", 1},
            {"container_digest", nullptr}}},
          {"samples", Json::array()},
          {"slopes", Json::object()},
          {"latencies", Json::object()},
          {"failures", Json::array()},
          {"result", "pass"}};
}

void TestProcessSampler() {
  std::vector<uint64_t> thread_counts;
  std::vector<uint64_t> fd_counts;
  for (int sample = 0; sample < 25; ++sample) {
    const auto metrics = soak::SampleProcessMetrics();
#if defined(__linux__)
    Check(metrics.unavailable.empty(), "required Linux metrics are available");
    Check(metrics.rss_bytes && *metrics.rss_bytes > 0,
          "RSS sampler returns a positive value");
    Check(metrics.peak_rss_bytes &&
              *metrics.peak_rss_bytes >= *metrics.rss_bytes,
          "peak RSS is at least current RSS");
    Check(metrics.thread_count && *metrics.thread_count >= 1,
          "thread sampler includes the current thread");
    Check(metrics.fd_count.has_value(), "fd sampler is available");
    Check(metrics.cpu_time_ns.has_value(), "CPU time sampler is available");
    thread_counts.push_back(*metrics.thread_count);
    fd_counts.push_back(*metrics.fd_count);
#endif
    const Json encoded = soak::ProcessMetricsJson(metrics);
    Check(encoded.contains("rss_bytes") && encoded.contains("unavailable"),
          "process metrics have canonical JSON fields");
  }
#if defined(__linux__)
  Check(*std::min_element(thread_counts.begin(), thread_counts.end()) ==
            *std::max_element(thread_counts.begin(), thread_counts.end()),
        "sampler does not create threads");
  Check(*std::min_element(fd_counts.begin(), fd_counts.end()) ==
            *std::max_element(fd_counts.begin(), fd_counts.end()),
        "sampler excludes its own directory fd");
#endif
}

void TestSlopeAnalysis() {
  const auto flat = soak::AnalyzeSlope(Series(0.0, 1.0), 4);
  Check(flat.sample_count == 16 && flat.warmup_samples == 4 &&
            std::abs(flat.theil_sen_per_iteration) < 1e-12,
        "Theil-Sen rejects alternating noise on a flat series");

  const auto rising = soak::AnalyzeSlope(Series(3.5), 4);
  Check(std::abs(rising.theil_sen_per_iteration - 3.5) < 1e-12 &&
            rising.quarter_median_delta > 0.0,
        "positive slope and plateau delta are reported");

  auto invalid = Series(1.0);
  invalid[5].iteration = invalid[4].iteration;
  bool rejected = false;
  try {
    (void)soak::AnalyzeSlope(invalid, 4);
  } catch (const std::invalid_argument&) {
    rejected = true;
  }
  Check(rejected, "duplicate iteration is rejected");
}

void TestOwnerSeriesGate() {
  soak::RunOptions options;
  options.iterations = 20;
  options.warmup = 4;
  soak::ProcessSeries transient;
  for (uint64_t iteration = 0; iteration < options.iterations; ++iteration) {
    transient.rss.push_back({iteration, 100.0});
    transient.threads.push_back({iteration, iteration == 10 ? 2.0 : 1.0});
    transient.fds.push_back({iteration, 3.0});
  }
  Json report = ValidReport();
  soak::FinishOwnerReport(options, transient, report);
  Check(report.at("result") == "pass",
        "zero-slope owner series permits a transient sampled thread");

  auto retained = transient;
  for (std::size_t index = 10; index < retained.threads.size(); ++index)
    retained.threads[index].value = 2.0;
  report = ValidReport();
  bool rejected = false;
  try {
    soak::FinishOwnerReport(options, retained, report);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  Check(rejected, "retained thread count misses the final baseline gate");
}

void TestReportContract() {
  Json report = ValidReport();
  Check(soak::ValidateSoakReport(report).Ok(), "valid soak report accepted");
  report["parallel_state_owner"] = true;
  Check(!soak::ValidateSoakReport(report).Ok(), "unknown report field rejected");

  report = ValidReport();
  report["warmup_iterations"] = report["iterations"];
  Check(!soak::ValidateSoakReport(report).Ok(),
        "warmup must be smaller than iterations");

  report = ValidReport();
  report["git"]["unexpected"] = true;
  Check(!soak::ValidateSoakReport(report).Ok(),
        "unknown nested metadata field rejected");

  report = ValidReport();
  report["schema_version"] = "1";
  report["build"]["compiler"] = 1;
  report["machine"]["cpu_count"] = 0;
  report["machine"]["container_digest"] = "sha256:ABC";
  Check(!soak::ValidateSoakReport(report).Ok(),
        "wrong scalar metadata types and constraints are rejected");

  report = ValidReport();
  report["samples"].push_back(
      {{"iteration", 0},
       {"checkpoint", "after_close"},
       {"process",
        {{"rss_bytes", 1},
         {"peak_rss_bytes", 1},
         {"thread_count", 1},
         {"fd_count", 1},
         {"cpu_time_ns", 1},
         {"unavailable", Json::array()}}},
       {"runtime_revision", nullptr},
       {"recent_event_count", nullptr},
       {"reserved_memory_bytes", 0},
       {"executor", nullptr}});
  Check(soak::ValidateSoakReport(report).Ok(),
        "closed canonical process sample accepted");
  report["samples"][0]["process"]["hidden_counter"] = 1;
  Check(!soak::ValidateSoakReport(report).Ok(),
        "unknown nested sample field rejected");

  report = ValidReport();
  report["samples"].push_back(
      {{"iteration", 0},
       {"checkpoint", "invalid_types"},
       {"process",
        {{"rss_bytes", -1},
         {"peak_rss_bytes", nullptr},
         {"thread_count", nullptr},
         {"fd_count", nullptr},
         {"cpu_time_ns", nullptr},
         {"unavailable", {"rss_bytes", "rss_bytes"}}}},
       {"runtime_revision", nullptr},
       {"recent_event_count", nullptr},
       {"reserved_memory_bytes", nullptr},
       {"executor",
        {{"worker_count", "one"}, {"queue_capacity", 0},
         {"queued_tasks", 0}, {"active_tasks", 0},
         {"waiting_submitters", 0}, {"completed_tasks", 0},
         {"cancelled_queued_tasks", 0}}}});
  Check(!soak::ValidateSoakReport(report).Ok(),
        "invalid process and executor metric types are rejected");

  report = ValidReport();
  report["slopes"]["rss"] =
      {{"sample_count", 3}, {"warmup_samples", 0},
       {"theil_sen_per_iteration", "flat"},
       {"previous_quarter_median", 1.0}, {"last_quarter_median", 1.0},
       {"quarter_median_delta", 0.0}, {"minimum", 1.0}, {"maximum", 1.0}};
  report["latencies"]["operation"] =
      {{"samples", 0}, {"p50_ms", -1.0}, {"p95_ms", 0.0},
       {"max_ms", 0.0}};
  report["failures"].push_back(
      {{"iteration", -1}, {"phase", ""}, {"message", 1}});
  Check(!soak::ValidateSoakReport(report).Ok(),
        "invalid analysis and failure value types are rejected");

  report = ValidReport();
  report["samples"].push_back(
      {{"iteration", 0},
       {"checkpoint", "owner_idle"},
       {"process",
        {{"rss_bytes", 1}, {"peak_rss_bytes", 1}, {"thread_count", 1},
         {"fd_count", 1}, {"cpu_time_ns", 1},
         {"unavailable", Json::array()}}},
       {"runtime_revision", nullptr},
       {"recent_event_count", nullptr},
       {"reserved_memory_bytes", nullptr},
       {"executor", nullptr},
       {"owner",
        {{"live_jobs", nullptr}, {"live_subscriptions", nullptr},
         {"callbacks", nullptr}, {"visualization_cache_entries", 0},
         {"visualization_cache_bytes", 0}, {"output_final_files", nullptr},
         {"output_temporary_files", nullptr},
         {"output_backup_entries", nullptr},
         {"output_recovery_manifests", nullptr},
         {"output_directory_count", nullptr},
         {"loaded_plugin_mappings", nullptr}}}});
  Check(soak::ValidateSoakReport(report).Ok(),
        "closed canonical owner sample accepted");
  report["samples"][0]["owner"]["parallel_owner"] = 1;
  Check(!soak::ValidateSoakReport(report).Ok(),
        "unknown owner metric rejected");

  TemporaryTree tree;
  const fs::path report_path = tree.Path() / "result/report.json";
  report = ValidReport();
  soak::WriteJsonExclusive(report_path, report);
  Check(fs::is_regular_file(report_path), "exclusive report writer creates file");
  bool rejected = false;
  try {
    soak::WriteJsonExclusive(report_path, report);
  } catch (const std::runtime_error&) {
    rejected = true;
  }
  Check(rejected, "exclusive report writer rejects overwrite");
}

}  // namespace

int main() {
  TestProcessSampler();
  TestSlopeAnalysis();
  TestOwnerSeriesGate();
  TestReportContract();
  std::cout << "Soak metric contract tests passed\n";
  return 0;
}
