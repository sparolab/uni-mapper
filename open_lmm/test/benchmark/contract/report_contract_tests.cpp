#include "support/benchmark/benchmark_report.hpp"
#include "support/benchmark/benchmark_bundle.hpp"
#include "support/benchmark/benchmark_pair.hpp"
#include "support/benchmark/test_assert.hpp"

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <unistd.h>

namespace {

using Json = nlohmann::json;
namespace fs = std::filesystem;
using open_lmm::test::benchmark::Check;

Json Metrics(std::initializer_list<const char*> fields) {
  Json value = Json::object();
  for (const char* field : fields) value[field] = 0;
  value["unavailable_reasons"] = Json::object();
  return value;
}

Json ValidReport() {
  const std::string digest = "sha256:" + std::string(64, 'a');
  Json report = {
      {"schema_version", 1},
      {"run_id", "small-v1-open-0001"},
      {"measurement_role", "public"},
      {"profile", "pr"},
      {"scenario", "open"},
      {"iteration", 1},
      {"git", {{"commit", std::string(40, 'b')}, {"dirty", false}}},
      {"build",
       {{"compiler", "gcc-12.3.0"},
        {"build_type", "Release"},
        {"sanitizer", "none"},
        {"container_digest", digest}}},
      {"machine",
       {{"os", "Linux"},
        {"kernel", "test"},
        {"cpu_model", "fixture"},
        {"cpu_count", 2},
        {"cpu_affinity", "0-1"},
        {"memory_bytes", 1024},
        {"memory_class", "1-1GiB"}}},
      {"fixture",
       {{"id", "small-v1"},
        {"manifest_sha256", digest},
        {"decoded_point_count", 65536},
        {"decoded_point_bytes", 1048576},
        {"config_fingerprint", digest},
        {"pair_fingerprint", digest},
        {"plugin_ids", {"builtin.free_dom", "builtin.scan_context"}},
        {"parallelism", "map-update-disabled"}}},
      {"timing",
       Metrics({"wall_time_ns", "cpu_time_ns", "stage_latency_ns",
                "command_latency_ns", "cancellation_latency_ns"})},
      {"stage_timings", Json::array()},
      {"process_memory",
       Metrics({"rss_start_bytes", "rss_end_bytes",
                "sampled_peak_rss_bytes", "process_hwm_bytes",
                "target_peak_delta_bytes", "sample_count",
                "sample_interval_ns", "retained_rss_delta_bytes"})},
      {"owner_memory",
       Metrics({"governor_reserved_total_bytes",
                "governor_resident_payload_bytes",
                "governor_transient_task_bytes", "governor_heavy_map_bytes",
                "governor_peak_reserved_total_bytes",
                "governor_peak_resident_payload_bytes",
                "governor_peak_transient_task_bytes",
                "governor_peak_heavy_map_bytes",
                "governor_admission_failures", "executor_worker_count",
                "executor_queue_capacity", "executor_max_queued_tasks",
                "executor_max_active_tasks",
                "executor_max_waiting_submitters"})},
      {"points",
       Metrics({"input_file_bytes", "decoded_source_points",
                "decoded_source_bytes", "retained_filtered_points",
                "retained_capacity_bytes", "optimized_pose_count",
                "map_output_points", "map_output_file_bytes",
                "visualization_source_points",
                "visualization_displayed_points", "cache_entry_points",
                "public_dto_point_bytes", "gui_staging_bytes"})},
      {"io", Metrics({"rchar", "wchar", "syscr", "syscw", "read_bytes",
                       "write_bytes", "cancelled_write_bytes"})},
      {"cache",
       Metrics({"entries", "bytes", "hits", "misses", "insertions",
                "evictions", "clears", "returned_snapshot_bytes",
                "cpu_staging_bytes", "gpu_upload_requested_bytes",
                "gpu_estimated_resident_bytes"})},
      {"artifacts",
       Metrics({"logical_output_bytes", "retained_owner_bytes",
                "sampled_target_peak_delta_bytes", "process_hwm_bytes",
                "peak_to_output_ratio", "peak_to_retained_ratio"})},
      {"failures", Json::array()},
      {"result", "uncalibrated"}};
  report["process_memory"]["memory_confidence"] = "low";
  report["process_memory"]["sample_count"] = 2;
  report["process_memory"]["sample_interval_ns"] = 1000000;
  report["cache"]["gpu_measurement_kind"] = "not_available";
  report["artifacts"]["output_digest"] = digest;
  report["artifacts"]["correctness"] = "pass";
  return report;
}

}  // namespace

int main() {
  using namespace open_lmm::test::benchmark;
  auto report = ValidReport();
  Check(ValidatePerformanceReport(report).Ok(),
        "complete closed performance report is accepted");

  auto missing_role = report;
  missing_role.erase("measurement_role");
  Check(!ValidatePerformanceReport(missing_role).Ok(),
        "raw report requires an explicit public or owner role");

  auto unknown = report;
  unknown["timing"]["mean_time_ns"] = 1;
  Check(!ValidatePerformanceReport(unknown).Ok(),
        "unknown metric is rejected");

  auto negative = report;
  negative["owner_memory"]["governor_reserved_total_bytes"] = -1;
  Check(!ValidatePerformanceReport(negative).Ok(),
        "negative owner bytes are rejected");

  auto unavailable = report;
  unavailable["timing"]["cancellation_latency_ns"] = nullptr;
  Check(!ValidatePerformanceReport(unavailable).Ok(),
        "null metric without a reason is rejected");
  unavailable["timing"]["unavailable_reasons"]
             ["cancellation_latency_ns"] = "not a cancellation scenario";
  Check(ValidatePerformanceReport(unavailable).Ok(),
        "null metric with an explicit reason is accepted");

  auto pipeline = report;
  pipeline["scenario"] = "full-pipeline";
  pipeline["stage_timings"] = {
      {{"stage", "data-load"}, {"latency_ns", 10},
       {"start_sequence", 2}, {"terminal_sequence", 3}},
      {{"stage", "alignment"}, {"latency_ns", 20},
       {"start_sequence", 4}, {"terminal_sequence", 5}},
      {{"stage", "save"}, {"latency_ns", 30},
       {"start_sequence", 6}, {"terminal_sequence", 7}}};
  Check(ValidatePerformanceReport(pipeline).Ok(),
        "full pipeline preserves ordered per-stage event windows");
  pipeline["stage_timings"].erase(pipeline["stage_timings"].begin() + 1);
  Check(!ValidatePerformanceReport(pipeline).Ok(),
        "full pipeline cannot omit an expected stage window");

  const fs::path output = fs::temp_directory_path() /
                          ("open_lmm_benchmark_report_" +
                           std::to_string(static_cast<uint64_t>(getpid())) +
                           ".json");
  std::error_code error;
  fs::remove(output, error);
  WriteJsonExclusive(output, report);
  bool refused_overwrite = false;
  try {
    WriteJsonExclusive(output, report);
  } catch (const std::runtime_error&) {
    refused_overwrite = true;
  }
  std::ifstream written(output);
  Json parsed;
  written >> parsed;
  fs::remove(output, error);
  Check(refused_overwrite && parsed == report,
        "report writer preserves JSON and refuses overwrite");

  const fs::path bundle_root = fs::temp_directory_path() /
                               ("open_lmm_benchmark_bundle_" +
                                std::to_string(static_cast<uint64_t>(getpid())));
  fs::remove_all(bundle_root, error);
  fs::create_directory(bundle_root);
  std::vector<fs::path> report_paths;
  for (uint64_t iteration = 1; iteration <= 5; ++iteration) {
    auto sample = report;
    sample["iteration"] = iteration;
    sample["run_id"] = "small-v1-open-" + std::to_string(iteration);
    sample["timing"]["wall_time_ns"] = iteration * 10;
    const auto sample_path =
        bundle_root / ("raw-" + std::to_string(iteration) + ".json");
    WriteJsonExclusive(sample_path, sample);
    report_paths.push_back(sample_path);
  }
  const auto uncalibrated = AggregatePerformanceReports(report_paths);
  Check(ValidatePerformanceBundle(uncalibrated).Ok() &&
            uncalibrated.at("comparison") == "uncalibrated" &&
            uncalibrated.at("reports").size() == 5,
        "five valid raw reports aggregate without inventing a baseline");
  auto unknown_bundle = uncalibrated;
  unknown_bundle["automatic_baseline_update"] = true;
  Check(!ValidatePerformanceBundle(unknown_bundle).Ok(),
        "bundle validator rejects unknown policy fields");
  const auto wall_metric = std::find_if(
      uncalibrated.at("metrics").begin(), uncalibrated.at("metrics").end(),
      [](const Json& metric) {
        return metric.at("name") == "timing.wall_time_ns";
      });
  Check(wall_metric != uncalibrated.at("metrics").end() &&
            wall_metric->at("summary").at("median") == 30.0 &&
            wall_metric->at("summary").at("p95") == 50,
        "bundle preserves median and nearest-rank p95");

  std::vector<fs::path> signed_rss_paths;
  for (uint64_t iteration = 1; iteration <= 5; ++iteration) {
    auto sample = report;
    sample["iteration"] = iteration;
    sample["run_id"] = "small-v1-signed-rss-" +
                       std::to_string(iteration);
    sample["process_memory"]["retained_rss_delta_bytes"] =
        iteration == 1 ? 4096 : -4096;
    const auto sample_path =
        bundle_root / ("signed-rss-" + std::to_string(iteration) + ".json");
    WriteJsonExclusive(sample_path, sample);
    signed_rss_paths.push_back(sample_path);
  }
  const auto signed_rss_bundle =
      AggregatePerformanceReports(signed_rss_paths);
  const auto retained_metric = std::find_if(
      signed_rss_bundle.at("metrics").begin(),
      signed_rss_bundle.at("metrics").end(), [](const Json& metric) {
        return metric.at("name") ==
               "process_memory.retained_rss_delta_bytes";
      });
  Check(ValidatePerformanceBundle(signed_rss_bundle).Ok() &&
            retained_metric == signed_rss_bundle.at("metrics").end(),
        "signed retained RSS remains raw diagnostic data and does not make "
        "aggregate availability depend on allocator timing");

  Json baseline = {
      {"schema_version", 1},
      {"baseline_id", "reviewed-test-v1"},
      {"key", uncalibrated.at("key")},
      {"source_bundle_sha256", "sha256:" + std::string(64, 'd')},
      {"metrics",
       {{{"name", "timing.wall_time_ns"},
         {"statistic", "median"},
         {"expected", 30},
         {"relative_allowance", 0.0},
         {"absolute_allowance", 0}}}}};
  const auto baseline_path = bundle_root / "baseline.json";
  WriteJsonExclusive(baseline_path, baseline);
  const auto passing =
      AggregatePerformanceReports(report_paths, baseline_path);
  Check(passing.at("comparison") == "pass" &&
            passing.at("comparisons").at(0).at("delta") == 0.0,
        "reviewed exact baseline comparison passes");

  auto other_baseline = baseline;
  other_baseline["baseline_id"] = "reviewed-other-v1";
  other_baseline["key"]["scenario"] = "data-load";
  Json catalog = {{"schema_version", 1},
                  {"catalog_id", "reviewed-catalog-v1"},
                  {"baselines", {other_baseline, baseline}}};
  const auto catalog_path = bundle_root / "catalog.json";
  WriteJsonExclusive(catalog_path, catalog);
  const auto catalog_passing =
      AggregatePerformanceReports(report_paths, catalog_path);
  Check(catalog_passing.at("comparison") == "pass" &&
            catalog_passing.at("baseline").at("id") ==
                "reviewed-test-v1",
        "catalog selects the one exact scenario and environment key");
  std::vector<fs::path> other_report_paths;
  for (uint64_t iteration = 1; iteration <= 5; ++iteration) {
    auto sample = report;
    sample["scenario"] = "data-load";
    sample["iteration"] = iteration;
    sample["run_id"] = "small-v1-data-load-" + std::to_string(iteration);
    sample["timing"]["wall_time_ns"] = iteration * 10;
    const auto sample_path = bundle_root /
                             ("data-load-raw-" +
                              std::to_string(iteration) + ".json");
    WriteJsonExclusive(sample_path, sample);
    other_report_paths.push_back(sample_path);
  }
  const auto other_catalog_passing =
      AggregatePerformanceReports(other_report_paths, catalog_path);
  Check(other_catalog_passing.at("comparison") == "pass" &&
            other_catalog_passing.at("baseline").at("id") ==
                "reviewed-other-v1",
        "one catalog independently selects another scenario key");

  auto unknown_catalog = catalog;
  unknown_catalog["automatic_update"] = true;
  const auto unknown_catalog_path = bundle_root / "catalog-unknown.json";
  WriteJsonExclusive(unknown_catalog_path, unknown_catalog);
  bool unknown_catalog_rejected = false;
  try {
    static_cast<void>(
        AggregatePerformanceReports(report_paths, unknown_catalog_path));
  } catch (const std::invalid_argument&) {
    unknown_catalog_rejected = true;
  }
  Check(unknown_catalog_rejected,
        "closed catalog rejects unknown automatic policy fields");

  auto invalid_key_catalog = catalog;
  invalid_key_catalog["baselines"][0]["key"]["cpu_count"] = 0;
  const auto invalid_key_catalog_path =
      bundle_root / "catalog-invalid-key.json";
  WriteJsonExclusive(invalid_key_catalog_path, invalid_key_catalog);
  bool invalid_key_catalog_rejected = false;
  try {
    static_cast<void>(
        AggregatePerformanceReports(report_paths, invalid_key_catalog_path));
  } catch (const std::invalid_argument&) {
    invalid_key_catalog_rejected = true;
  }
  Check(invalid_key_catalog_rejected,
        "catalog validates every baseline key, including unused entries");

  catalog["baselines"] = Json::array({other_baseline});
  const auto catalog_mismatch_path = bundle_root / "catalog-mismatch.json";
  WriteJsonExclusive(catalog_mismatch_path, catalog);
  const auto catalog_mismatch =
      AggregatePerformanceReports(report_paths, catalog_mismatch_path);
  Check(catalog_mismatch.at("comparison") == "baseline_mismatch" &&
            catalog_mismatch.at("baseline").at("id") ==
                "reviewed-catalog-v1",
        "catalog without an exact key fails closed");

  catalog["baselines"] = Json::array({baseline, baseline});
  const auto duplicate_catalog_path = bundle_root / "catalog-duplicate.json";
  WriteJsonExclusive(duplicate_catalog_path, catalog);
  bool duplicate_catalog_rejected = false;
  try {
    static_cast<void>(
        AggregatePerformanceReports(report_paths, duplicate_catalog_path));
  } catch (const std::invalid_argument&) {
    duplicate_catalog_rejected = true;
  }
  Check(duplicate_catalog_rejected,
        "catalog rejects duplicate exact baseline keys");

  baseline["key"]["container_digest"] =
      "sha256:" + std::string(64, 'e');
  const auto mismatch_path = bundle_root / "mismatch.json";
  WriteJsonExclusive(mismatch_path, baseline);
  Check(AggregatePerformanceReports(report_paths, mismatch_path)
                .at("comparison") == "baseline_mismatch",
        "container or fixture key mismatch fails closed");

  baseline["key"] = uncalibrated.at("key");
  baseline["metrics"][0]["expected"] = 1;
  const auto regression_path = bundle_root / "regression.json";
  WriteJsonExclusive(regression_path, baseline);
  Check(AggregatePerformanceReports(report_paths, regression_path)
                .at("comparison") == "fail",
        "metric above reviewed allowance is a regression");

  auto failed_sample = report;
  failed_sample["iteration"] = 5;
  failed_sample["run_id"] = "small-v1-open-failed-5";
  failed_sample["timing"]["wall_time_ns"] = 50;
  failed_sample["result"] = "fail";
  failed_sample["artifacts"]["correctness"] = "fail";
  failed_sample["failures"] =
      {{{"phase", "target"}, {"message", "deterministic failure"}}};
  const auto failed_path = bundle_root / "failed-5.json";
  WriteJsonExclusive(failed_path, failed_sample);
  auto failed_reports = report_paths;
  failed_reports.back() = failed_path;
  const auto failed_bundle = AggregatePerformanceReports(failed_reports);
  Check(failed_bundle.at("comparison") == "fail" &&
            failed_bundle.at("reports").size() == 5 &&
            !failed_bundle.at("failures").empty(),
        "failed repetition remains referenced and fails the whole bundle");

  std::vector<fs::path> sequential_paths;
  std::vector<fs::path> parallel_paths;
  std::vector<fs::path> sequential_owner_paths;
  std::vector<fs::path> parallel_owner_paths;
  std::vector<PairParityEvidence> parity;
  for (uint64_t iteration = 1; iteration <= 2; ++iteration) {
    auto sequential = report;
    sequential["scenario"] = "map-update-sequential";
    sequential["fixture"]["parallelism"] = "sequential";
    sequential["iteration"] = iteration;
    sequential["run_id"] = "pair-sequential-" + std::to_string(iteration);
    sequential["timing"]["wall_time_ns"] = 200 + iteration;
    sequential["points"]["map_output_points"] = 100;
    auto parallel = sequential;
    parallel["scenario"] = "map-update-parallel";
    parallel["fixture"]["parallelism"] = "parallel";
    parallel["fixture"]["manifest_sha256"] =
        "sha256:" + std::string(64, 'c');
    parallel["fixture"]["config_fingerprint"] =
        "sha256:" + std::string(64, 'd');
    parallel["run_id"] = "pair-parallel-" + std::to_string(iteration);
    parallel["timing"]["wall_time_ns"] = 100 + iteration;
    const auto sequential_path =
        bundle_root / ("pair-sequential-" + std::to_string(iteration) +
                       ".json");
    const auto parallel_path =
        bundle_root / ("pair-parallel-" + std::to_string(iteration) +
                       ".json");
    auto sequential_owner = sequential;
    sequential_owner["measurement_role"] = "owner";
    sequential_owner["run_id"] =
        "pair-sequential-owner-" + std::to_string(iteration);
    auto parallel_owner = parallel;
    parallel_owner["measurement_role"] = "owner";
    parallel_owner["run_id"] =
        "pair-parallel-owner-" + std::to_string(iteration);
    const auto sequential_owner_path =
        bundle_root / ("pair-sequential-owner-" +
                       std::to_string(iteration) + ".json");
    const auto parallel_owner_path =
        bundle_root / ("pair-parallel-owner-" +
                       std::to_string(iteration) + ".json");
    WriteJsonExclusive(sequential_path, sequential);
    WriteJsonExclusive(parallel_path, parallel);
    WriteJsonExclusive(sequential_owner_path, sequential_owner);
    WriteJsonExclusive(parallel_owner_path, parallel_owner);
    sequential_paths.push_back(sequential_path);
    parallel_paths.push_back(parallel_path);
    sequential_owner_paths.push_back(sequential_owner_path);
    parallel_owner_paths.push_back(parallel_owner_path);
    const auto parity_path =
        bundle_root / ("parity-" + std::to_string(iteration) + ".log");
    std::ofstream parity_output(parity_path);
    parity_output << "overall=PASS\n";
    parity_output.close();
    parity.push_back({iteration, true, parity_path});
  }
  const auto sequential_bundle =
      AggregatePerformanceReports(sequential_paths);
  const auto parallel_bundle = AggregatePerformanceReports(parallel_paths);
  const auto sequential_owner_bundle =
      AggregatePerformanceReports(sequential_owner_paths);
  const auto parallel_owner_bundle =
      AggregatePerformanceReports(parallel_owner_paths);
  const auto sequential_bundle_path = bundle_root / "sequential-bundle.json";
  const auto parallel_bundle_path = bundle_root / "parallel-bundle.json";
  const auto sequential_owner_bundle_path =
      bundle_root / "sequential-owner-bundle.json";
  const auto parallel_owner_bundle_path =
      bundle_root / "parallel-owner-bundle.json";
  WriteJsonExclusive(sequential_bundle_path, sequential_bundle);
  WriteJsonExclusive(parallel_bundle_path, parallel_bundle);
  WriteJsonExclusive(sequential_owner_bundle_path, sequential_owner_bundle);
  WriteJsonExclusive(parallel_owner_bundle_path, parallel_owner_bundle);
  const auto pair = BuildMapUpdatePairReport(
      sequential_bundle_path, parallel_bundle_path,
      sequential_owner_bundle_path, parallel_owner_bundle_path, parity);
  Check(ValidateMapUpdatePairReport(pair).Ok() &&
            pair.at("result") == "pass" &&
            pair.at("comparison") == "uncalibrated" &&
            pair.at("pairs").size() == 2 &&
            pair.at("pairs").at(0).at("timing").at("wall_speedup") > 1.0,
        "paired map-update report joins parity, timing and owner evidence");

  const auto make_pair_baseline = [](const Json& bundle,
                                     const std::string& id) {
    const auto metric = std::find_if(
        bundle.at("metrics").begin(), bundle.at("metrics").end(),
        [](const Json& candidate) {
          return candidate.at("name") == "timing.wall_time_ns";
        });
    return Json{
        {"schema_version", 1},
        {"baseline_id", id},
        {"key", bundle.at("key")},
        {"source_bundle_sha256", "sha256:" + std::string(64, 'f')},
        {"metrics",
         {{{"name", "timing.wall_time_ns"},
           {"statistic", "median"},
           {"expected", metric->at("summary").at("median")},
           {"relative_allowance", 0.0},
           {"absolute_allowance", 0}}}}};
  };
  Json pair_catalog = {
      {"schema_version", 1},
      {"catalog_id", "reviewed-pair-catalog-v1"},
      {"baselines",
       {make_pair_baseline(sequential_bundle, "sequential-public-v1"),
        make_pair_baseline(parallel_bundle, "parallel-public-v1"),
        make_pair_baseline(sequential_owner_bundle, "sequential-owner-v1"),
        make_pair_baseline(parallel_owner_bundle, "parallel-owner-v1")}}};
  const auto pair_catalog_path = bundle_root / "pair-catalog.json";
  WriteJsonExclusive(pair_catalog_path, pair_catalog);
  const auto calibrated_sequential_bundle =
      AggregatePerformanceReports(sequential_paths, pair_catalog_path);
  const auto calibrated_parallel_bundle =
      AggregatePerformanceReports(parallel_paths, pair_catalog_path);
  const auto calibrated_sequential_owner_bundle =
      AggregatePerformanceReports(sequential_owner_paths, pair_catalog_path);
  const auto calibrated_parallel_owner_bundle =
      AggregatePerformanceReports(parallel_owner_paths, pair_catalog_path);
  const auto calibrated_sequential_path =
      bundle_root / "calibrated-sequential-bundle.json";
  const auto calibrated_parallel_path =
      bundle_root / "calibrated-parallel-bundle.json";
  const auto calibrated_sequential_owner_path =
      bundle_root / "calibrated-sequential-owner-bundle.json";
  const auto calibrated_parallel_owner_path =
      bundle_root / "calibrated-parallel-owner-bundle.json";
  WriteJsonExclusive(calibrated_sequential_path,
                     calibrated_sequential_bundle);
  WriteJsonExclusive(calibrated_parallel_path, calibrated_parallel_bundle);
  WriteJsonExclusive(calibrated_sequential_owner_path,
                     calibrated_sequential_owner_bundle);
  WriteJsonExclusive(calibrated_parallel_owner_path,
                     calibrated_parallel_owner_bundle);
  Check(BuildMapUpdatePairReport(
            calibrated_sequential_path, calibrated_parallel_path,
            calibrated_sequential_owner_path,
            calibrated_parallel_owner_path, parity)
            .at("comparison") == "pass",
        "paired report propagates complete public and owner calibration");
  parity.back().passed = false;
  Check(BuildMapUpdatePairReport(
            sequential_bundle_path, parallel_bundle_path,
            sequential_owner_bundle_path, parallel_owner_bundle_path, parity)
            .at("result") == "fail",
        "semantic parity failure fails the paired report");

  fs::remove_all(bundle_root, error);
  Check(!error, "bundle contract fixture cleanup succeeds");
  std::cout << "benchmark report contract tests passed\n";
}
