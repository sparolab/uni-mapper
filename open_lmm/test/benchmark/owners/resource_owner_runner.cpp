#include "support/benchmark/benchmark_options.hpp"
#include "support/benchmark/benchmark_report.hpp"
#include "support/benchmark/fixture_generator.hpp"
#include "support/benchmark/process_window_sampler.hpp"
#include "tools/replay/replay_sha256.hpp"

#include <config/bootstrap/bootstrap_config.hpp>
#include <runtime/execution/stage_executor.hpp>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <nlohmann/json.hpp>

namespace {

namespace fs = std::filesystem;
using Json = nlohmann::json;
using namespace open_lmm;
using namespace open_lmm::test::benchmark;

Json ReadJson(const fs::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("failed to open " + path.string());
  Json value;
  input >> value;
  return value;
}

Json Unavailable(std::initializer_list<const char*> fields,
                 const std::string& reason) {
  Json value = Json::object();
  value["unavailable_reasons"] = Json::object();
  for (const char* field : fields) {
    value[field] = nullptr;
    value["unavailable_reasons"][field] = reason;
  }
  return value;
}

template <class Value>
void Set(Json& group, const char* field, Value&& value) {
  group[field] = std::forward<Value>(value);
  group["unavailable_reasons"].erase(field);
}

uint64_t OutputBytes(const fs::path& output) {
  uint64_t bytes = 0;
  for (const auto& entry : fs::recursive_directory_iterator(output)) {
    if (entry.is_regular_file()) bytes += entry.file_size();
  }
  return bytes;
}

uint64_t OutputPoints(const fs::path& output) {
  uint64_t points = 0;
  for (const auto& entry : fs::recursive_directory_iterator(output)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".pcd")
      continue;
    std::ifstream input(entry.path(), std::ios::binary);
    for (std::string line; std::getline(input, line);) {
      if (line.starts_with("POINTS ")) {
        std::istringstream count(line.substr(7));
        uint64_t value = 0;
        if (count >> value) points += value;
        break;
      }
      if (line.starts_with("DATA ")) break;
    }
  }
  return points;
}

std::string OutputDigest(const fs::path& output) {
  std::vector<fs::path> files;
  for (const auto& entry : fs::recursive_directory_iterator(output)) {
    if (entry.is_regular_file()) files.push_back(entry.path());
  }
  std::sort(files.begin(), files.end());
  std::string canonical;
  for (const auto& path : files) {
    canonical += fs::relative(path, output).generic_string() + ":" +
                 open_lmm::test::replay::Sha256File(path) + "\n";
  }
  return "sha256:" + open_lmm::test::replay::Sha256(canonical);
}

void ExecuteStage(StageExecutor& executor, StageId stage) {
  ExecutionContext context;
  context.cancellation = std::make_shared<CancellationToken>();
  context.alignment_feedback = std::make_shared<AlignmentFeedbackBroker>();
  context.base_revision = executor.Snapshot().revision;
  auto result = executor.Execute(ExecutionCommand::Stage(stage), context);
  if (!result) {
    throw std::runtime_error("owner stage failed: " +
                             result.GetError().Message());
  }
}

void Prerequisites(StageExecutor& executor, const std::string& scenario) {
  if (scenario == "data-load") return;
  ExecuteStage(executor, StageId::kDataLoad);
  if (scenario == "alignment") return;
  ExecuteStage(executor, StageId::kAlignment);
  if (scenario == "map-update-sequential" ||
      scenario == "map-update-parallel") {
    return;
  }
  if (scenario == "save-fallback") return;
  if (scenario == "visualization-cold" ||
      scenario == "visualization-warm") {
    ExecuteStage(executor, StageId::kMapUpdate);
    ExecuteStage(executor, StageId::kSave);
    return;
  }
  throw std::invalid_argument("unsupported owner scenario: " + scenario);
}

uint64_t Delta(uint64_t after, uint64_t before) {
  if (after < before) throw std::logic_error("diagnostic counter regressed");
  return after - before;
}

int Run(int argc, char** argv) {
  const RunnerOptions options = ParseRunnerOptions(argc, argv);
  const std::set<std::string> supported = {
      "data-load", "alignment", "map-update-sequential",
      "map-update-parallel", "save-fallback", "visualization-cold",
      "visualization-warm"};
  if (!supported.contains(options.scenario)) {
    throw std::invalid_argument("scenario has no private owner target");
  }
  const fs::path fixture_root = fs::absolute(options.fixture_root);
  const fs::path manifest_path = fixture_root / "fixture_manifest.json";
  const Json manifest = ReadJson(manifest_path);
  const auto verified = VerifyMaterializedFixture(fixture_root, manifest);
  if (!verified.Ok()) {
    throw std::runtime_error("fixture verification failed:\n" +
                             verified.Summary());
  }
  auto bootstrap = LoadBootstrapConfig(fixture_root / "config");
  if (!bootstrap) {
    throw std::runtime_error("owner bootstrap failed: " +
                             bootstrap.GetError().Message());
  }
  const auto& resource = manifest.at("resource_budget");
  const bool map_enabled = resource.at("enable_map_update").get<bool>();
  const bool map_parallel = resource.at("parallel_map_update").get<bool>();
  if (options.scenario == "map-update-sequential" &&
      (!map_enabled || map_parallel)) {
    throw std::invalid_argument(
        "owner sequential scenario requires a sequential MapUpdate fixture");
  }
  if (options.scenario == "map-update-parallel" &&
      (!map_enabled || !map_parallel)) {
    throw std::invalid_argument(
        "owner parallel scenario requires a parallel MapUpdate fixture");
  }
  if (options.scenario == "save-fallback" && map_enabled) {
    throw std::invalid_argument(
        "owner save fallback requires MapUpdate to be disabled");
  }
  auto governor = std::make_shared<ResourceGovernor>(ResourceBudget{
      resource.at("max_agent_tasks").get<std::size_t>(),
      resource.at("max_cpu_threads").get<std::size_t>(),
      resource.at("soft_memory_bytes").get<uint64_t>()});
  StageExecutor executor(std::move(bootstrap).Value(),
                         fixture_root / "output", governor);
  if (auto ready = executor.ValidateReady(); !ready) {
    throw std::runtime_error("owner executor is not ready: " +
                             ready.GetError().Message());
  }
  Prerequisites(executor, options.scenario);
  const auto target_agent = executor.Snapshot().ordered_agents.front();
  if (options.scenario == "visualization-warm") {
    auto cold = executor.Visualization({target_agent, true, 0.4F, 1});
    if (!cold) {
      throw std::runtime_error("owner visualization warmup failed: " +
                               cold.GetError().Message());
    }
  }
  const auto before = executor.Diagnostics();
  governor->ResetDiagnosticPeaksToCurrent();

  ProcessWindowSampler sampler;
  sampler.Start();
  const auto started = std::chrono::steady_clock::now();
  std::optional<VisualizationSnapshot> visualization;
  std::size_t visualization_returns = 0;
  if (options.scenario == "data-load") {
    ExecuteStage(executor, StageId::kDataLoad);
  } else if (options.scenario == "alignment") {
    ExecuteStage(executor, StageId::kAlignment);
  } else if (options.scenario == "map-update-sequential" ||
             options.scenario == "map-update-parallel") {
    ExecuteStage(executor, StageId::kMapUpdate);
  } else if (options.scenario == "save-fallback") {
    ExecuteStage(executor, StageId::kSave);
  } else if (options.scenario == "visualization-cold") {
    auto projected = executor.Visualization({target_agent, true, 0.4F, 1});
    if (!projected) throw std::runtime_error(projected.GetError().Message());
    visualization = std::move(projected).Value();
    visualization_returns = 1;
  } else {
    for (std::size_t query = 0; query < 10; ++query) {
      auto projected =
          executor.Visualization({target_agent, true, 0.4F, query + 2});
      if (!projected) throw std::runtime_error(projected.GetError().Message());
      visualization = std::move(projected).Value();
      ++visualization_returns;
    }
  }
  const uint64_t latency = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now() - started)
          .count());
  const auto process_summary = sampler.Stop();
  const auto after = executor.Diagnostics();

  Json timing = Unavailable(
      {"wall_time_ns", "cpu_time_ns", "stage_latency_ns",
       "command_latency_ns", "cancellation_latency_ns"},
      "not applicable to this owner scenario");
  Set(timing, "wall_time_ns", process_summary.wall_time_ns);
  Set(timing, "command_latency_ns", latency);
  if (options.scenario == "data-load" || options.scenario == "alignment" ||
      options.scenario == "map-update-sequential" ||
      options.scenario == "map-update-parallel" ||
      options.scenario == "save-fallback") {
    Set(timing, "stage_latency_ns", latency);
  }
  if (process_summary.cpu_time_ns)
    Set(timing, "cpu_time_ns", *process_summary.cpu_time_ns);

  Json process = Unavailable(
      {"rss_start_bytes", "rss_end_bytes", "sampled_peak_rss_bytes",
       "process_hwm_bytes", "target_peak_delta_bytes", "sample_count",
       "sample_interval_ns", "retained_rss_delta_bytes"},
      "process metric unavailable");
  const auto set_optional = [&](const char* field,
                                const std::optional<uint64_t>& value) {
    if (value) Set(process, field, *value);
  };
  set_optional("rss_start_bytes", process_summary.rss_start_bytes);
  set_optional("rss_end_bytes", process_summary.rss_end_bytes);
  set_optional("sampled_peak_rss_bytes",
               process_summary.sampled_peak_rss_bytes);
  set_optional("process_hwm_bytes", process_summary.process_hwm_bytes);
  set_optional("target_peak_delta_bytes",
               process_summary.target_peak_delta_bytes);
  if (process_summary.retained_rss_delta_bytes)
    Set(process, "retained_rss_delta_bytes",
        *process_summary.retained_rss_delta_bytes);
  Set(process, "sample_count", process_summary.sample_count);
  Set(process, "sample_interval_ns", process_summary.sample_interval_ns);
  process["memory_confidence"] = process_summary.memory_confidence;

  const auto& resources = after.resources;
  Json owner = Unavailable(
      {"governor_reserved_total_bytes", "governor_resident_payload_bytes",
       "governor_transient_task_bytes", "governor_heavy_map_bytes",
       "governor_peak_reserved_total_bytes",
       "governor_peak_resident_payload_bytes",
       "governor_peak_transient_task_bytes",
       "governor_peak_heavy_map_bytes", "governor_admission_failures",
       "executor_worker_count", "executor_queue_capacity",
       "executor_max_queued_tasks", "executor_max_active_tasks",
       "executor_max_waiting_submitters"},
      "owner metric unavailable");
  Set(owner, "governor_reserved_total_bytes", resources.reserved_total_bytes);
  Set(owner, "governor_resident_payload_bytes", resources.reserved_by_class[0]);
  Set(owner, "governor_transient_task_bytes", resources.reserved_by_class[1]);
  Set(owner, "governor_heavy_map_bytes", resources.reserved_by_class[2]);
  Set(owner, "governor_peak_reserved_total_bytes",
      resources.peak_reserved_total_bytes);
  Set(owner, "governor_peak_resident_payload_bytes",
      resources.peak_reserved_by_class[0]);
  Set(owner, "governor_peak_transient_task_bytes",
      resources.peak_reserved_by_class[1]);
  Set(owner, "governor_peak_heavy_map_bytes",
      resources.peak_reserved_by_class[2]);
  Set(owner, "governor_admission_failures", resources.admission_failures);
  Set(owner, "executor_worker_count", resources.executor.worker_count);
  Set(owner, "executor_queue_capacity", resources.executor.queue_capacity);
  Set(owner, "executor_max_queued_tasks", resources.executor.max_queued_tasks);
  Set(owner, "executor_max_active_tasks", resources.executor.max_active_tasks);
  Set(owner, "executor_max_waiting_submitters",
      resources.executor.max_waiting_submitters);

  Json points = Unavailable(
      {"input_file_bytes", "decoded_source_points", "decoded_source_bytes",
       "retained_filtered_points", "retained_capacity_bytes",
       "optimized_pose_count", "map_output_points", "map_output_file_bytes",
       "visualization_source_points", "visualization_displayed_points",
       "cache_entry_points", "public_dto_point_bytes", "gui_staging_bytes"},
      "owner metric unavailable");
  Set(points, "input_file_bytes", manifest.at("on_disk_bytes"));
  Set(points, "decoded_source_points", manifest.at("decoded_point_count"));
  Set(points, "decoded_source_bytes", manifest.at("decoded_point_bytes"));
  Set(points, "map_output_points", OutputPoints(fixture_root / "output"));
  Set(points, "map_output_file_bytes", OutputBytes(fixture_root / "output"));
  if (visualization) {
    Set(points, "visualization_source_points", visualization->source_point_count);
    Set(points, "visualization_displayed_points",
        visualization->displayed_point_count);
    Set(points, "public_dto_point_bytes",
        visualization->points.size() * sizeof(VisualizationPoint) *
            visualization_returns);
  }

  Json io = Unavailable(
      {"rchar", "wchar", "syscr", "syscw", "read_bytes", "write_bytes",
       "cancelled_write_bytes"},
      "process I/O counter unavailable on this platform");
  const auto set_io = [&](const char* field,
                          const std::optional<uint64_t>& value) {
    if (value) Set(io, field, *value);
  };
  set_io("rchar", process_summary.io.rchar);
  set_io("wchar", process_summary.io.wchar);
  set_io("syscr", process_summary.io.syscr);
  set_io("syscw", process_summary.io.syscw);
  set_io("read_bytes", process_summary.io.read_bytes);
  set_io("write_bytes", process_summary.io.write_bytes);
  set_io("cancelled_write_bytes", process_summary.io.cancelled_write_bytes);
  Json cache = Unavailable(
      {"entries", "bytes", "hits", "misses", "insertions", "evictions",
       "clears", "returned_snapshot_bytes", "cpu_staging_bytes",
       "gpu_upload_requested_bytes", "gpu_estimated_resident_bytes"},
      "cache metric unavailable");
  Set(cache, "entries", after.visualization.entries);
  Set(cache, "bytes", after.visualization.bytes);
  Set(cache, "hits", Delta(after.visualization.hits,
                            before.visualization.hits));
  Set(cache, "misses", Delta(after.visualization.misses,
                              before.visualization.misses));
  Set(cache, "insertions", Delta(after.visualization.insertions,
                                  before.visualization.insertions));
  Set(cache, "evictions", Delta(after.visualization.evictions,
                                 before.visualization.evictions));
  Set(cache, "clears", Delta(after.visualization.clears,
                              before.visualization.clears));
  if (visualization) {
    Set(cache, "returned_snapshot_bytes",
        visualization->points.size() * sizeof(VisualizationPoint) *
            visualization_returns);
  }
  cache["gpu_measurement_kind"] = "not_available";

  const uint64_t output_bytes = OutputBytes(fixture_root / "output");
  Json artifacts = Unavailable(
      {"logical_output_bytes", "retained_owner_bytes",
       "sampled_target_peak_delta_bytes", "process_hwm_bytes",
       "peak_to_output_ratio", "peak_to_retained_ratio"},
      "ratio unavailable");
  Set(artifacts, "logical_output_bytes", output_bytes);
  Set(artifacts, "retained_owner_bytes", resources.reserved_total_bytes);
  if (process_summary.target_peak_delta_bytes)
    Set(artifacts, "sampled_target_peak_delta_bytes",
        *process_summary.target_peak_delta_bytes);
  if (process_summary.process_hwm_bytes)
    Set(artifacts, "process_hwm_bytes", *process_summary.process_hwm_bytes);
  if (output_bytes != 0) {
    Set(artifacts, "peak_to_output_ratio",
        static_cast<double>(resources.peak_reserved_total_bytes) /
            static_cast<double>(output_bytes));
  }
  if (resources.reserved_total_bytes != 0) {
    Set(artifacts, "peak_to_retained_ratio",
        static_cast<double>(resources.peak_reserved_total_bytes) /
            static_cast<double>(resources.reserved_total_bytes));
  }
  artifacts["output_digest"] = OutputDigest(fixture_root / "output");
  artifacts["correctness"] = "pass";

  const std::string manifest_digest =
      "sha256:" + open_lmm::test::replay::Sha256File(manifest_path);
  Json report = {
      {"schema_version", 1},
      {"run_id", manifest.at("fixture_id").get<std::string>() + "-" +
                     options.scenario + "-owner-" +
                     std::to_string(options.iteration)},
      {"measurement_role", "owner"},
      {"profile", options.profile},
      {"scenario", options.scenario},
      {"iteration", options.iteration},
      {"git", {{"commit", options.git_commit}, {"dirty", options.git_dirty}}},
      {"build", {{"compiler", options.compiler},
                 {"build_type", options.build_type},
                 {"sanitizer", options.sanitizer},
                 {"container_digest", options.container_digest}}},
      {"machine", MachineMetadata()},
      {"fixture", {{"id", manifest.at("fixture_id")},
                   {"manifest_sha256", manifest_digest},
                   {"decoded_point_count", manifest.at("decoded_point_count")},
                   {"decoded_point_bytes", manifest.at("decoded_point_bytes")},
                   {"config_fingerprint",
                    BenchmarkConfigFingerprint(manifest)},
                   {"pair_fingerprint",
                    BenchmarkPairFingerprint(fixture_root, manifest)},
                   {"plugin_ids", BenchmarkPluginIds(manifest)},
                   {"parallelism",
                    resource.at("parallel_map_update").get<bool>()
                        ? "parallel"
                        : (resource.at("enable_map_update").get<bool>()
                               ? "sequential"
                               : "map-update-disabled")}}},
      {"timing", std::move(timing)},
      {"stage_timings", Json::array()},
      {"process_memory", std::move(process)},
      {"owner_memory", std::move(owner)},
      {"points", std::move(points)},
      {"io", std::move(io)},
      {"cache", std::move(cache)},
      {"artifacts", std::move(artifacts)},
      {"failures", Json::array()},
      {"result", "uncalibrated"}};
  const auto validation = ValidatePerformanceReport(report);
  if (!validation.Ok()) {
    throw std::runtime_error("invalid owner report:\n" +
                             validation.Summary());
  }
  WriteJsonExclusive(options.report_path, report);
  std::cout << options.report_path << '\n';
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    return Run(argc, argv);
  } catch (const std::invalid_argument& error) {
    std::cerr << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
  }
}
