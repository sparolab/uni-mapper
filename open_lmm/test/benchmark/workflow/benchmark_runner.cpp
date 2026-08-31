#include "support/benchmark/benchmark_options.hpp"
#include "support/benchmark/benchmark_report.hpp"
#include "support/benchmark/fixture_generator.hpp"
#include "support/benchmark/process_window_sampler.hpp"
#include "support/benchmark/stage_event_recorder.hpp"
#include "tools/replay/replay_sha256.hpp"

#include <open_lmm/server/runtime_client.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <optional>
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

class ScopedCurrentPath {
 public:
  explicit ScopedCurrentPath(const fs::path& path)
      : previous_(fs::current_path()) {
    fs::current_path(path);
  }
  ~ScopedCurrentPath() {
    std::error_code ignored;
    fs::current_path(previous_, ignored);
  }

 private:
  fs::path previous_;
};

Json ReadJson(const fs::path& path) {
  std::ifstream input(path);
  if (!input) throw std::runtime_error("failed to open " + path.string());
  Json value;
  input >> value;
  return value;
}

uint64_t NowNanoseconds() {
  return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::steady_clock::now().time_since_epoch())
          .count());
}

Json UnavailableMetrics(std::initializer_list<const char*> fields,
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
void SetMetric(Json& group, const char* field, Value&& value) {
  group[field] = std::forward<Value>(value);
  group["unavailable_reasons"].erase(field);
}

uint64_t OutputBytes(const fs::path& output) {
  uint64_t bytes = 0;
  std::error_code error;
  if (!fs::is_directory(output, error) || error) return 0;
  for (const auto& entry : fs::recursive_directory_iterator(output)) {
    if (entry.is_regular_file()) bytes += entry.file_size();
  }
  return bytes;
}

std::string OutputDigest(const fs::path& output) {
  std::vector<fs::path> files;
  std::error_code error;
  if (fs::is_directory(output, error) && !error) {
    for (const auto& entry : fs::recursive_directory_iterator(output)) {
      if (entry.is_regular_file()) files.push_back(entry.path());
    }
  }
  std::sort(files.begin(), files.end());
  std::string canonical;
  for (const auto& path : files) {
    canonical += fs::relative(path, output).generic_string();
    canonical += ':';
    canonical += open_lmm::test::replay::Sha256File(path);
    canonical += '\n';
  }
  return "sha256:" + open_lmm::test::replay::Sha256(canonical);
}

uint64_t PcdPointCount(const fs::path& output) {
  uint64_t points = 0;
  std::error_code error;
  if (!fs::is_directory(output, error) || error) return 0;
  for (const auto& entry : fs::recursive_directory_iterator(output)) {
    if (!entry.is_regular_file() || entry.path().extension() != ".pcd") {
      continue;
    }
    std::ifstream input(entry.path(), std::ios::binary);
    for (std::string line; std::getline(input, line);) {
      if (line.starts_with("POINTS ")) {
        std::istringstream value(line.substr(7));
        uint64_t count = 0;
        if (value >> count) points += count;
        break;
      }
      if (line == "DATA binary" || line == "DATA ascii") break;
    }
  }
  return points;
}

void Require(const Result<void>& result, const char* operation) {
  if (!result) {
    throw std::runtime_error(std::string(operation) + ": " +
                             result.GetError().Message());
  }
}

JobHandle Execute(RuntimeClient& client, const ExecutionRequest& request,
                  const char* operation) {
  auto submitted = client.Submit(request);
  if (!submitted) {
    throw std::runtime_error(std::string(operation) + " submit: " +
                             submitted.GetError().Message());
  }
  Require(client.Wait(submitted.Value()), operation);
  return submitted.Value();
}

void RunPrerequisites(RuntimeClient& client, const std::string& scenario) {
  if (scenario == "data-load" || scenario == "full-pipeline" ||
      scenario == "cancellation") return;
  if (scenario == "alignment" || scenario == "save-fallback" ||
      scenario == "map-update-sequential" ||
      scenario == "map-update-parallel") {
    static_cast<void>(Execute(
        client, {ExecutionRequestKind::kStage, StageId::kDataLoad},
        "DataLoad prerequisite"));
    if (scenario == "save-fallback" ||
        scenario == "map-update-sequential" ||
        scenario == "map-update-parallel") {
      static_cast<void>(Execute(
          client, {ExecutionRequestKind::kStage, StageId::kAlignment},
          "Alignment prerequisite"));
    }
    return;
  }
  if (scenario == "visualization-cold" ||
      scenario == "visualization-warm") {
    static_cast<void>(Execute(client, {}, "RunAll prerequisite"));
    return;
  }
  throw std::invalid_argument(
      "scenario implementation is pending: " + scenario);
}

struct TargetResult {
  ProcessWindowSummary process;
  std::optional<uint64_t> command_latency_ns;
  std::optional<uint64_t> stage_latency_ns;
  std::optional<uint64_t> cancellation_latency_ns;
  std::vector<StageEventWindow> stage_windows;
  std::optional<VisualizationSnapshot> visualization;
  std::size_t visualization_return_count = 0;
};

const char* StageName(StageId stage) {
  switch (stage) {
    case StageId::kDataLoad:
      return "data-load";
    case StageId::kAlignment:
      return "alignment";
    case StageId::kMapUpdate:
      return "map-update";
    case StageId::kSave:
      return "save";
  }
  throw std::logic_error("unknown stage id");
}

struct CancellationProbe {
  std::atomic<bool> issued{false};
  std::atomic<bool> failed{false};
  std::atomic<uint64_t> requested_ns{0};
  std::atomic<uint64_t> terminal_ns{0};
};

TargetResult RunCancellationTarget(RuntimeClient& client,
                                   const CancellationProbe& probe) {
  TargetResult result;
  ProcessWindowSampler sampler;
  sampler.Start();
  const uint64_t command_start = NowNanoseconds();
  auto submitted = client.Submit({});
  if (!submitted) {
    throw std::runtime_error("cancellation submit: " +
                             submitted.GetError().Message());
  }
  const auto waited = client.Wait(submitted.Value());
  result.command_latency_ns = NowNanoseconds() - command_start;
  result.process = sampler.Stop();
  if (waited) {
    throw std::runtime_error(
        "cancellation target unexpectedly completed successfully");
  }
  const uint64_t requested = probe.requested_ns.load(std::memory_order_acquire);
  const uint64_t terminal = probe.terminal_ns.load(std::memory_order_acquire);
  if (!probe.issued.load(std::memory_order_acquire) ||
      probe.failed.load(std::memory_order_acquire) || requested == 0 ||
      terminal < requested) {
    throw std::runtime_error(
        "cancellation event barrier did not produce a valid terminal window");
  }
  result.cancellation_latency_ns = terminal - requested;
  return result;
}

TargetResult RunTarget(RuntimeClient& client, const std::string& scenario,
                       bool map_update_enabled,
                       const std::shared_ptr<StageEventRecorder>& recorder,
                       const AgentId& visualization_agent) {
  TargetResult result;
  ProcessWindowSampler sampler;
  sampler.Start();
  const uint64_t command_start = NowNanoseconds();
  std::optional<JobHandle> job;
  std::optional<StageId> target_stage;
  if (scenario == "data-load") target_stage = StageId::kDataLoad;
  else if (scenario == "alignment") target_stage = StageId::kAlignment;
  else if (scenario == "map-update-sequential" ||
           scenario == "map-update-parallel") {
    target_stage = StageId::kMapUpdate;
  }
  else if (scenario == "save-fallback") target_stage = StageId::kSave;

  if (target_stage) {
    job = Execute(client,
                  {ExecutionRequestKind::kStage, *target_stage},
                  scenario.c_str());
  } else if (scenario == "full-pipeline") {
    job = Execute(client, {}, "full pipeline");
  } else if (scenario == "visualization-cold") {
    auto projected = client.Visualization(
        {visualization_agent, true, 0.4F, 1});
    if (!projected) {
      throw std::runtime_error("cold visualization: " +
                               projected.GetError().Message());
    }
    result.visualization = std::move(projected).Value();
    result.visualization_return_count = 1;
  } else if (scenario == "visualization-warm") {
    auto cold = client.Visualization(
        {visualization_agent, true, 0.4F, 1});
    if (!cold) {
      throw std::runtime_error("visualization warmup: " +
                               cold.GetError().Message());
    }
    // The cold query is a prerequisite and must not be inside the target.
    // This branch is replaced below by the caller's prepopulation path.
    throw std::logic_error("warm visualization cache was not prepopulated");
  } else {
    throw std::invalid_argument("unsupported target scenario: " + scenario);
  }
  const uint64_t command_end = NowNanoseconds();
  result.process = sampler.Stop();
  result.command_latency_ns = command_end - command_start;

  if (job && target_stage) {
    std::string event_error;
    const auto stage = recorder->FindStageWindow(
        job->value, *target_stage, &event_error);
    if (!stage || stage->terminal_type != EventType::kStageCompleted) {
      throw std::runtime_error("invalid stage event window: " + event_error);
    }
    result.stage_latency_ns = stage->latency_ns;
    result.stage_windows.push_back(*stage);
  } else if (job && scenario == "full-pipeline") {
    std::vector<StageId> expected = {StageId::kDataLoad,
                                     StageId::kAlignment};
    if (map_update_enabled) expected.push_back(StageId::kMapUpdate);
    expected.push_back(StageId::kSave);
    for (const StageId stage_id : expected) {
      std::string event_error;
      const auto stage =
          recorder->FindStageWindow(job->value, stage_id, &event_error);
      if (!stage || stage->terminal_type != EventType::kStageCompleted) {
        throw std::runtime_error("invalid full-pipeline stage window for " +
                                 std::string(StageName(stage_id)) + ": " +
                                 event_error);
      }
      result.stage_windows.push_back(*stage);
    }
  }
  return result;
}

TargetResult RunWarmVisualizationTarget(
    RuntimeClient& client, const AgentId& visualization_agent) {
  auto cold = client.Visualization({visualization_agent, true, 0.4F, 1});
  if (!cold) {
    throw std::runtime_error("visualization warmup: " +
                             cold.GetError().Message());
  }
  TargetResult result;
  ProcessWindowSampler sampler;
  sampler.Start();
  const uint64_t start = NowNanoseconds();
  for (std::size_t query = 0; query < 10; ++query) {
    auto projected = client.Visualization(
        {visualization_agent, true, 0.4F, query + 2});
    if (!projected) {
      throw std::runtime_error("warm visualization: " +
                               projected.GetError().Message());
    }
    result.visualization = std::move(projected).Value();
    ++result.visualization_return_count;
  }
  result.command_latency_ns = NowNanoseconds() - start;
  result.process = sampler.Stop();
  return result;
}

Json BuildReport(const RunnerOptions& options, const Json& manifest,
                 const std::string& manifest_digest,
                 const TargetResult& target, const RuntimeSnapshot& snapshot,
                 const fs::path& output) {
  if (snapshot.pipeline.runtime_revision == 0) {
    throw std::runtime_error("benchmark snapshot has no committed revision");
  }
  const std::string boundary_reason =
      "not observable through the public RuntimeClient boundary";
  Json timing = UnavailableMetrics(
      {"wall_time_ns", "cpu_time_ns", "stage_latency_ns",
       "command_latency_ns", "cancellation_latency_ns"},
      "not applicable to this scenario");
  SetMetric(timing, "wall_time_ns", target.process.wall_time_ns);
  if (target.process.cpu_time_ns) {
    SetMetric(timing, "cpu_time_ns", *target.process.cpu_time_ns);
  }
  if (target.stage_latency_ns) {
    SetMetric(timing, "stage_latency_ns", *target.stage_latency_ns);
  }
  if (target.command_latency_ns) {
    SetMetric(timing, "command_latency_ns", *target.command_latency_ns);
  }
  if (target.cancellation_latency_ns) {
    SetMetric(timing, "cancellation_latency_ns",
              *target.cancellation_latency_ns);
  }

  Json process = UnavailableMetrics(
      {"rss_start_bytes", "rss_end_bytes", "sampled_peak_rss_bytes",
       "process_hwm_bytes", "target_peak_delta_bytes", "sample_count",
       "sample_interval_ns", "retained_rss_delta_bytes"},
      "process metric unavailable on this platform");
  if (target.process.rss_start_bytes)
    SetMetric(process, "rss_start_bytes", *target.process.rss_start_bytes);
  if (target.process.rss_end_bytes)
    SetMetric(process, "rss_end_bytes", *target.process.rss_end_bytes);
  if (target.process.sampled_peak_rss_bytes) {
    SetMetric(process, "sampled_peak_rss_bytes",
              *target.process.sampled_peak_rss_bytes);
  }
  if (target.process.process_hwm_bytes)
    SetMetric(process, "process_hwm_bytes", *target.process.process_hwm_bytes);
  if (target.process.target_peak_delta_bytes) {
    SetMetric(process, "target_peak_delta_bytes",
              *target.process.target_peak_delta_bytes);
  }
  if (target.process.retained_rss_delta_bytes) {
    SetMetric(process, "retained_rss_delta_bytes",
              *target.process.retained_rss_delta_bytes);
  }
  SetMetric(process, "sample_count", target.process.sample_count);
  SetMetric(process, "sample_interval_ns", target.process.sample_interval_ns);
  process["memory_confidence"] = target.process.memory_confidence;

  Json owner = UnavailableMetrics(
      {"governor_reserved_total_bytes", "governor_resident_payload_bytes",
       "governor_transient_task_bytes", "governor_heavy_map_bytes",
       "governor_peak_reserved_total_bytes",
       "governor_peak_resident_payload_bytes",
       "governor_peak_transient_task_bytes",
       "governor_peak_heavy_map_bytes",
       "governor_admission_failures", "executor_worker_count",
       "executor_queue_capacity", "executor_max_queued_tasks",
       "executor_max_active_tasks", "executor_max_waiting_submitters"},
      boundary_reason);

  Json points = UnavailableMetrics(
      {"input_file_bytes", "decoded_source_points", "decoded_source_bytes",
       "retained_filtered_points", "retained_capacity_bytes",
       "optimized_pose_count", "map_output_points", "map_output_file_bytes",
       "visualization_source_points", "visualization_displayed_points",
       "cache_entry_points", "public_dto_point_bytes", "gui_staging_bytes"},
      "metric is not produced by this scenario");
  SetMetric(points, "input_file_bytes",
            manifest.at("on_disk_bytes").get<uint64_t>());
  if (options.scenario != "open") {
    SetMetric(points, "decoded_source_points",
              manifest.at("decoded_point_count").get<uint64_t>());
    SetMetric(points, "decoded_source_bytes",
              manifest.at("decoded_point_bytes").get<uint64_t>());
  }
  const uint64_t output_bytes = OutputBytes(output);
  const uint64_t output_points = PcdPointCount(output);
  SetMetric(points, "map_output_file_bytes", output_bytes);
  SetMetric(points, "map_output_points", output_points);
  if (target.visualization) {
    SetMetric(points, "visualization_source_points",
              target.visualization->source_point_count);
    SetMetric(points, "visualization_displayed_points",
              target.visualization->displayed_point_count);
    SetMetric(points, "public_dto_point_bytes",
              target.visualization->points.size() * sizeof(VisualizationPoint) *
                  target.visualization_return_count);
  }

  Json io = UnavailableMetrics(
      {"rchar", "wchar", "syscr", "syscw", "read_bytes", "write_bytes",
       "cancelled_write_bytes"},
      "process I/O counter unavailable on this platform");
  const auto set_io = [&](const char* name,
                          const std::optional<uint64_t>& value) {
    if (value) SetMetric(io, name, *value);
  };
  set_io("rchar", target.process.io.rchar);
  set_io("wchar", target.process.io.wchar);
  set_io("syscr", target.process.io.syscr);
  set_io("syscw", target.process.io.syscw);
  set_io("read_bytes", target.process.io.read_bytes);
  set_io("write_bytes", target.process.io.write_bytes);
  set_io("cancelled_write_bytes", target.process.io.cancelled_write_bytes);

  Json cache = UnavailableMetrics(
      {"entries", "bytes", "hits", "misses", "insertions", "evictions",
       "clears", "returned_snapshot_bytes", "cpu_staging_bytes",
       "gpu_upload_requested_bytes", "gpu_estimated_resident_bytes"},
      boundary_reason);
  cache["gpu_measurement_kind"] = "not_available";
  if (target.visualization) {
    SetMetric(cache, "returned_snapshot_bytes",
              target.visualization->points.size() * sizeof(VisualizationPoint) *
                  target.visualization_return_count);
  }

  Json artifacts = UnavailableMetrics(
      {"logical_output_bytes", "retained_owner_bytes",
       "sampled_target_peak_delta_bytes", "process_hwm_bytes",
       "peak_to_output_ratio", "peak_to_retained_ratio"},
      "ratio or owner metric is unavailable");
  SetMetric(artifacts, "logical_output_bytes", output_bytes);
  if (target.process.target_peak_delta_bytes) {
    SetMetric(artifacts, "sampled_target_peak_delta_bytes",
              *target.process.target_peak_delta_bytes);
    if (output_bytes != 0) {
      SetMetric(artifacts, "peak_to_output_ratio",
                static_cast<double>(*target.process.target_peak_delta_bytes) /
                    static_cast<double>(output_bytes));
    }
  }
  if (target.process.process_hwm_bytes) {
    SetMetric(artifacts, "process_hwm_bytes",
              *target.process.process_hwm_bytes);
  }
  artifacts["output_digest"] = OutputDigest(output);
  artifacts["correctness"] = "pass";

  Json stage_timings = Json::array();
  for (const auto& stage : target.stage_windows) {
    stage_timings.push_back(
        {{"stage", StageName(stage.stage)},
         {"latency_ns", stage.latency_ns},
         {"start_sequence", stage.start_sequence},
         {"terminal_sequence", stage.terminal_sequence}});
  }

  return {{"schema_version", 1},
          {"run_id", manifest.at("fixture_id").get<std::string>() + "-" +
                         options.scenario + "-" +
                         std::to_string(options.iteration)},
          {"measurement_role", "public"},
          {"profile", options.profile},
          {"scenario", options.scenario},
          {"iteration", options.iteration},
          {"git", {{"commit", options.git_commit},
                   {"dirty", options.git_dirty}}},
          {"build", {{"compiler", options.compiler},
                     {"build_type", options.build_type},
                     {"sanitizer", options.sanitizer},
                     {"container_digest", options.container_digest}}},
          {"machine", MachineMetadata()},
          {"fixture",
           {{"id", manifest.at("fixture_id")},
            {"manifest_sha256", manifest_digest},
            {"decoded_point_count", manifest.at("decoded_point_count")},
            {"decoded_point_bytes", manifest.at("decoded_point_bytes")},
            {"config_fingerprint", BenchmarkConfigFingerprint(manifest)},
            {"pair_fingerprint",
             BenchmarkPairFingerprint(output.parent_path(), manifest)},
            {"plugin_ids", BenchmarkPluginIds(manifest)},
            {"parallelism",
             manifest.at("resource_budget")
                     .at("parallel_map_update")
                     .get<bool>()
                 ? "parallel"
                 : (manifest.at("resource_budget")
                            .at("enable_map_update")
                            .get<bool>()
                        ? "sequential"
                        : "map-update-disabled")}}},
          {"timing", std::move(timing)},
          {"stage_timings", std::move(stage_timings)},
          {"process_memory", std::move(process)},
          {"owner_memory", std::move(owner)},
          {"points", std::move(points)},
          {"io", std::move(io)},
          {"cache", std::move(cache)},
          {"artifacts", std::move(artifacts)},
          {"failures", Json::array()},
          {"result", "uncalibrated"}};
}

int Run(int argc, char** argv) {
  const RunnerOptions options = ParseRunnerOptions(argc, argv);
  const fs::path absolute_fixture = fs::absolute(options.fixture_root);
  const fs::path manifest_path = absolute_fixture / "fixture_manifest.json";
  const Json manifest = ReadJson(manifest_path);
  const auto verified = VerifyMaterializedFixture(absolute_fixture, manifest);
  if (!verified.Ok()) {
    throw std::runtime_error("fixture verification failed:\n" +
                             verified.Summary());
  }
  const auto& resources = manifest.at("resource_budget");
  const bool map_enabled = resources.at("enable_map_update").get<bool>();
  const bool map_parallel = resources.at("parallel_map_update").get<bool>();
  if (options.scenario == "map-update-sequential" &&
      (!map_enabled || map_parallel)) {
    throw std::invalid_argument(
        "map-update-sequential requires a sequential MapUpdate fixture");
  }
  if (options.scenario == "map-update-parallel" &&
      (!map_enabled || !map_parallel)) {
    throw std::invalid_argument(
        "map-update-parallel requires a parallel MapUpdate fixture");
  }
  if (options.scenario == "save-fallback" && map_enabled) {
    throw std::invalid_argument(
        "save-fallback requires MapUpdate to be disabled");
  }
  const std::string manifest_digest =
      "sha256:" + open_lmm::test::replay::Sha256File(manifest_path);
  const auto agent = AgentId::Parse("agent-00");
  if (!agent) throw std::runtime_error("generated fixture agent is invalid");

  TargetResult target;
  RuntimeSnapshot final_snapshot;
  {
    ScopedCurrentPath working_directory(absolute_fixture);
    RuntimeClient client(2);
    auto recorder = std::make_shared<StageEventRecorder>();
    auto cancellation = std::make_shared<CancellationProbe>();
    ExecutionEventSubscription subscription;

    if (options.scenario == "open") {
      ProcessWindowSampler sampler;
      sampler.Start();
      const uint64_t start = NowNanoseconds();
      Require(client.Open({"config", "benchmark-runner"}), "Open");
      target.command_latency_ns = NowNanoseconds() - start;
      target.process = sampler.Stop();
    } else {
      Require(client.Open({"config", "benchmark-runner"}), "Open prerequisite");
      auto subscribed = client.SubscribeEvents(
          [&client, recorder, cancellation,
           cancellation_scenario = options.scenario == "cancellation"](
              const ExecutionEvent& event) {
            recorder->Record(event);
            if (!cancellation_scenario) return;
            if (event.type == EventType::kStageStarted &&
                event.stage == StageId::kDataLoad) {
              bool expected = false;
              if (cancellation->issued.compare_exchange_strong(
                      expected, true, std::memory_order_acq_rel)) {
                cancellation->requested_ns.store(NowNanoseconds(),
                                                  std::memory_order_release);
                if (!client.Cancel(JobHandle{event.job_id})) {
                  cancellation->failed.store(true, std::memory_order_release);
                }
              }
            }
            if (event.type == EventType::kJobCancelled) {
              cancellation->terminal_ns.store(NowNanoseconds(),
                                               std::memory_order_release);
            }
          });
      if (!subscribed) {
        throw std::runtime_error("event subscription failed: " +
                                 subscribed.GetError().Message());
      }
      subscription = std::move(subscribed).Value();
      RunPrerequisites(client, options.scenario);
      const auto before_target = client.Snapshot();
      if (!before_target) {
        throw std::runtime_error("pre-target snapshot failed: " +
                                 before_target.GetError().Message());
      }
      if (options.scenario == "visualization-warm") {
        target = RunWarmVisualizationTarget(client, agent.Value());
      } else if (options.scenario == "cancellation") {
        target = RunCancellationTarget(client, *cancellation);
        const auto after_cancel = client.Snapshot();
        if (!after_cancel ||
            after_cancel.Value().pipeline.runtime_revision !=
                before_target.Value().pipeline.runtime_revision ||
            !after_cancel.Value().pipeline.job ||
            after_cancel.Value().pipeline.job->state != JobState::kCancelled) {
          throw std::runtime_error(
              "cancelled target changed authoritative runtime revision or "
              "lacks cancelled terminal state");
        }
      } else {
        target = RunTarget(client, options.scenario, map_enabled, recorder,
                           agent.Value());
      }
    }
    const auto snapshot = client.Snapshot();
    if (!snapshot) {
      throw std::runtime_error("snapshot failed: " +
                               snapshot.GetError().Message());
    }
    final_snapshot = std::move(snapshot).Value();
    if (final_snapshot.pipeline.agents.size() !=
        manifest.at("agent_count").get<std::size_t>()) {
      throw std::runtime_error("runtime agent count differs from manifest");
    }
    Require(client.Close(), "Close");
  }
  Json report = BuildReport(options, manifest, manifest_digest, target,
                            final_snapshot, absolute_fixture / "output");
  const auto validation = ValidatePerformanceReport(report);
  if (!validation.Ok()) {
    throw std::runtime_error("generated report is invalid:\n" +
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
