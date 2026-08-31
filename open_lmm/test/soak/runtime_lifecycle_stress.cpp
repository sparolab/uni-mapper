#include "support/runtime/recording_runtime_port.hpp"
#include "support/soak/soak_metrics.hpp"
#include "support/synchronization.hpp"

#include <runtime/service/runtime_service.hpp>

#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <nlohmann/json.hpp>
#include <sys/utsname.h>
#include <unistd.h>

namespace fs = std::filesystem;
namespace soak = open_lmm::test::soak;
using Json = nlohmann::json;

#ifndef OPEN_LMM_SOAK_SANITIZER_NAME
#define OPEN_LMM_SOAK_SANITIZER_NAME "none"
#endif

namespace {

struct Arguments {
  uint64_t iterations = 100;
  uint64_t warmup = 10;
  uint64_t seed = 104729;
  std::string profile = "fast";
  std::string scenario = "runtime-lifecycle";
  std::string git_commit = std::string(40, '0');
  bool git_dirty = false;
  std::optional<fs::path> report;
};

uint64_t ParseUnsigned(const std::string& value, const std::string& option) {
  std::size_t consumed = 0;
  uint64_t parsed = 0;
  try {
    parsed = std::stoull(value, &consumed);
  } catch (const std::exception&) {
    throw std::invalid_argument(option + " requires an unsigned integer");
  }
  if (consumed != value.size()) {
    throw std::invalid_argument(option + " requires an unsigned integer");
  }
  return parsed;
}

Arguments ParseArguments(int argc, char** argv) {
  Arguments arguments;
  for (int index = 1; index < argc; ++index) {
    const std::string option = argv[index];
    auto text = [&]() -> std::string {
      if (++index >= argc) throw std::invalid_argument("missing value for " + option);
      return argv[index];
    };
    if (option == "--iterations")
      arguments.iterations = ParseUnsigned(text(), option);
    else if (option == "--warmup")
      arguments.warmup = ParseUnsigned(text(), option);
    else if (option == "--seed")
      arguments.seed = ParseUnsigned(text(), option);
    else if (option == "--profile")
      arguments.profile = text();
    else if (option == "--scenario")
      arguments.scenario = text();
    else if (option == "--git-commit")
      arguments.git_commit = text();
    else if (option == "--git-dirty")
      arguments.git_dirty = true;
    else if (option == "--report")
      arguments.report = fs::path(text());
    else if (option == "--help") {
      std::cout << "usage: open_lmm_runtime_lifecycle_stress_tests "
                   "[--iterations N] [--warmup N] [--seed N] "
                   "[--profile fast|sanitizer|nightly] "
                   "[--scenario runtime-lifecycle|all-headless] "
                   "[--git-commit SHA] [--git-dirty] [--report FILE]\n";
      std::exit(0);
    } else {
      throw std::invalid_argument("unknown option: " + option);
    }
  }
  if (arguments.iterations < 5 || arguments.iterations > 5'000 ||
      arguments.warmup >= arguments.iterations ||
      arguments.iterations - arguments.warmup < 4) {
    throw std::invalid_argument(
        "iterations must be 5..5000 with four post-warmup samples");
  }
  if (arguments.git_commit.size() != 40 ||
      !std::all_of(arguments.git_commit.begin(), arguments.git_commit.end(),
                   [](unsigned char value) { return std::isxdigit(value); })) {
    throw std::invalid_argument("git commit must contain 40 hexadecimal digits");
  }
  if (arguments.scenario != "runtime-lifecycle" &&
      arguments.scenario != "all-headless") {
    throw std::invalid_argument("unsupported lifecycle scenario");
  }
  return arguments;
}

class TemporaryTree {
 public:
  TemporaryTree() {
    path_ = fs::temp_directory_path() /
            ("open_lmm_runtime_lifecycle_soak_" +
             std::to_string(::getpid()) + "_" +
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

open_lmm::AgentId Id(const char* value) {
  return open_lmm::AgentId::Parse(value).Value();
}

class StressPort final : public open_lmm::test::RuntimePortFixture {
 public:
  StressPort() : RuntimePortFixture({Id("agent")}) {}

  open_lmm::Result<void> ExecuteFixture(
      const open_lmm::ExecutionCommand&,
      const open_lmm::ExecutionContext& context) override {
    const unsigned call = calls_.fetch_add(1, std::memory_order_relaxed);
    if (call == 0 || call == 2) return open_lmm::Result<void>::Ok();
    if (call == 1) {
      cancel_gate.ArriveAndWait("release pre-commit cancellation");
      return Cancelled(context, "explicit cancellation");
    }
    if (call == 3) {
      gate_late_receipt_.store(true, std::memory_order_release);
      return open_lmm::Result<void>::Ok();
    }
    if (call == 4) {
      close_gate.ArriveAndWait("release close cancellation");
      const auto result = Cancelled(context, "close cancellation");
      close_worker_exited.Signal();
      return result;
    }
    return open_lmm::Result<void>::Failure(
        open_lmm::Error::InvalidArgument("unexpected soak command"));
  }

  open_lmm::test::PhaseGate cancel_gate;
  open_lmm::test::PhaseGate close_gate;
  open_lmm::test::ManualResetEvent committed_before_return;
  open_lmm::test::ManualResetEvent release_committed_receipt;
  open_lmm::test::ManualResetEvent close_worker_exited;

 private:
  open_lmm::Result<void> AfterCommitFixture(
      const open_lmm::ExecutionCommand&, const open_lmm::ExecutionContext&,
      const open_lmm::ExecutionReceipt&) override {
    if (!gate_late_receipt_.exchange(false, std::memory_order_acq_rel))
      return open_lmm::Result<void>::Ok();
    committed_before_return.Signal();
    release_committed_receipt.Wait("release committed soak receipt");
    return open_lmm::Result<void>::Ok();
  }

  static open_lmm::Result<void> Cancelled(
      const open_lmm::ExecutionContext& context, const char* message) {
    if (!context.cancellation->IsCancellationRequested()) {
      return open_lmm::Result<void>::Failure(
          open_lmm::Error::InvalidArgument(
              "cancellation token was not authoritative"));
    }
    return open_lmm::Result<void>::Failure(
        open_lmm::Error::Cancelled(message));
  }

  std::atomic<unsigned> calls_{0};
  std::atomic<bool> gate_late_receipt_{false};
};

void WriteRootConfig(const fs::path& directory, const fs::path& output_root) {
  fs::create_directories(directory);
  std::ofstream output(directory / "config.json");
  if (!output) throw std::runtime_error("failed to create soak root config");
  output << "{\n"
         << "  \"global\": {\n"
         << "    \"config_path\": \"\",\n"
         << "    \"config_map_server\": \"map.json\",\n"
         << "    \"config_data_loader\": \"loader.json\",\n"
         << "    \"config_loop_detector\": \"loop.json\",\n"
         << "    \"config_backend_optimizer\": \"optimizer.json\",\n"
         << "    \"config_dynamic_remover\": \"remover.json\"\n"
         << "  },\n"
         << "  \"directory\": {\n"
         << "    \"root_dir_path\": \"/tmp/open-lmm-soak-data\",\n"
         << "    \"sub_dir_list\": [\"agent\"],\n"
         << "    \"root_save_dir\": \"" << output_root.string() << "\"\n"
         << "  }\n"
         << "}\n";
  if (!output) throw std::runtime_error("failed to write soak root config");
}

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
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

Json ExecutorJson(const open_lmm::BoundedExecutorSnapshot& snapshot) {
  return {{"worker_count", snapshot.worker_count},
          {"queue_capacity", snapshot.queue_capacity},
          {"queued_tasks", snapshot.queued_tasks},
          {"active_tasks", snapshot.active_tasks},
          {"waiting_submitters", snapshot.waiting_submitters},
          {"completed_tasks", snapshot.completed_tasks},
          {"cancelled_queued_tasks", snapshot.cancelled_queued_tasks}};
}

double Percentile(std::vector<double> values, double percentile) {
  if (values.empty()) return 0.0;
  std::sort(values.begin(), values.end());
  const std::size_t index = static_cast<std::size_t>(
      std::ceil(percentile * static_cast<double>(values.size()))) - 1;
  return values[std::min(index, values.size() - 1)];
}

Json LatencyJson(const std::vector<double>& milliseconds) {
  return {{"samples", milliseconds.size()},
          {"p50_ms", Percentile(milliseconds, 0.50)},
          {"p95_ms", Percentile(milliseconds, 0.95)},
          {"max_ms", *std::max_element(milliseconds.begin(),
                                        milliseconds.end())}};
}

Json InitialReport(const Arguments& arguments) {
  return {{"schema_version", 1},
          {"run_id", "runtime-lifecycle-" + arguments.profile},
          {"profile", arguments.profile},
          {"scenario", arguments.scenario},
          {"iterations", arguments.iterations},
          {"warmup_iterations", arguments.warmup},
          {"seed", arguments.seed},
          {"git",
           {{"commit", arguments.git_commit}, {"dirty", arguments.git_dirty}}},
          {"build",
           {{"compiler", __VERSION__},
            {"sanitizer", OPEN_LMM_SOAK_SANITIZER_NAME},
            {"build_type", "configured"}}},
          {"machine", MachineMetadata()},
          {"samples", Json::array()},
          {"slopes", Json::object()},
          {"latencies", Json::object()},
          {"failures", Json::array()},
          {"result", "fail"}};
}

struct RunData {
  std::vector<soak::MetricPoint> rss;
  std::vector<soak::MetricPoint> threads;
  std::vector<soak::MetricPoint> fds;
  std::vector<double> stage_latency_ms;
  std::vector<double> cancel_retry_latency_ms;
  std::vector<double> late_cancel_latency_ms;
  std::vector<double> close_latency_ms;
};

void AppendSample(Json& report, uint64_t iteration, const char* checkpoint,
                  const soak::ProcessMetrics& process,
                  const open_lmm::RuntimeService* service = nullptr,
                  const open_lmm::RuntimeSnapshot* snapshot = nullptr) {
  Json sample{{"iteration", iteration},
              {"checkpoint", checkpoint},
              {"process", soak::ProcessMetricsJson(process)},
              {"runtime_revision",
               snapshot ? Json(snapshot->pipeline.runtime_revision)
                        : Json(nullptr)},
              {"recent_event_count",
               snapshot ? Json(snapshot->pipeline.recent_events.size())
                        : Json(nullptr)},
              {"reserved_memory_bytes",
               service ? Json(service->Governor().ReservedMemoryBytes())
                       : Json(nullptr)},
              {"executor",
               service
                   ? ExecutorJson(service->Governor().AgentExecutor().Snapshot())
                   : Json(nullptr)}};
  if (service) {
    const auto diagnostics = service->Diagnostics();
    Json owner = {{"live_jobs", diagnostics.public_job_count},
                  {"live_subscriptions", diagnostics.subscriber_count},
                  {"callbacks", diagnostics.callbacks_in_flight},
                  {"visualization_cache_entries", nullptr},
                  {"visualization_cache_bytes", nullptr},
                  {"output_final_files", nullptr},
                  {"output_temporary_files", nullptr},
                  {"output_backup_entries", nullptr},
                  {"output_recovery_manifests", nullptr},
                  {"output_directory_count", nullptr},
                  {"loaded_plugin_mappings", nullptr}};
    sample["owner"] = std::move(owner);
  }
  report["samples"].push_back(std::move(sample));
}

RunData RunLifecycle(const Arguments& arguments, Json& report) {
  TemporaryTree fixture;
  const fs::path config = fixture.Path() / "config";
  const fs::path output = fixture.Path() / "output";
  WriteRootConfig(config, output);

  RunData data;
  std::optional<open_lmm::JobHandle> retired_job;
  AppendSample(report, 0, "before_owner", soak::SampleProcessMetrics());
  {
    std::shared_ptr<StressPort> port;
    open_lmm::RuntimeService service(
        1, [&](const open_lmm::BootstrapConfigSnapshot&, const fs::path&)
               -> open_lmm::Result<
                   std::shared_ptr<open_lmm::StageRuntimePort>> {
          port = std::make_shared<StressPort>();
          return open_lmm::Result<
              std::shared_ptr<open_lmm::StageRuntimePort>>::Ok(
              port);
        });

    for (uint64_t iteration = 0; iteration < arguments.iterations;
         ++iteration) {
      Require(service.Open({config, "soak-" + std::to_string(iteration),
                            output})
                  .IsOk(),
              "Open failed at iteration " + std::to_string(iteration));
      if (retired_job) {
        Require(!service.Wait(*retired_job),
                "retired epoch job was accepted at iteration " +
                    std::to_string(iteration));
      }

      std::atomic<uint64_t> callback_count{0};
      open_lmm::test::ManualResetEvent first_terminal_event;
      auto subscribed = service.SubscribeEvents(
          [&](const open_lmm::ExecutionEvent& event) {
            ++callback_count;
            if (event.type == open_lmm::EventType::kJobCompleted)
              first_terminal_event.Signal();
          });
      Require(subscribed.IsOk(), "SubscribeEvents failed");
      auto subscription = std::move(subscribed).Value();

      auto opened = service.Snapshot();
      Require(opened && opened.Value().pipeline.runtime_revision == 1,
              "Open did not publish revision 1");
      const auto after_open_metrics = soak::SampleProcessMetrics();
      AppendSample(report, iteration, "after_open", after_open_metrics,
                   &service, &opened.Value());

      open_lmm::ExecutionRequest request;
      request.kind = open_lmm::ExecutionRequestKind::kStage;
      request.stage = open_lmm::StageId::kDataLoad;
      const auto stage_started = std::chrono::steady_clock::now();
      auto submitted = service.Submit(request);
      Require(submitted && service.Wait(submitted.Value()),
              "DataLoad run failed");
      first_terminal_event.Wait("first terminal event dispatched");
      data.stage_latency_ms.push_back(
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - stage_started)
              .count());
      retired_job = submitted.Value();

      auto terminal = service.Snapshot();
      Require(terminal && terminal.Value().pipeline.runtime_revision == 2 &&
                  terminal.Value().pipeline.recent_events.size() <= 256 &&
                  callback_count.load(std::memory_order_acquire) != 0,
              "terminal authority/event invariant failed");
      AppendSample(report, iteration, "operation_terminal",
                   soak::SampleProcessMetrics(), &service,
                   &terminal.Value());

      const auto cancel_retry_started = std::chrono::steady_clock::now();
      auto cancelled = service.Submit(request);
      Require(cancelled.IsOk(), "cancel fixture submission failed");
      port->cancel_gate.WaitUntilEntered("cancel fixture entered");
      Require(service.Cancel(cancelled.Value()).IsOk(),
              "pre-commit cancellation was rejected");
      port->cancel_gate.Release();
      Require(!service.Wait(cancelled.Value()),
              "pre-commit cancellation reported committed success");
      auto retry = service.Submit(request);
      Require(retry && service.Wait(retry.Value()),
              "Run/Cancel/Run retry failed");
      data.cancel_retry_latency_ms.push_back(
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - cancel_retry_started)
              .count());
      auto retried = service.Snapshot();
      Require(retried && retried.Value().pipeline.runtime_revision == 3,
              "retry did not publish the next authoritative revision");

      const auto late_cancel_started = std::chrono::steady_clock::now();
      auto late = service.Submit(request);
      Require(late.IsOk(), "late-cancel fixture submission failed");
      port->committed_before_return.Wait("committed soak receipt withheld");
      Require(port->Snapshot().revision == 4,
              "late-cancel fixture did not commit before receipt return");
      Require(service.Cancel(late.Value()).IsOk(),
              "late cancellation request was rejected");
      port->release_committed_receipt.Signal();
      Require(service.Wait(late.Value()).IsOk(),
              "committed success did not win over late cancellation");
      auto late_committed = service.Snapshot();
      Require(late_committed &&
                  late_committed.Value().pipeline.runtime_revision == 4 &&
                  late_committed.Value().pipeline.job &&
                  late_committed.Value().pipeline.job->state ==
                      open_lmm::JobState::kSucceeded &&
                  late_committed.Value().pipeline.job->cancellation
                      .cancel_requested_at_unix_ns,
              "late cancellation did not preserve committed authority and telemetry");
      data.late_cancel_latency_ms.push_back(
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - late_cancel_started)
              .count());

      subscription.Reset();
      auto closing_job = service.Submit(request);
      Require(closing_job.IsOk(), "close fixture submission failed");
      port->close_gate.WaitUntilEntered("close fixture entered");
      Require(service.Cancel(closing_job.Value()).IsOk(),
              "close fixture cancellation was rejected");
      const auto close_started = std::chrono::steady_clock::now();
      open_lmm::test::ManualResetEvent close_invoked;
      open_lmm::test::ManualResetEvent close_completed;
      open_lmm::Result<void> closed = open_lmm::Result<void>::Failure(
          open_lmm::Error::InvalidArgument("close thread did not run"));
      std::thread closer([&] {
        close_invoked.Signal();
        closed = service.Close(open_lmm::CloseMode::kCancelAndWait);
        close_completed.Signal();
      });
      close_invoked.Wait("close thread started");
      Require(!close_completed.IsSignaled(),
              "Close returned while the cancelled worker was gated");
      port->close_gate.Release();
      closer.join();
      port->close_worker_exited.Wait("cancelled close worker exited");
      Require(closed.IsOk() && !service.IsOpen(), "Close failed");
      Require(service.Close(open_lmm::CloseMode::kCancelAndWait).IsOk() &&
                  !service.IsOpen(),
              "duplicate idle Close was not idempotent");
      data.close_latency_ms.push_back(
          std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - close_started)
              .count());
      const auto executor = service.Governor().AgentExecutor().Snapshot();
      Require(service.Governor().ReservedMemoryBytes() == 0 &&
                  executor.queued_tasks == 0 && executor.active_tasks == 0 &&
                  executor.waiting_submitters == 0,
              "resource owner did not return to idle after Close");

      std::error_code cleanup_error;
      fs::remove_all(output, cleanup_error);
      Require(!cleanup_error, "failed to remove test-owned successful output");
      fs::create_directories(output);

      const auto after_close = soak::SampleProcessMetrics();
      Require(after_close.unavailable.empty(),
              "required Linux process metric became unavailable");
      AppendSample(report, iteration, "after_close", after_close, &service);
      data.rss.push_back({iteration,
                          static_cast<double>(*after_close.rss_bytes)});
      data.threads.push_back(
          {iteration, static_cast<double>(*after_close.thread_count)});
      data.fds.push_back(
          {iteration, static_cast<double>(*after_close.fd_count)});
    }
  }
  AppendSample(report, arguments.iterations, "after_owner_destruction",
               soak::SampleProcessMetrics());
  return data;
}

void FinishReport(const Arguments& arguments, const RunData& data,
                  Json& report) {
  const auto rss = soak::AnalyzeSlope(data.rss, arguments.warmup);
  const auto threads = soak::AnalyzeSlope(data.threads, arguments.warmup);
  const auto fds = soak::AnalyzeSlope(data.fds, arguments.warmup);
  report["slopes"] = {{"rss_bytes", soak::SlopeAnalysisJson(rss)},
                       {"thread_count", soak::SlopeAnalysisJson(threads)},
                       {"fd_count", soak::SlopeAnalysisJson(fds)}};
  report["latencies"] =
      {{"stage", LatencyJson(data.stage_latency_ms)},
       {"cancel_retry", LatencyJson(data.cancel_retry_latency_ms)},
       {"late_cancel", LatencyJson(data.late_cancel_latency_ms)},
       {"close", LatencyJson(data.close_latency_ms)}};
  Require(threads.theil_sen_per_iteration == 0.0 &&
              fds.theil_sen_per_iteration == 0.0 &&
              data.threads.back().value == threads.minimum &&
              data.fds.back().value == fds.minimum,
          "thread or fd count has non-zero slope or did not return to its "
          "post-warmup minimum");
  report["result"] = "pass";
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const Arguments arguments = ParseArguments(argc, argv);
    Json report = InitialReport(arguments);
    try {
      const RunData data = RunLifecycle(arguments, report);
      FinishReport(arguments, data, report);
    } catch (const std::exception& error) {
      report["failures"].push_back(
          {{"iteration", nullptr},
           {"phase", "runtime_lifecycle"},
           {"message", error.what()}});
      report["result"] = "fail";
    }
    const auto validation = soak::ValidateSoakReport(report);
    if (!validation.Ok()) {
      throw std::runtime_error("invalid soak report:\n" +
                               validation.Summary());
    }
    if (arguments.report) soak::WriteJsonExclusive(*arguments.report, report);
    const bool passed = report.at("result") == "pass";
    std::cout << "runtime lifecycle stress=" << (passed ? "PASS" : "FAIL")
              << " iterations=" << arguments.iterations
              << " seed=" << arguments.seed << '\n';
    if (!passed) std::cerr << report.dump(2) << '\n';
    return passed ? 0 : 1;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid soak request: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "soak infrastructure failure: " << error.what() << '\n';
    return 2;
  }
}
