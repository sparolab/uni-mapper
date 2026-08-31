#include "support/runtime/recording_runtime_port.hpp"
#include "support/soak/owner_stress_support.hpp"
#include "support/synchronization.hpp"

#include <runtime/service/runtime_service.hpp>

#include <atomic>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>

#include <nlohmann/json.hpp>

#ifndef OPEN_LMM_SOAK_SANITIZER_NAME
#define OPEN_LMM_SOAK_SANITIZER_NAME "none"
#endif

namespace soak = open_lmm::test::soak;
namespace test = open_lmm::test;
namespace fs = std::filesystem;
using Json = nlohmann::json;

namespace {
using namespace open_lmm;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

void WriteRootConfig(const fs::path& directory, const fs::path& output_root) {
  fs::create_directories(directory);
  std::ofstream output(directory / "config.json");
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
  Require(static_cast<bool>(output), "failed to write root config");
}

std::string ReadText(const fs::path& path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

ConfigCandidate ReplacementCandidate(const fs::path& config,
                                     const fs::path& from,
                                     const fs::path& to) {
  auto json = ReadText(config / "config.json");
  const auto offset = json.find(from.string());
  Require(offset != std::string::npos,
          "replacement fixture could not find output root");
  json.replace(offset, from.string().size(), to.string());
  ConfigCandidate candidate;
  candidate.domain = ConfigDomain::kGlobal;
  candidate.document_json = std::move(json);
  return candidate;
}

class SuccessPort final : public test::RuntimePortFixture {
 public:
  SuccessPort() : RuntimePortFixture({Id("agent")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    return Result<void>::Ok();
  }
};

class FeedbackPort final : public test::RuntimePortFixture {
 public:
  FeedbackPort() : RuntimePortFixture({Id("agent")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext& context) override {
    AlignmentFeedbackSnapshot request;
    request.proposal.target_agent = Id("agent");
    request.proposal.source_agent = Id("agent");
    feedback_requested.Signal();
    auto response = context.alignment_feedback->Request(std::move(request),
                                                        context.cancellation);
    if (!response) return Result<void>::Failure(response.GetError());
    return Result<void>::Ok();
  }

  test::ManualResetEvent feedback_requested;
};

class VisualizationPort final : public test::RuntimePortFixture {
 public:
  VisualizationPort() : RuntimePortFixture({Id("agent")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    return Result<void>::Ok();
  }

  Result<VisualizationSnapshot> CreateVisualization(
      const AgentId& agent) const override {
    projection_gate.ArriveAndWait("release visualization projection");
    VisualizationSnapshot snapshot;
    snapshot.agent = agent;
    snapshot.revision = 1;
    snapshot.map_available = true;
    return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
  }

  mutable test::PhaseGate projection_gate;
};

class ResourceWaitPort final : public test::RuntimePortFixture {
 public:
  explicit ResourceWaitPort(std::shared_ptr<ResourceGovernor> governor)
      : RuntimePortFixture({Id("agent")}), governor_(std::move(governor)) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext& context) override {
    wait_started.Signal();
    auto acquired = governor_->AcquireHeavyMemoryPhase(context.cancellation);
    if (!acquired) return Result<void>::Failure(acquired.GetError());
    governor_->ReleaseHeavyMemoryPhase();
    return Result<void>::Ok();
  }

  test::ManualResetEvent wait_started;

 private:
  std::shared_ptr<ResourceGovernor> governor_;
};

ExecutionRequest StageRequest() {
  ExecutionRequest request;
  request.kind = ExecutionRequestKind::kStage;
  request.stage = StageId::kDataLoad;
  return request;
}

void ExerciseRootReplacement(const fs::path& root) {
  const auto config = root / "config";
  const auto old_output = root / "old-output";
  const auto new_output = root / "new-output";
  WriteRootConfig(config, old_output);
  unsigned factory_calls = 0;
  std::optional<fs::path> failed_candidate_output;
  RuntimeService service(
      1, [&](const BootstrapConfigSnapshot&, const fs::path& output_directory)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        if (factory_calls++ == 1) {
          failed_candidate_output = output_directory;
          fs::create_directories(output_directory);
          std::ofstream(output_directory / "unpublished.marker") << "candidate";
          return Result<std::shared_ptr<StageRuntimePort>>::Failure(
              Error::InvalidArgument("synthetic replacement build failure"));
        }
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<SuccessPort>());
      });
  Require(service.Open({config, "old"}).IsOk(), "root fixture open failed");
  auto old_job = service.Submit(StageRequest());
  Require(old_job && service.Wait(old_job.Value()),
          "root fixture initial job failed");
  const auto before = service.Snapshot().Value().pipeline;
  auto candidate = ReplacementCandidate(config, old_output, new_output);
  Require(!service.ReplaceRootConfig(
              {config, "stale"}, candidate,
              {before.runtime_revision - 1, before.config_revision}),
          "stale root replacement was accepted");
  Require(ReadText(config / "config.json").find(old_output.string()) !=
              std::string::npos,
          "stale root replacement mutated the committed file");
  Require(!service.ReplaceRootConfig(
              {config, "failed"}, candidate,
              {before.runtime_revision, before.config_revision}) &&
              failed_candidate_output && !fs::exists(*failed_candidate_output),
          "failed replacement candidate output was retained");
  auto after_failure = service.Snapshot();
  Require(after_failure && after_failure.Value().label == "old" &&
              after_failure.Value().pipeline.runtime_revision ==
                  before.runtime_revision &&
              ReadText(config / "config.json").find(old_output.string()) !=
                  std::string::npos,
          "failed replacement changed committed runtime or config authority");
  Require(service.ReplaceRootConfig(
              {config, "new"}, candidate,
              {before.runtime_revision, before.config_revision})
              .IsOk(),
          "idle root replacement failed");
  auto replaced = service.Snapshot();
  Require(replaced && replaced.Value().label == "new" &&
              replaced.Value().pipeline.recent_events.empty() &&
              !replaced.Value().pipeline.job &&
              ReadText(config / "config.json").find(new_output.string()) !=
                  std::string::npos &&
              !service.Wait(old_job.Value()),
          "replacement did not atomically publish a fresh epoch");
  Require(service.Close().IsOk(), "root replacement close failed");
}

void ExerciseFeedbackShutdown(const fs::path& root) {
  const auto config = root / "config";
  const auto output = root / "output";
  WriteRootConfig(config, output);
  std::shared_ptr<FeedbackPort> port;
  RuntimeService service(
      1, [&](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        port = std::make_shared<FeedbackPort>();
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(port);
      });
  Require(service.Open({config, "feedback"}).IsOk() &&
              service.SetAlignmentFeedbackEnabled(true).IsOk(),
          "feedback fixture open failed");
  auto job = service.Submit(StageRequest());
  Require(job.IsOk(), "feedback fixture submit failed");
  port->feedback_requested.Wait("feedback request entered");
  auto before = service.Snapshot().Value().pipeline;
  auto candidate = ReplacementCandidate(config, output,
                                        root / "replacement-output");
  Require(!service.ReplaceRootConfig(
              {config, "busy"}, candidate,
              {before.runtime_revision, before.config_revision}) &&
              !service.Close(CloseMode::kRejectIfRunning),
          "busy replacement or reject-if-running close changed authority");
  Require(service.Close(CloseMode::kCancelAndWait).IsOk() &&
              !service.IsOpen() && !service.Wait(job.Value()),
          "feedback wait was not cancelled and drained by Close");
}

void ExerciseSubscriptions(const fs::path& root) {
  const auto config = root / "config";
  WriteRootConfig(config, root / "output");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<SuccessPort>());
      });
  Require(service.Open({config, "subscriptions"}).IsOk(),
          "subscription fixture open failed");

  std::atomic<unsigned> self_calls{0};
  std::optional<ExecutionEventSubscription> self;
  auto self_result = service.SubscribeEvents([&](const ExecutionEvent&) {
    ++self_calls;
    if (self) self->Reset();
  });
  Require(self_result.IsOk(), "self-reset subscription failed");
  self.emplace(std::move(self_result).Value());
  auto first = service.Submit(StageRequest());
  Require(first && service.Wait(first.Value()) && self_calls.load() == 1,
          "self-reset callback observed events after reset");

  std::atomic<unsigned> throwing_calls{0};
  auto throwing = service.SubscribeEvents([&](const ExecutionEvent&) {
    if (throwing_calls.fetch_add(1) == 0)
      throw std::runtime_error("fixture callback");
  });
  Require(throwing.IsOk(), "throwing subscription failed");
  auto throwing_subscription = std::move(throwing).Value();
  auto second = service.Submit(StageRequest());
  Require(second && service.Wait(second.Value()),
          "callback exception escaped runtime dispatch");
  throwing_subscription.Reset();

  test::PhaseGate callback_gate;
  std::atomic<unsigned> gated_calls{0};
  auto gated = service.SubscribeEvents([&](const ExecutionEvent&) {
    if (gated_calls.fetch_add(1) == 0)
      callback_gate.ArriveAndWait("release in-flight callback");
  });
  Require(gated.IsOk(), "gated subscription failed");
  auto gated_subscription = std::move(gated).Value();
  Result<JobHandle> third = Result<JobHandle>::Failure(
      Error::InvalidArgument("submit thread did not run"));
  std::thread submitter([&] { third = service.Submit(StageRequest()); });
  callback_gate.WaitUntilEntered("in-flight callback entered");
  test::ManualResetEvent reset_started;
  test::ManualResetEvent reset_completed;
  std::thread resetter([&] {
    reset_started.Signal();
    gated_subscription.Reset();
    reset_completed.Signal();
  });
  reset_started.Wait("subscription reset started");
  Require(!reset_completed.IsSignaled(),
          "subscription reset returned before callback drain");
  callback_gate.Release();
  submitter.join();
  resetter.join();
  Require(third && service.Wait(third.Value()) && gated_calls.load() >= 1,
          "in-flight reset did not establish a callback barrier");
  const auto calls_after_reset = gated_calls.load();
  auto fourth = service.Submit(StageRequest());
  Require(fourth && service.Wait(fourth.Value()) &&
              gated_calls.load() == calls_after_reset,
          "callback started after subscription reset returned");
  Require(service.Close().IsOk(), "subscription fixture close failed");

  std::optional<ExecutionEventSubscription> survivor;
  {
    auto owner = std::make_unique<RuntimeService>(
        1, [](const BootstrapConfigSnapshot&, const fs::path&)
               -> Result<std::shared_ptr<StageRuntimePort>> {
          return Result<std::shared_ptr<StageRuntimePort>>::Ok(
              std::make_shared<SuccessPort>());
        });
    Require(owner->Open({config, "survivor"}).IsOk(),
            "survivor fixture open failed");
    auto subscribed = owner->SubscribeEvents([](const ExecutionEvent&) {});
    Require(subscribed.IsOk(), "survivor subscription failed");
    survivor.emplace(std::move(subscribed).Value());
  }
  survivor->Reset();
}

void ExerciseCallbackAndOpenShutdown(const fs::path& root) {
  const auto callback_config = root / "callback-config";
  WriteRootConfig(callback_config, root / "callback-output");
  RuntimeService callback_service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<SuccessPort>());
      });
  Require(callback_service.Open({callback_config, "callback"}).IsOk(),
          "callback-close fixture open failed");
  test::PhaseGate callback_gate;
  auto subscribed = callback_service.SubscribeEvents(
      [&](const ExecutionEvent&) {
        callback_gate.ArriveAndWait("release callback during Close");
      });
  Require(subscribed.IsOk(), "callback-close subscription failed");
  auto subscription = std::move(subscribed).Value();
  Result<JobHandle> job = Result<JobHandle>::Failure(
      Error::InvalidArgument("callback submit did not run"));
  std::thread submitter([&] { job = callback_service.Submit(StageRequest()); });
  callback_gate.WaitUntilEntered("callback entered before Close");
  test::ManualResetEvent close_started;
  test::ManualResetEvent close_completed;
  Result<void> closed = Result<void>::Failure(
      Error::InvalidArgument("callback close did not run"));
  std::thread closer([&] {
    close_started.Signal();
    closed = callback_service.Close(CloseMode::kCancelAndWait);
    close_completed.Signal();
  });
  close_started.Wait("callback close started");
  callback_service.WaitForLifecycleForDiagnostics(
      RuntimeLifecycleState::kClosing);
  Require(!close_completed.IsSignaled(),
          "Close returned before in-flight callback drained");
  callback_gate.Release();
  submitter.join();
  closer.join();
  Require(job.IsOk() && closed.IsOk() && !callback_service.IsOpen(),
          "callback Close did not drain and retire runtime");
  subscription.Reset();

  const auto open_config = root / "open-config";
  WriteRootConfig(open_config, root / "open-output");
  test::PhaseGate open_gate;
  std::optional<fs::path> cancelled_candidate_output;
  RuntimeService opening_service(
      1, [&](const BootstrapConfigSnapshot&,
             const fs::path& output_directory)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        cancelled_candidate_output = output_directory;
        fs::create_directories(output_directory);
        std::ofstream(output_directory / "unpublished.marker") << "candidate";
        open_gate.ArriveAndWait("release Open candidate");
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<SuccessPort>());
      });
  Result<void> opened = Result<void>::Failure(
      Error::InvalidArgument("Open thread did not run"));
  std::thread opening(
      [&] { opened = opening_service.Open({open_config, "opening"}); });
  open_gate.WaitUntilEntered("Open candidate entered");
  test::ManualResetEvent open_close_started;
  test::ManualResetEvent open_close_completed;
  Result<void> open_closed = Result<void>::Failure(
      Error::InvalidArgument("Open Close thread did not run"));
  std::thread open_closer([&] {
    open_close_started.Signal();
    open_closed = opening_service.Close(CloseMode::kCancelAndWait);
    open_close_completed.Signal();
  });
  open_close_started.Wait("Close during Open started");
  opening_service.WaitForLifecycleForDiagnostics(
      RuntimeLifecycleState::kClosing);
  Require(!open_close_completed.IsSignaled(),
          "Close returned before Open candidate retired");
  open_gate.Release();
  opening.join();
  open_closer.join();
  Require(!opened && open_closed && !opening_service.IsOpen() &&
              cancelled_candidate_output &&
              !fs::exists(*cancelled_candidate_output),
          "Close during Open published or retained a cancelled candidate");
}

void ExerciseVisualizationShutdown(const fs::path& root) {
  const auto config = root / "config";
  WriteRootConfig(config, root / "output");
  std::shared_ptr<VisualizationPort> port;
  RuntimeService service(
      1, [&](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        port = std::make_shared<VisualizationPort>();
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(port);
      });
  Require(service.Open({config, "visualization"}).IsOk(),
          "visualization-close fixture open failed");
  Result<VisualizationSnapshot> projected =
      Result<VisualizationSnapshot>::Failure(
          Error::InvalidArgument("visualization query did not run"));
  std::thread projector(
      [&] { projected = service.Visualization(Id("agent")); });
  port->projection_gate.WaitUntilEntered("visualization projection entered");
  test::ManualResetEvent close_started;
  test::ManualResetEvent close_completed;
  Result<void> closed = Result<void>::Failure(
      Error::InvalidArgument("visualization Close did not run"));
  std::thread closer([&] {
    close_started.Signal();
    closed = service.Close(CloseMode::kCancelAndWait);
    close_completed.Signal();
  });
  close_started.Wait("visualization Close started");
  service.WaitForLifecycleForDiagnostics(RuntimeLifecycleState::kClosing);
  Require(!close_completed.IsSignaled(),
          "Close returned before visualization operation drained");
  port->projection_gate.Release();
  projector.join();
  closer.join();
  Require(projected && closed && !service.IsOpen(),
          "visualization operation was not drained before Close returned");
}

void ExerciseResourceAdmissionShutdown(const fs::path& root) {
  const auto config = root / "config";
  WriteRootConfig(config, root / "output");
  auto admission =
      std::make_shared<ResourceGovernor>(ResourceBudget{1, 1, 1024});
  Require(admission->AcquireHeavyMemoryPhase({}).IsOk(),
          "resource fixture could not reserve the heavy phase");
  std::shared_ptr<ResourceWaitPort> port;
  RuntimeService service(
      1, [&](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        port = std::make_shared<ResourceWaitPort>(admission);
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(port);
      });
  Require(service.Open({config, "resource-wait"}).IsOk(),
          "resource-wait fixture open failed");
  auto job = service.Submit(StageRequest());
  Require(job.IsOk(), "resource-wait fixture submit failed");
  port->wait_started.Wait("resource admission wait entered");
  Require(service.Close(CloseMode::kCancelAndWait).IsOk() &&
              !service.IsOpen() && !service.Wait(job.Value()),
          "Close did not cancel and drain resource admission wait");
  admission->ReleaseHeavyMemoryPhase();
  Require(admission->AcquireHeavyMemoryPhase({}).IsOk(),
          "resource admission owner was not reusable after cancellation");
  admission->ReleaseHeavyMemoryPhase();
}

soak::ProcessSeries Run(const soak::RunOptions& options, Json& report) {
  soak::TemporaryDirectory tree("open_lmm_runtime_concurrency_soak");
  soak::ProcessSeries series;
  for (uint64_t iteration = 0; iteration < options.iterations; ++iteration) {
    const auto iteration_root = tree.Path() / std::to_string(iteration);
    ExerciseRootReplacement(iteration_root / "replacement");
    ExerciseFeedbackShutdown(iteration_root / "feedback");
    ExerciseSubscriptions(iteration_root / "subscriptions");
    ExerciseCallbackAndOpenShutdown(iteration_root / "shutdown");
    ExerciseVisualizationShutdown(iteration_root / "visualization");
    ExerciseResourceAdmissionShutdown(iteration_root / "resource-admission");
    std::error_code error;
    fs::remove_all(iteration_root, error);
    Require(!error, "failed to clean iteration fixture");
    const auto process = soak::SampleProcessMetrics();
    soak::AppendOwnerSample(report, iteration, "runtime_owners_destroyed",
                            process, soak::EmptyOwnerMetrics());
    soak::AddProcessPoint(series, iteration, process);
  }
  return series;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = soak::ParseRunOptions(argc, argv);
    Json report = soak::InitialOwnerReport(
        options, "runtime-concurrency", OPEN_LMM_SOAK_SANITIZER_NAME);
    try {
      const auto series = Run(options, report);
      soak::FinishOwnerReport(options, series, report);
    } catch (const std::exception& error) {
      report["failures"].push_back(
          {{"iteration", nullptr},
           {"phase", "runtime_concurrency"},
           {"message", error.what()}});
      report["result"] = "fail";
    }
    const auto validation = soak::ValidateSoakReport(report);
    if (!validation.Ok())
      throw std::runtime_error("invalid soak report:\n" +
                               validation.Summary());
    if (options.report) soak::WriteJsonExclusive(*options.report, report);
    const bool passed = report.at("result") == "pass";
    std::cout << "runtime concurrency stress=" << (passed ? "PASS" : "FAIL")
              << " iterations=" << options.iterations << '\n';
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
