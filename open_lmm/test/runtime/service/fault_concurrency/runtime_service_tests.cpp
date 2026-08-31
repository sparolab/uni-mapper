#include <runtime/service/runtime_service.hpp>

#include "support/runtime/recording_runtime_port.hpp"
#include "support/synchronization.hpp"

#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <optional>
#include <thread>
#include <vector>

namespace {
using namespace open_lmm;
namespace fs = std::filesystem;

void Check(bool value, const char* message) {
  if (value) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

void WriteRootConfig(const fs::path& directory, const fs::path& output_root,
                     const char* agent) {
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
         << "    \"root_dir_path\": \"/tmp/runtime-service-data\",\n"
         << "    \"sub_dir_list\": [\"" << agent << "\"],\n"
         << "    \"root_save_dir\": \"" << output_root.string() << "\"\n"
         << "  }\n"
         << "}\n";
}

std::string ReadText(const fs::path& path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

class InteractivePort final : public test::RuntimePortFixture {
 public:
  InteractivePort() : RuntimePortFixture({Id("agent")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext& context) override {
    if (!context.alignment_feedback->IsEnabled()) return Result<void>::Ok();
    AlignmentFeedbackSnapshot request;
    request.proposal.target_agent = Id("agent");
    request.proposal.source_agent = Id("agent");
    auto response = context.alignment_feedback->Request(std::move(request),
                                                        context.cancellation);
    if (!response) return Result<void>::Failure(response.GetError());
    return response.Value().decision == AlignmentDecision::kCancel
               ? Result<void>::Failure(Error::Cancelled("alignment feedback cancelled"))
               : Result<void>::Ok();
  }

  Result<VisualizationSnapshot> CreateVisualization(
      const AgentId& agent) const override {
    VisualizationSnapshot snapshot;
    snapshot.agent = agent;
    snapshot.revision = 1;
    snapshot.map_available = true;
    return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
  }
};

class BlockingPort final : public test::RuntimePortFixture {
 public:
  BlockingPort() : RuntimePortFixture({Id("agent")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext& context) override {
    entered.store(true, std::memory_order_release);
    entered_event.Signal();
    const auto deadline = std::chrono::steady_clock::now() +
                          test::kWatchdogTimeout;
    std::unique_lock lock(cancellation_mutex);
    while (!context.cancellation->IsCancellationRequested() &&
           std::chrono::steady_clock::now() < deadline) {
      cancellation_changed.wait_for(lock, std::chrono::milliseconds(1));
    }
    Check(context.cancellation->IsCancellationRequested(),
          "blocking fixture observes cancellation before watchdog");
    exited.store(true, std::memory_order_release);
    exited_event.Signal();
    return Result<void>::Failure(Error::Cancelled("blocking fixture cancelled"));
  }

  std::atomic<bool> entered{false};
  std::atomic<bool> exited{false};
  test::ManualResetEvent entered_event;
  test::ManualResetEvent exited_event;
  std::mutex cancellation_mutex;
  std::condition_variable cancellation_changed;
};

class SuccessPort final : public test::RuntimePortFixture {
 public:
  SuccessPort() : RuntimePortFixture({Id("agent")}) {}
  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    return Result<void>::Ok();
  }
};

class RecoveryPort final : public test::RuntimePortFixture {
 public:
  RecoveryPort() : RuntimePortFixture({Id("agent")}) {}

  std::optional<Error> RecoveryAfterCommit() const override {
    Error recovery = Error::IoError(
        "manual recovery manifest: /tmp/recovery-runtime.json");
    recovery.WithExecution("file_transaction", "recovery_required")
        .MarkFatalRuntime();
    return recovery;
  }

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    return Result<void>::Ok();
  }

  Result<VisualizationSnapshot> CreateVisualization(
      const AgentId& agent) const override {
    VisualizationSnapshot snapshot;
    snapshot.agent = agent;
    snapshot.revision = 1;
    return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
  }
};

class RecoveryReceiptMismatchPort final : public test::RuntimePortFixture {
 public:
  RecoveryReceiptMismatchPort() : RuntimePortFixture({Id("agent")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    return Result<void>::Ok();
  }

  std::optional<Error> RecoveryAfterCommit() const override {
    Error recovery = Error::IoError(
        "manual recovery manifest: /tmp/compound-recovery.json");
    recovery.WithExecution("file_transaction", "recovery_required")
        .MarkFatalRuntime();
    return recovery;
  }

  ExecutionReceipt AdjustReceipt(ExecutionReceipt receipt) const override {
    receipt.recovery_required.reset();
    return receipt;
  }

  Result<VisualizationSnapshot> CreateVisualization(
      const AgentId& agent) const override {
    VisualizationSnapshot snapshot;
    snapshot.agent = agent;
    snapshot.revision = 2;
    return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
  }
};

class PostCommitConfigFaultPort final : public test::RuntimePortFixture {
 public:
  PostCommitConfigFaultPort() : RuntimePortFixture({Id("agent")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    return Result<void>::Ok();
  }

  Result<ConfigCommandReceipt> ApplyConfig(
      const ConfigCandidate& candidate, const ExpectedRevision& expected,
      const ExecutionContext& context) override {
    auto committed = RuntimePortFixture::ApplyConfig(candidate, expected,
                                                     context);
    if (!committed) return committed;
    return Result<ConfigCommandReceipt>::Failure(
        Error::InvalidArgument("fixture failed after config commit"));
  }
};

class OpenLatch {
 public:
  Result<std::shared_ptr<StageRuntimePort>> Create() {
    {
      std::lock_guard lock(mutex_);
      ++calls_;
      entered_ = true;
    }
    entered_changed_.notify_all();
    std::unique_lock lock(mutex_);
    released_.wait(lock, [&] { return release_; });
    return Result<std::shared_ptr<StageRuntimePort>>::Ok(
        std::make_shared<SuccessPort>());
  }

  void WaitForEntry() {
    std::unique_lock lock(mutex_);
    entered_changed_.wait(lock, [&] { return entered_; });
  }
  void Release() {
    {
      std::lock_guard lock(mutex_);
      release_ = true;
    }
    released_.notify_all();
  }
  [[nodiscard]] unsigned Calls() const {
    std::lock_guard lock(mutex_);
    return calls_;
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable entered_changed_;
  std::condition_variable released_;
  bool entered_ = false;
  bool release_ = false;
  unsigned calls_ = 0;
};

class OutputOpenLatch {
 public:
  Result<std::shared_ptr<StageRuntimePort>> Create(
      const fs::path& output_directory) {
    fs::create_directories(output_directory);
    std::ofstream(output_directory / "candidate.marker") << "created\n";

    bool block = false;
    {
      std::lock_guard lock(mutex_);
      outputs_.push_back(output_directory);
      block = outputs_.size() == 1;
      if (block) entered_ = true;
    }
    entered_changed_.notify_all();
    if (block) {
      std::unique_lock lock(mutex_);
      released_.wait(lock, [&] { return release_; });
    }
    return Result<std::shared_ptr<StageRuntimePort>>::Ok(
        std::make_shared<SuccessPort>());
  }

  void WaitForEntry() {
    std::unique_lock lock(mutex_);
    entered_changed_.wait(lock, [&] { return entered_; });
  }
  void Release() {
    {
      std::lock_guard lock(mutex_);
      release_ = true;
    }
    released_.notify_all();
  }
  [[nodiscard]] fs::path Output(std::size_t index) const {
    std::lock_guard lock(mutex_);
    Check(index < outputs_.size(), "requested output was captured");
    return outputs_[index];
  }

 private:
  mutable std::mutex mutex_;
  std::condition_variable entered_changed_;
  std::condition_variable released_;
  bool entered_ = false;
  bool release_ = false;
  std::vector<fs::path> outputs_;
};

ConfigCandidate ReplacementCandidate(const fs::path& config,
                                     const fs::path& from,
                                     const fs::path& to) {
  auto json = ReadText(config / "config.json");
  const auto offset = json.find(from.string());
  Check(offset != std::string::npos, "replacement fixture locates output root");
  json.replace(offset, from.string().size(), to.string());
  ConfigCandidate candidate;
  candidate.domain = ConfigDomain::kGlobal;
  candidate.document_json = std::move(json);
  return candidate;
}

void TestSingleRuntimeLifecycleAndJobIdentity() {
  const auto root = fs::temp_directory_path() / "open_lmm_single_runtime_tests";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<InteractivePort>());
      });

  Check(service.Open({root / "config", "one"}).IsOk(), "open one runtime");
  Check(!service.Open({root / "config", "two"}), "reject a second runtime");
  Check(service.SetAlignmentFeedbackEnabled(true).IsOk(),
        "interactive fixture explicitly enables feedback authority");
  std::atomic<uint64_t> queued{0};
  test::ManualResetEvent queued_event;
  test::ManualResetEvent feedback_event;
  auto subscribed = service.SubscribeEvents([&](const ExecutionEvent& event) {
    if (event.type == EventType::kJobQueued) {
      queued.store(event.job_id);
      queued_event.Signal();
    }
    if (event.type == EventType::kAlignmentFeedbackRequested) {
      feedback_event.Signal();
    }
  });
  Check(subscribed.IsOk(), "subscribe to unkeyed events");
  auto subscription = std::move(subscribed).Value();

  ExecutionRequest one_stage;
  one_stage.kind = ExecutionRequestKind::kStage;
  one_stage.stage = StageId::kDataLoad;
  auto first = service.Submit(one_stage);
  Check(first && first.Value().value != 0, "submit returns opaque job handle");
  queued_event.Wait("queued event is delivered");
  Check(queued.load() == first.Value().value,
        "queued event and submit share the public job handle");
  feedback_event.Wait("feedback is visible from the sole runtime");
  const auto feedback = service.AlignmentFeedback().Value();
  Check(service.RespondToAlignment(
            first.Value(), {feedback->proposal.request_id,
                            AlignmentDecision::kAccept, std::nullopt,
                            feedback->session_revision}) &&
            service.Wait(first.Value()),
        "feedback and wait target the same unkeyed job handle");
  const auto last_before_replacement = service.Snapshot().Value()
                                           .pipeline.recent_events.back().sequence;

  auto replacement = ReplacementCandidate(root / "config", root / "output",
                                          root / "replaced-output");
  const auto before_replacement = service.Snapshot().Value().pipeline;
  Check(!service.ReplaceRootConfig({root / "config", "stale"}, replacement,
                                   {before_replacement.runtime_revision - 1,
                                    before_replacement.config_revision}),
        "root replacement rejects a stale authoritative revision");
  Check(service.ReplaceRootConfig(
            {root / "config", "replacement"}, replacement,
            {before_replacement.runtime_revision,
             before_replacement.config_revision})
            .IsOk(),
        "idle root replacement succeeds");
  auto snapshot = service.Snapshot();
  Check(snapshot && snapshot.Value().label == "replacement" &&
            snapshot.Value().output_directory.parent_path() ==
                root / "replaced-output" &&
            ReadText(root / "config/config.json").find("replaced-output") !=
                std::string::npos,
        "replacement atomically exposes new runtime and persisted root config");
  Check(snapshot.Value().pipeline.recent_events.empty() &&
            !snapshot.Value().pipeline.job,
        "replacement starts an empty current-epoch replay snapshot");
  feedback_event.Reset();
  auto second = service.Submit(one_stage);
  Check(second && second.Value().value != first.Value().value,
        "handles are never reused after controller replacement");
  Check(!service.Wait(first.Value()), "retired handle is rejected by new epoch");
  feedback_event.Wait("replacement owns a new feedback channel");
  const auto second_feedback = service.AlignmentFeedback().Value();
  Check(service.RespondToAlignment(
            second.Value(), {second_feedback->proposal.request_id,
                             AlignmentDecision::kAccept, std::nullopt,
                             second_feedback->session_revision}) &&
            service.Wait(second.Value()),
        "new runtime executes through the same public API");
  const auto after_second = service.Snapshot().Value().pipeline.recent_events;
  Check(!after_second.empty() &&
            after_second.front().sequence > last_before_replacement,
        "replacement replay contains only new events with a monotonic sequence");
  subscription.Reset();
  Check(service.Close().IsOk() && !service.IsOpen(), "close retires sole runtime");
  fs::remove_all(root);
}

void TestHeadlessFeedbackAndDisableCancellation() {
  const auto root = fs::temp_directory_path() / "open_lmm_single_runtime_feedback";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<InteractivePort>());
      });
  Check(service.Open({root / "config", "headless"}).IsOk(),
        "headless feedback fixture opens");
  test::ManualResetEvent feedback_requested;
  auto subscribed = service.SubscribeEvents([&](const ExecutionEvent& event) {
    if (event.type == EventType::kAlignmentFeedbackRequested) {
      feedback_requested.Signal();
    }
  });
  Check(subscribed.IsOk(), "feedback fixture subscribes to committed events");
  auto subscription = std::move(subscribed).Value();
  ExecutionRequest request;
  request.kind = ExecutionRequestKind::kStage;
  request.stage = StageId::kDataLoad;
  auto headless = service.Submit(request);
  Check(headless && service.Wait(headless.Value()),
        "feedback-disabled runtime never waits for interactive input");
  Check(!service.AlignmentFeedback().Value().has_value(),
        "headless runtime has no published feedback request");

  Check(service.SetAlignmentFeedbackEnabled(true).IsOk(),
        "authority can be enabled for an interactive owner");
  auto interactive = service.Submit(request);
  Check(interactive.IsOk(), "interactive fixture submits");
  feedback_requested.Wait("interactive feedback request becomes visible");
  Check(service.SetAlignmentFeedbackEnabled(false).IsOk(),
        "disabling authority cancels the active request");
  Check(!service.Wait(interactive.Value()),
        "cancelled feedback request terminates its waiting command");
  const auto terminal = service.AlignmentFeedback().Value();
  Check(terminal &&
            terminal->review_state == AlignmentReviewState::kCancelled &&
            !terminal->terminal_message.empty(),
        "disabled authority preserves a terminal read-only review");
  subscription.Reset();
  Check(service.Close().IsOk(), "headless feedback fixture closes");
  fs::remove_all(root);
}

void TestReplacementAndCloseRejectBusyRuntime() {
  const auto root = fs::temp_directory_path() / "open_lmm_single_runtime_busy";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  std::shared_ptr<BlockingPort> port;
  RuntimeService service(
      1, [&](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        port = std::make_shared<BlockingPort>();
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(port);
      });
  Check(service.Open({root / "config", "busy"}).IsOk(), "open busy fixture");
  auto job = service.Submit({ExecutionRequestKind::kStage, StageId::kDataLoad});
  Check(job.IsOk(), "submit blocking job");
  port->entered_event.Wait("blocking job entered");
  auto candidate = ReplacementCandidate(root / "config", root / "output",
                                        root / "replacement-output");
  const auto before_replacement = service.Snapshot().Value().pipeline;
  Check(!service.ReplaceRootConfig(
            {root / "config", "replacement"}, candidate,
            {before_replacement.runtime_revision,
             before_replacement.config_revision}),
        "replacement rejects an active operation instead of creating another session");
  Check(!service.Close(CloseMode::kRejectIfRunning),
        "non-cancelling close rejects active job");
  Check(service.Close(CloseMode::kCancelAndWait).IsOk(),
        "close cancels before waiting for active work");
  port->exited_event.Wait("cancelled worker exits without close/wait deadlock");
  fs::remove_all(root);
}

void TestSubscriptionResetBarrier() {
  const auto root = fs::temp_directory_path() / "open_lmm_single_runtime_subscription";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<SuccessPort>());
      });
  Check(service.Open({root / "config", "subscription"}).IsOk(),
        "subscription fixture opens one runtime");
  std::atomic<unsigned> calls{0};
  std::optional<ExecutionEventSubscription> own_subscription;
  auto subscribed = service.SubscribeEvents([&](const ExecutionEvent&) {
    ++calls;
    if (own_subscription) own_subscription->Reset();
  });
  Check(subscribed.IsOk(), "runtime event subscription succeeds");
  own_subscription.emplace(std::move(subscribed).Value());
  ExecutionRequest request;
  request.kind = ExecutionRequestKind::kStage;
  request.stage = StageId::kDataLoad;
  auto job = service.Submit(request);
  Check(job.IsOk(), "runtime submits unkeyed job");
  Check(service.Wait(job.Value()).IsOk(), "runtime waits for unkeyed job");
  Check(calls.load() == 1,
        "self-reset callback receives no event after its reset returns");
  Check(service.Close().IsOk(), "runtime closes one active instance");
  fs::remove_all(root);
}

void TestSubscriptionOutlivesRuntime() {
  const auto root = fs::temp_directory_path() / "open_lmm_single_runtime_subscription_lifetime";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  std::optional<ExecutionEventSubscription> subscription;
  {
    auto service = std::make_unique<RuntimeService>(
        1, [](const BootstrapConfigSnapshot&, const fs::path&)
               -> Result<std::shared_ptr<StageRuntimePort>> {
          return Result<std::shared_ptr<StageRuntimePort>>::Ok(
              std::make_shared<SuccessPort>());
        });
    Check(service->Open({root / "config", "subscription-lifetime"}).IsOk(),
          "subscription lifetime fixture opens");
    auto subscribed = service->SubscribeEvents([](const ExecutionEvent&) {});
    Check(subscribed.IsOk(), "subscription lifetime fixture subscribes");
    subscription.emplace(std::move(subscribed).Value());
  }
  subscription->Reset();
  subscription.reset();
  fs::remove_all(root);
}

void TestSubscriptionResetWaitsForCopiedCallback() {
  const auto root = fs::temp_directory_path() / "open_lmm_single_runtime_subscription_barrier";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<SuccessPort>());
      });
  Check(service.Open({root / "config", "subscription-barrier"}).IsOk(),
        "subscription barrier fixture opens");
  test::PhaseGate callback_gate;
  std::atomic<bool> reset_returned{false};
  std::atomic<unsigned> calls{0};
  auto subscribed = service.SubscribeEvents([&](const ExecutionEvent&) {
    const auto call = calls.fetch_add(1, std::memory_order_relaxed);
    if (call != 0) return;
    callback_gate.ArriveAndWait("release copied event callback");
  });
  Check(subscribed.IsOk(), "subscription barrier fixture subscribes");
  auto subscription = std::move(subscribed).Value();
  Result<JobHandle> job = Result<JobHandle>::Failure(Error::InvalidArgument("not run"));
  std::thread submitter([&] {
    job = service.Submit({ExecutionRequestKind::kStage, StageId::kDataLoad});
  });
  callback_gate.WaitUntilEntered("first callback was copied and entered");
  test::ManualResetEvent reset_started;
  test::ManualResetEvent reset_completed;
  std::thread resetter([&] {
    reset_started.Signal();
    subscription.Reset();
    reset_returned.store(true, std::memory_order_release);
    reset_completed.Signal();
  });
  reset_started.Wait("subscription reset invocation starts");
  Check(!reset_completed.IsSignaled() &&
            !reset_returned.load(std::memory_order_acquire),
        "subscription reset waits for an already copied callback");
  callback_gate.Release();
  submitter.join();
  resetter.join();
  Check(job && service.Wait(job.Value()), "barrier fixture command completes");
  Check(calls.load(std::memory_order_relaxed) == 1,
        "no callback runs after reset returns");
  Check(service.Close().IsOk(), "subscription barrier fixture closes");
  fs::remove_all(root);
}

void TestOpenCloseTransitionCancellation() {
  const auto root = fs::temp_directory_path() / "open_lmm_single_runtime_open_close";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  auto latch = std::make_shared<OpenLatch>();
  RuntimeService service(
      1, [latch](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> { return latch->Create(); });

  Result<void> opened = Result<void>::Failure(Error::InvalidArgument("not run"));
  std::thread opening([&] { opened = service.Open({root / "config", "opening"}); });
  latch->WaitForEntry();
  std::atomic<bool> close_returned{false};
  test::ManualResetEvent close_started;
  test::ManualResetEvent close_completed;
  Result<void> closed = Result<void>::Failure(Error::InvalidArgument("not run"));
  std::thread closing([&] {
    close_started.Signal();
    closed = service.Close();
    close_returned.store(true, std::memory_order_release);
    close_completed.Signal();
  });
  close_started.Wait("close invocation starts during open transition");
  Check(!close_completed.IsSignaled() &&
            !close_returned.load(std::memory_order_acquire),
        "close waits for an opening runtime transition");
  latch->Release();
  opening.join();
  closing.join();
  Check(!opened && closed && !service.IsOpen(),
        "cancelled open never publishes an active runtime");
  Check(service.Open({root / "config", "reopen"}).IsOk(),
        "runtime can open after cancelled opening transition");
  Check(service.Close().IsOk(), "reopened runtime closes");
  fs::remove_all(root);
}

void TestCancelledOpenRollsBackUnpublishedOutput(bool use_root_candidate) {
  const auto root = fs::temp_directory_path() /
                    (use_root_candidate
                         ? "open_lmm_root_candidate_output_rollback"
                         : "open_lmm_file_config_output_rollback");
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  auto latch = std::make_shared<OutputOpenLatch>();
  RuntimeService service(
      1, [latch](const BootstrapConfigSnapshot&,
                 const fs::path& output_directory)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return latch->Create(output_directory);
      });

  std::optional<ConfigCandidate> candidate;
  if (use_root_candidate) {
    candidate = ReplacementCandidate(root / "config", root / "output",
                                     root / "candidate-output");
  }
  const auto open = [&](const char* label) {
    const BootstrapRequest request{root / "config", label};
    return candidate ? service.Open(request, *candidate) : service.Open(request);
  };

  Result<void> opened = Result<void>::Failure(Error::InvalidArgument("not run"));
  std::thread opening([&] { opened = open("opening"); });
  latch->WaitForEntry();
  const auto unpublished_output = latch->Output(0);
  Check(fs::is_regular_file(unpublished_output / "candidate.marker"),
        "fixture creates output before publication");

  std::atomic<bool> close_returned{false};
  test::ManualResetEvent close_started;
  test::ManualResetEvent close_completed;
  std::atomic<bool> output_existed_when_close_returned{true};
  Result<void> closed = Result<void>::Failure(Error::InvalidArgument("not run"));
  std::thread closing([&] {
    close_started.Signal();
    closed = service.Close();
    output_existed_when_close_returned.store(
        fs::exists(unpublished_output), std::memory_order_release);
    close_returned.store(true, std::memory_order_release);
    close_completed.Signal();
  });
  close_started.Wait("close invocation starts for unpublished output");
  Check(!close_completed.IsSignaled() &&
            !close_returned.load(std::memory_order_acquire),
        "close waits while output candidate is unpublished");
  latch->Release();
  opening.join();
  closing.join();

  Check(!opened && closed && !service.IsOpen(),
        "cancelled open does not publish a runtime");
  Check(!output_existed_when_close_returned.load(std::memory_order_acquire),
        "Close returns only after unpublished output rollback completes");
  Check(!fs::exists(unpublished_output),
        "cancelled open removes its unpublished runtime output");

  Check(open("reopen").IsOk(), "runtime reopens after cancelled output candidate");
  const auto published_output = latch->Output(1);
  Check(fs::is_regular_file(published_output / "candidate.marker"),
        "published runtime retains its output");
  Check(service.Close().IsOk(), "reopened output fixture closes");
  Check(fs::is_regular_file(published_output / "candidate.marker"),
        "closing a published runtime does not roll back its output");
  fs::remove_all(root);
}

void TestBuildFailureStillRollsBackOutput() {
  const auto root =
      fs::temp_directory_path() / "open_lmm_build_output_rollback";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  fs::path failed_output;
  RuntimeService service(
      1, [&failed_output](const BootstrapConfigSnapshot&,
                          const fs::path& output_directory)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        failed_output = output_directory;
        fs::create_directories(output_directory);
        std::ofstream(output_directory / "candidate.marker") << "created\n";
        return Result<std::shared_ptr<StageRuntimePort>>::Failure(
            Error::InvalidArgument("factory rejected candidate"));
      });
  const auto opened = service.Open({root / "config", "failure"});
  Check(!opened && !failed_output.empty() && !fs::exists(failed_output),
        "BuildInstance retains responsibility for failed factory output");
  fs::remove_all(root);
}

void TestConcurrentOpenReservesTransitionBeforeBootstrap() {
  const auto root = fs::temp_directory_path() / "open_lmm_single_runtime_open_race";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  auto latch = std::make_shared<OpenLatch>();
  RuntimeService service(
      1, [latch](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> { return latch->Create(); });
  Result<void> first = Result<void>::Failure(Error::InvalidArgument("not run"));
  std::thread opening([&] { first = service.Open({root / "config", "first"}); });
  latch->WaitForEntry();
  auto second = service.Open({root / "config", "second"});
  Check(!second && latch->Calls() == 1,
        "second open is rejected before it can bootstrap a candidate");
  latch->Release();
  opening.join();
  Check(first && service.IsOpen(), "reserved first open publishes exactly one runtime");
  Check(service.Close().IsOk(), "concurrent-open fixture closes");
  fs::remove_all(root);
}

void TestPublicJobRetentionIsBounded() {
  const auto root =
      fs::temp_directory_path() / "open_lmm_single_runtime_job_retention";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<SuccessPort>());
      });
  Check(service.Open({root / "config", "job-retention"}).IsOk(),
        "job retention fixture opens");
  ExecutionRequest request;
  request.kind = ExecutionRequestKind::kStage;
  request.stage = StageId::kDataLoad;
  std::optional<JobHandle> oldest;
  std::optional<JobHandle> newest;
  for (size_t i = 0; i < 10000; ++i) {
    auto submitted = service.Submit(request);
    Check(submitted.IsOk() && service.Wait(submitted.Value()).IsOk(),
          "long-running registry fixture completes each sequential job");
    if (!oldest) oldest = submitted.Value();
    newest = submitted.Value();
  }
  const auto expired = service.Wait(*oldest);
  Check(!expired &&
            expired.GetError().Message().find("unknown or expired") !=
                std::string::npos,
        "old terminal handle expires from the bounded registry");
  Check(service.Wait(*newest).IsOk(),
        "most recent terminal handle remains available");
  Check(newest->value > oldest->value,
        "public handles remain monotonic and are never reused");
  Check(service.Close().IsOk(), "job retention fixture closes");
  fs::remove_all(root);
}

void TestSynchronousPostCommitFailurePublishesFatalHealth() {
  const auto root =
      fs::temp_directory_path() / "open_lmm_post_commit_config_fault";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  auto port = std::make_shared<PostCommitConfigFaultPort>();
  RuntimeService service(
      1, [port](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(port);
      });
  Check(service.Open({root / "config", "post-commit-config"}).IsOk(),
        "post-commit config fixture opens");
  ConfigCandidate candidate;
  candidate.domain = ConfigDomain::kMapSave;
  candidate.document_json = "{}";
  auto applied = service.ApplyConfig(candidate, {1, 1});
  Check(!applied &&
            applied.GetError().severity == Error::Severity::kFatalRuntime,
        "advanced config authority without receipt is fatal");
  const auto snapshot = service.Snapshot();
  Check(snapshot && snapshot.Value().state == RuntimeStatus::kFailedFatal &&
            snapshot.Value().pipeline.runtime_revision == 2 &&
            snapshot.Value().pipeline.config_revision == 2,
        "public runtime snapshot exposes synchronous protocol failure");
  const auto revision = port->Snapshot().revision;
  auto retry = service.ApplyConfig(candidate, {2, 2});
  Check(!retry &&
            retry.GetError().severity == Error::Severity::kFatalRuntime &&
            port->Snapshot().revision == revision,
        "fatal protocol health rejects later mutation before port entry");
  Check(service.Close().IsOk(), "post-commit config fixture closes");
  fs::remove_all(root);
}

void TestCommittedRecoveryHealthIsPersistentAndGatesMutation() {
  const auto root =
      fs::temp_directory_path() / "open_lmm_committed_recovery_health";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<RecoveryPort>());
      });
  Check(service.Open({root / "config", "recovery"}).IsOk(),
        "recovery fixture opens healthy");
  auto job = service.Submit(
      {ExecutionRequestKind::kStage, StageId::kDataLoad});
  Check(job && service.Wait(job.Value()),
        "committed recovery stage remains a successful command");
  auto snapshot = service.Snapshot();
  bool structured_event = false;
  for (const auto& event : snapshot.Value().pipeline.recent_events) {
    structured_event = structured_event ||
                       (event.type == EventType::kStageCompleted &&
                        event.error &&
                        event.error->severity ==
                            Error::Severity::kFatalRuntime);
  }
  Check(snapshot && snapshot.Value().state == RuntimeStatus::kFailedFatal &&
            structured_event,
        "public snapshot and structured event expose recovery health");
  const auto rejected = service.Submit(
      {ExecutionRequestKind::kStage, StageId::kDataLoad});
  Check(!rejected && rejected.GetError().severity ==
                         Error::Severity::kFatalRuntime,
        "persistent recovery health rejects another mutation");
  ConfigCandidate config;
  config.domain = ConfigDomain::kMapSave;
  config.document_json = "{}";
  const auto config_rejected = service.ApplyConfig(config, {2, 1});
  Check(!config_rejected && config_rejected.GetError().severity ==
                                Error::Severity::kFatalRuntime,
        "persistent recovery health rejects config mutation");
  Check(service.NodeDescriptors().IsOk() && service.Snapshot().IsOk() &&
            service.Visualization(Id("agent")).IsOk(),
        "read-only queries remain available while recovery is required");
  Check(service.Close().IsOk(), "recovery fixture closes deterministically");
  fs::remove_all(root);
}

void TestCompoundRecoveryAndProtocolHealthRemainPublic() {
  const auto root = fs::temp_directory_path() /
                    "open_lmm_compound_recovery_protocol_health";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<RecoveryReceiptMismatchPort>());
      });
  Check(service.Open({root / "config", "compound-recovery"}).IsOk(),
        "compound recovery fixture opens");
  const auto job = service.Submit(
      {ExecutionRequestKind::kStage, StageId::kDataLoad});
  Check(job && !service.Wait(job.Value()),
        "recovery receipt mismatch fails the job");

  const auto snapshot = service.Snapshot();
  const auto exposes_both_causes = [](const Error& error) {
    return error.Message().find("/tmp/compound-recovery.json") !=
               std::string::npos &&
           error.Message().find("receipt recovery health does not match") !=
               std::string::npos;
  };
  Check(snapshot && snapshot.Value().state == RuntimeStatus::kFailedFatal &&
            std::any_of(snapshot.Value().pipeline.recent_events.begin(),
                        snapshot.Value().pipeline.recent_events.end(),
                        [&](const ExecutionEvent& event) {
                          return event.error &&
                                 event.error->code == Error::Code::kIoError &&
                                 exposes_both_causes(*event.error);
                        }),
        "public snapshot event preserves recovery and protocol causes");

  const auto rejected = service.Submit(
      {ExecutionRequestKind::kStage, StageId::kDataLoad});
  ConfigCandidate config;
  config.domain = ConfigDomain::kMapSave;
  config.document_json = "{}";
  const auto config_rejected = service.ApplyConfig(config, {2, 1});
  Check(!rejected && !config_rejected &&
            rejected.GetError().code == Error::Code::kIoError &&
            exposes_both_causes(rejected.GetError()) &&
            exposes_both_causes(config_rejected.GetError()),
        "compound health gates later mutations without hiding manifest");
  Check(service.Snapshot().IsOk() &&
            service.Visualization(Id("agent")).IsOk(),
        "compound fatal health keeps read-only queries available");
  Check(service.Close().IsOk(), "compound recovery fixture closes");
  fs::remove_all(root);
}

void TestRootReplacementPublishesCommittedRecoveryHealth() {
  const auto root =
      fs::temp_directory_path() / "open_lmm_root_recovery_health";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<SuccessPort>());
      });
  Check(service.Open({root / "config", "old"}).IsOk(),
        "root recovery fixture opens");
  const auto before = service.Snapshot().Value().pipeline;
  auto candidate = ReplacementCandidate(root / "config", root / "output",
                                        root / "replacement-output");
  fs::remove(root / "config/config.json");
  fs::create_directories(root / "config/config.json");
  std::ofstream(root / "config/config.json/retained")
      << "force backup cleanup failure\n";
  auto replaced = service.ReplaceRootConfig(
      {root / "config", "new"}, candidate,
      {before.runtime_revision, before.config_revision});
  auto snapshot = service.Snapshot();
  bool structured_event = false;
  for (const auto& event : snapshot.Value().pipeline.recent_events) {
    structured_event = structured_event ||
                       (event.error &&
                        event.error->severity ==
                            Error::Severity::kFatalRuntime &&
                        event.error->Message().find("recovery manifest") !=
                            std::string::npos);
  }
  Check(replaced && snapshot && snapshot.Value().label == "new" &&
            snapshot.Value().state == RuntimeStatus::kFailedFatal &&
            structured_event &&
            fs::is_regular_file(root / "config/config.json") &&
            fs::is_directory(
                root / "config/config.json.open_lmm_backup"),
        "root files and candidate runtime stay authoritative with recovery health");
  Check(!service.Submit(
            {ExecutionRequestKind::kStage, StageId::kDataLoad}),
        "root recovery health gates later pipeline mutation");

  fs::remove_all(root / "config/config.json.open_lmm_backup");
  for (const auto& entry : fs::directory_iterator(root / "config")) {
    if (entry.path().filename().string().find(".open_lmm_recovery_") == 0) {
      fs::remove(entry.path());
    }
  }
  const auto degraded = service.Snapshot().Value().pipeline;
  auto healthy_candidate = ReplacementCandidate(
      root / "config", root / "replacement-output", root / "healthy-output");
  auto healthy = service.ReplaceRootConfig(
      {root / "config", "healthy"}, healthy_candidate,
      {degraded.runtime_revision, degraded.config_revision});
  const auto recovered = service.Snapshot();
  auto accepted = service.Submit(
      {ExecutionRequestKind::kStage, StageId::kDataLoad});
  Check(healthy && recovered &&
            recovered.Value().state == RuntimeStatus::kReady && accepted &&
            service.Wait(accepted.Value()),
        "verified healthy replacement clears recovery health");
  Check(service.Close().IsOk(), "root recovery fixture closes");
  fs::remove_all(root);
}

}  // namespace

#ifndef OPEN_LMM_RUNTIME_SERVICE_SUITE
#define OPEN_LMM_RUNTIME_SERVICE_SUITE 0
#endif

int main() {
#if OPEN_LMM_RUNTIME_SERVICE_SUITE == 1
  TestSingleRuntimeLifecycleAndJobIdentity();
  TestHeadlessFeedbackAndDisableCancellation();
  TestReplacementAndCloseRejectBusyRuntime();
  TestPublicJobRetentionIsBounded();
#elif OPEN_LMM_RUNTIME_SERVICE_SUITE == 2
  TestSubscriptionResetBarrier();
  TestSubscriptionOutlivesRuntime();
  TestSubscriptionResetWaitsForCopiedCallback();
  TestOpenCloseTransitionCancellation();
  TestCancelledOpenRollsBackUnpublishedOutput(false);
  TestCancelledOpenRollsBackUnpublishedOutput(true);
  TestBuildFailureStillRollsBackOutput();
  TestConcurrentOpenReservesTransitionBeforeBootstrap();
  TestSynchronousPostCommitFailurePublishesFatalHealth();
  TestCommittedRecoveryHealthIsPersistentAndGatesMutation();
  TestCompoundRecoveryAndProtocolHealthRemainPublic();
  TestRootReplacementPublishesCommittedRecoveryHealth();
#else
#error "OPEN_LMM_RUNTIME_SERVICE_SUITE must select a layer suite"
#endif
  std::cout << "runtime service single-runtime tests passed\n";
  return 0;
}
