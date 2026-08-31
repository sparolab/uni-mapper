#include <open_lmm/server/runtime_client.hpp>
#include <open_lmm/server/runtime_service.hpp>

#include "test_runtime_port.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <thread>

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
    while (!context.cancellation->IsCancellationRequested()) {
      std::this_thread::yield();
    }
    exited.store(true, std::memory_order_release);
    return Result<void>::Failure(Error::Cancelled("blocking fixture cancelled"));
  }

  std::atomic<bool> entered{false};
  std::atomic<bool> exited{false};
};

class SuccessPort final : public test::RuntimePortFixture {
 public:
  SuccessPort() : RuntimePortFixture({Id("agent")}) {}
  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    return Result<void>::Ok();
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

template <typename Predicate>
void WaitUntil(Predicate predicate, const char* message) {
  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::seconds(2);
  while (!predicate() && std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  Check(predicate(), message);
}

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
  auto subscribed = service.SubscribeEvents([&](const ExecutionEvent& event) {
    if (event.type == EventType::kJobQueued) queued.store(event.job_id);
  });
  Check(subscribed.IsOk(), "subscribe to unkeyed events");
  auto subscription = std::move(subscribed).Value();

  ExecutionRequest one_stage;
  one_stage.kind = ExecutionRequestKind::kStage;
  one_stage.stage = StageId::kDataLoad;
  auto first = service.Submit(one_stage);
  Check(first && first.Value().value != 0, "submit returns opaque job handle");
  WaitUntil([&] { return queued.load() != 0; }, "queued event is delivered");
  Check(queued.load() == first.Value().value,
        "queued event and submit share the public job handle");
  WaitUntil([&] {
    auto feedback = service.AlignmentFeedback();
    return feedback && feedback.Value().has_value();
  }, "feedback is visible from the sole runtime");
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
  auto second = service.Submit(one_stage);
  Check(second && second.Value().value != first.Value().value,
        "handles are never reused after controller replacement");
  Check(!service.Wait(first.Value()), "retired handle is rejected by new epoch");
  WaitUntil([&] {
    auto current = service.AlignmentFeedback();
    return current && current.Value().has_value();
  }, "replacement owns a new feedback channel");
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
  WaitUntil([&] {
    auto feedback = service.AlignmentFeedback();
    return feedback && feedback.Value().has_value();
  }, "interactive feedback request becomes visible");
  Check(service.SetAlignmentFeedbackEnabled(false).IsOk(),
        "disabling authority cancels the active request");
  Check(!service.Wait(interactive.Value()),
        "cancelled feedback request terminates its waiting command");
  const auto terminal = service.AlignmentFeedback().Value();
  Check(terminal &&
            terminal->review_state == AlignmentReviewState::kCancelled &&
            !terminal->terminal_message.empty(),
        "disabled authority preserves a terminal read-only review");
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
  WaitUntil([&] { return port->entered.load(std::memory_order_acquire); },
            "blocking job entered");
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
  WaitUntil([&] { return port->exited.load(std::memory_order_acquire); },
            "cancelled worker exits without close/wait deadlock");
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
  std::atomic<bool> callback_entered{false};
  std::atomic<bool> release_callback{false};
  std::atomic<bool> reset_returned{false};
  std::atomic<unsigned> calls{0};
  auto subscribed = service.SubscribeEvents([&](const ExecutionEvent&) {
    const auto call = calls.fetch_add(1, std::memory_order_relaxed);
    if (call != 0) return;
    callback_entered.store(true, std::memory_order_release);
    while (!release_callback.load(std::memory_order_acquire)) {
      std::this_thread::yield();
    }
  });
  Check(subscribed.IsOk(), "subscription barrier fixture subscribes");
  auto subscription = std::move(subscribed).Value();
  Result<JobHandle> job = Result<JobHandle>::Failure(Error::InvalidArgument("not run"));
  std::thread submitter([&] {
    job = service.Submit({ExecutionRequestKind::kStage, StageId::kDataLoad});
  });
  WaitUntil([&] { return callback_entered.load(std::memory_order_acquire); },
            "first callback was copied and entered");
  std::thread resetter([&] {
    subscription.Reset();
    reset_returned.store(true, std::memory_order_release);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  Check(!reset_returned.load(std::memory_order_acquire),
        "subscription reset waits for an already copied callback");
  release_callback.store(true, std::memory_order_release);
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
  Result<void> closed = Result<void>::Failure(Error::InvalidArgument("not run"));
  std::thread closing([&] {
    closed = service.Close();
    close_returned.store(true, std::memory_order_release);
  });
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  Check(!close_returned.load(std::memory_order_acquire),
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

}  // namespace

int main() {
  TestSingleRuntimeLifecycleAndJobIdentity();
  TestHeadlessFeedbackAndDisableCancellation();
  TestReplacementAndCloseRejectBusyRuntime();
  TestSubscriptionResetBarrier();
  TestSubscriptionOutlivesRuntime();
  TestSubscriptionResetWaitsForCopiedCallback();
  TestOpenCloseTransitionCancellation();
  TestConcurrentOpenReservesTransitionBeforeBootstrap();
  TestPublicJobRetentionIsBounded();
  std::cout << "runtime service single-runtime tests passed\n";
  return 0;
}
