#include <open_lmm/server/runtime_service.hpp>

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <vector>

namespace {
using namespace open_lmm;
namespace fs = std::filesystem;

void Check(bool condition, const char* message) {
  if (condition) return;
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

class IsolatedRunner final : public StageRunner {
 public:
  IsolatedRunner(AgentId agent, bool block)
      : agent_(std::move(agent)), block_(block) {}

  void SetCancellationToken(std::shared_ptr<CancellationToken> token) override {
    cancellation_ = std::move(token);
  }
  Result<void> RunStage(StageId) override {
    entered = true;
    while (block_ && !released.load(std::memory_order_acquire)) {
      if (cancellation_ && cancellation_->IsCancellationRequested()) {
        exited = true;
        return Result<void>::Failure(
            Error::Cancelled("isolated session safe point"));
      }
      std::this_thread::yield();
    }
    exited = true;
    return Result<void>::Ok();
  }
  Result<void> RunNode(NodeId, std::optional<AgentId>) override {
    return RunStage(StageId::kDataLoad);
  }
  Result<void> RunOptimizeThrough(const AgentId&) override {
    return RunStage(StageId::kAlignment);
  }
  std::vector<AgentId> AgentIds() const override { return {agent_}; }

  std::atomic<bool> entered{false};
  std::atomic<bool> exited{false};
  std::atomic<bool> released{false};

 private:
  AgentId agent_;
  bool block_;
  std::shared_ptr<CancellationToken> cancellation_;
};

class FatalRunner final : public StageRunner {
 public:
  void SetCancellationToken(std::shared_ptr<CancellationToken>) override {}
  Result<void> RunStage(StageId) override {
    return Result<void>::Failure(
        Error::IoError("fatal session fixture").MarkFatalSession());
  }
  Result<void> RunNode(NodeId, std::optional<AgentId>) override {
    return RunStage(StageId::kDataLoad);
  }
  Result<void> RunOptimizeThrough(const AgentId&) override {
    return RunStage(StageId::kAlignment);
  }
  std::vector<AgentId> AgentIds() const override { return {Id("fatal-agent")}; }
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

void TestMultiSessionIsolationAndLifecycle() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_service_tests";
  fs::remove_all(root);
  const fs::path output_root = root / "output";
  WriteRootConfig(root / "config-one", output_root, "agent1");
  WriteRootConfig(root / "config-two", output_root, "agent2");

  std::mutex factory_mutex;
  std::vector<fs::path> outputs;
  std::map<std::string, std::shared_ptr<IsolatedRunner>> runners;
  RuntimeService service(
      2, [&](const BootstrapRequest& request, const fs::path& output)
             -> Result<std::shared_ptr<StageRunner>> {
        const bool first = request.label == "one";
        auto runner = std::make_shared<IsolatedRunner>(
            first ? Id("agent1") : Id("agent2"), true);
        std::lock_guard lock(factory_mutex);
        outputs.push_back(output);
        runners.emplace(request.label, runner);
        return Result<std::shared_ptr<StageRunner>>::Ok(runner);
      });

  auto one = service.CreateSession({root / "config-one", "one"});
  auto two = service.CreateSession({root / "config-two", "two"});
  Check(one && two && one.Value() != two.Value(),
        "two root configs create distinct opaque sessions");
  Check(SessionId::Parse(one.Value().Value()).IsOk() &&
            !SessionId::Parse("one"),
        "SessionId is a canonical 128-bit opaque ID");
  Check(service.Governor().ActiveSessions() == 2 &&
            service.Governor().MaximumSessions() == 2,
        "process-wide session admission is accounted");
  Check(!service.CreateSession({root / "config-two", "over-budget"}),
        "resource governor rejects over-budget session");
  {
    std::lock_guard lock(factory_mutex);
    Check(outputs.size() == 2 && outputs[0] != outputs[1] &&
              outputs[0].parent_path() == output_root &&
              outputs[1].parent_path() == output_root,
          "same output root receives collision-free session namespaces");
  }

  std::atomic<int> one_events{0};
  std::atomic<int> two_events{0};
  std::atomic<uint64_t> one_first_sequence{0};
  std::atomic<uint64_t> two_first_sequence{0};
  auto one_subscription = service.SubscribeEvents(
      one.Value(), [&](const SessionExecutionEvent& event) {
        Check(event.session_id == one.Value(), "session-one event namespace");
        uint64_t unset = 0;
        one_first_sequence.compare_exchange_strong(unset,
                                                   event.event.sequence);
        ++one_events;
        Check(service.Snapshot(event.session_id).IsOk(),
              "event callback re-enters its session snapshot");
      });
  auto two_subscription = service.SubscribeEvents(
      two.Value(), [&](const SessionExecutionEvent& event) {
        Check(event.session_id == two.Value(), "session-two event namespace");
        uint64_t unset = 0;
        two_first_sequence.compare_exchange_strong(unset,
                                                   event.event.sequence);
        ++two_events;
      });
  Check(one_subscription && two_subscription, "session subscriptions created");

  auto one_job = service.Submit(one.Value(), {});
  auto two_job = service.Submit(two.Value(), {});
  Check(one_job && two_job && one_job.Value() == 1 && two_job.Value() == 1,
        "job IDs are isolated per session");
  WaitUntil([&] { return runners.at("one")->entered.load() &&
                         runners.at("two")->entered.load(); },
            "both sessions run concurrently");
  Check(!service.Submit(one.Value(), {}),
        "same-session concurrent submission is rejected");
  Check(!service.CloseSession(one.Value(), CloseMode::kRejectIfRunning),
        "RejectIfRunning preserves active session");

  Check(service.Cancel(one.Value(), one_job.Value()).IsOk(),
        "session-one cancellation accepted");
  WaitUntil([&] { return runners.at("one")->exited.load(); },
            "session-one cancellation observed");
  Check(!runners.at("two")->exited.load(),
        "session-one cancellation does not affect session two");
  Check(service.CloseSession(one.Value(), CloseMode::kCancelAndWait).IsOk(),
        "cancelled session closes after join");
  Check(!service.Snapshot(one.Value()) &&
            service.Governor().ActiveSessions() == 1,
        "closed session is removed and admission released");

  runners.at("two")->released = true;
  WaitUntil([&] {
    auto snapshot = service.Snapshot(two.Value());
    return snapshot && snapshot.Value().state == RuntimeSessionState::kReady &&
           snapshot.Value().pipeline.job &&
           snapshot.Value().pipeline.job->state == JobState::kSucceeded;
  }, "session two completes independently");
  Check(service.CloseSession(two.Value(), CloseMode::kRejectIfRunning).IsOk(),
        "idle session closes without cancellation");
  Check(service.SessionIds().empty() &&
            service.Governor().ActiveSessions() == 0 &&
            one_events.load() > 0 && two_events.load() > 0 &&
            one_first_sequence.load() == 1 &&
            two_first_sequence.load() == 1,
        "registry, event namespaces, and governor quiesce");
  fs::remove_all(root);
}

void TestShutdownStressCancelsAndJoins() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_service_shutdown_tests";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "agent1");
  std::shared_ptr<IsolatedRunner> runner;
  {
    RuntimeService service(
        1, [&](const BootstrapRequest&, const fs::path&)
               -> Result<std::shared_ptr<StageRunner>> {
          runner = std::make_shared<IsolatedRunner>(Id("agent1"), true);
          return Result<std::shared_ptr<StageRunner>>::Ok(runner);
        });
    auto session = service.CreateSession({root / "config", "shutdown"});
    Check(session && service.Submit(session.Value(), {}),
          "shutdown stress job submitted");
    WaitUntil([&] { return runner->entered.load(); },
              "shutdown stress worker entered");
  }
  Check(runner->exited.load(),
        "RuntimeService destruction cancels and joins active workers");
  fs::remove_all(root);
}

void TestFatalStateAndCompletedClose() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_service_fatal_tests";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "fatal-agent");
  RuntimeService service(
      1, [](const BootstrapRequest&, const fs::path&)
             -> Result<std::shared_ptr<StageRunner>> {
        return Result<std::shared_ptr<StageRunner>>::Ok(
            std::make_shared<FatalRunner>());
      });
  auto session = service.CreateSession({root / "config", "fatal"});
  auto job = session ? service.Submit(session.Value(), {})
                     : Result<JobId>::Failure(
                           Error::InvalidArgument("session creation failed"));
  Check(session && job, "fatal fixture job submitted");
  WaitUntil([&] {
    auto snapshot = service.Snapshot(session.Value());
    return snapshot &&
           snapshot.Value().state == RuntimeSessionState::kFailedFatal;
  }, "fatal execution is distinguished from recoverable failure");
  Check(!service.Submit(session.Value(), {}),
        "fatal session rejects a new submission");
  Check(service.CloseSession(session.Value(), CloseMode::kRejectIfRunning).IsOk(),
        "completed fatal job closes without an active-job false positive");
  fs::remove_all(root);
}

}  // namespace

int main() {
  TestMultiSessionIsolationAndLifecycle();
  TestShutdownStressCancelsAndJoins();
  TestFatalStateAndCompletedClose();
  std::cout << "runtime service multi-session tests passed\n";
  return 0;
}
