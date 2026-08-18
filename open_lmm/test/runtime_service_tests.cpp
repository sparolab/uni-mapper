#include <open_lmm/server/runtime_client.hpp>
#include <open_lmm/server/runtime_service.hpp>

#include "test_runtime_port.hpp"

#include <atomic>
#include <chrono>
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
    AlignmentFeedbackSnapshot request;
    request.proposal.target_agent = Id("agent");
    request.proposal.source_agent = Id("agent");
    auto response = context.alignment_feedback->Request(std::move(request),
                                                        context.cancellation);
    return response ? Result<void>::Ok()
                    : Result<void>::Failure(response.GetError());
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
                            AlignmentDecision::kAccept, std::nullopt}) &&
            service.Wait(first.Value()),
        "feedback and wait target the same unkeyed job handle");

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
                             AlignmentDecision::kAccept, std::nullopt}) &&
            service.Wait(second.Value()),
        "new runtime executes through the same public API");
  subscription.Reset();
  Check(service.Close().IsOk() && !service.IsOpen(), "close retires sole runtime");
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

}  // namespace

int main() {
  TestSingleRuntimeLifecycleAndJobIdentity();
  TestReplacementAndCloseRejectBusyRuntime();
  TestSubscriptionResetBarrier();
  std::cout << "runtime service single-runtime tests passed\n";
  return 0;
}
