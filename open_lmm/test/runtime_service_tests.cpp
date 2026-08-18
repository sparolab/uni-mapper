#include <open_lmm/server/runtime_service.hpp>
#include "test_runtime_port.hpp"

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

void WriteDefaultRuntimeFixture(const fs::path& config_directory,
                                const fs::path& data_root,
                                const fs::path& output_root) {
  const fs::path agent = data_root / "agent1";
  fs::create_directories(agent / "Scans");
  {
    std::ofstream pose(agent / "poses.txt");
    pose << "1 0 0 0 0 1 0 0 0 0 1 0\n";
  }
  {
    std::ofstream scan(agent / "Scans/000000.pcd");
    scan << "# .PCD v0.7\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity\n"
         << "SIZE 4 4 4 4\n"
         << "TYPE F F F F\n"
         << "COUNT 1 1 1 1\n"
         << "WIDTH 1\nHEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS 1\nDATA ascii\n"
         << "10 0 0 1\n";
  }
  fs::create_directories(config_directory / "server");
  fs::create_directories(config_directory / "core");
  {
    std::ofstream root(config_directory / "config.json");
    root << "{\n"
         << "  \"global\": {\n"
         << "    \"config_map_server\": \"server/map.json\",\n"
         << "    \"config_data_loader\": \"core/data.json\",\n"
         << "    \"config_loop_detector\": \"core/loop.json\",\n"
         << "    \"config_backend_optimizer\": \"core/optimizer.json\",\n"
         << "    \"config_dynamic_remover\": \"core/remover.json\"\n"
         << "  },\n"
         << "  \"directory\": {\n"
         << "    \"root_dir_path\": \"" << data_root.string() << "\",\n"
         << "    \"sub_dir_list\": [\"agent1\"],\n"
         << "    \"root_save_dir\": \"" << output_root.string() << "\"\n"
         << "  }\n"
         << "}\n";
  }
  {
    std::ofstream map(config_directory / "server/map.json");
    map << "{\"map_server\":{"
        << "\"enable_map_updater\":false,"
        << "\"anchor_agent_index\":0,"
        << "\"save_voxel_size\":0.2,"
        << "\"parallel_data_load\":false,"
        << "\"parallel_map_update\":false,"
        << "\"max_parallel_agents\":1}}\n";
  }
  {
    std::ofstream data(config_directory / "core/data.json");
    data << "{\"data_loader\":{"
         << "\"data_loader_type\":\"file_based\","
         << "\"pose_format\":\"kitti\","
         << "\"pose_file_name\":\"poses.txt\","
         << "\"extrinsic\":[0,0,0,0,0,0,1],"
         << "\"scan_type\":\"pcd\","
         << "\"scan_dir_name\":\"Scans\","
         << "\"voxel_size\":0.5,"
         << "\"min_range\":1.0,"
         << "\"max_range\":60.0,"
         << "\"delimiter\":\" \"}}\n";
  }
  {
    std::ofstream loop(config_directory / "core/loop.json");
    loop << "{"
         << "\"loop_detector\":{\"loop_detector_type\":\"kdtree\","
         << "\"plugin_abi\":\"auto\",\"model\":\"scan_context\"},"
         << "\"database\":{\"descriptor_vector_dim\":20,"
         << "\"distance_threshold\":0.15,\"num_candidates\":3,"
         << "\"rebuild_threshold\":50},"
         << "\"alignment\":{\"pcm_translation_threshold\":10.0,"
         << "\"pcm_rotation_threshold_deg\":20.0,"
         << "\"pcm_solver\":\"heuristic\",\"pcm_threads\":1,"
         << "\"pcm_max_candidates\":0}}\n";
  }
  {
    std::ofstream optimizer(config_directory / "core/optimizer.json");
    optimizer << "{\"backend_optimizer\":{"
              << "\"backend_optimizer_type\":\"incremental\","
              << "\"relinearizeThreshold\":0.1,"
              << "\"relinearizeSkip\":1,"
              << "\"isam_extra_updates\":1,"
              << "\"min_loop_frame_gap\":30,"
              << "\"icp_search_num\":1}}\n";
  }
  {
    std::ofstream remover(config_directory / "core/remover.json");
    remover << "{\"dynamic_remover\":{"
            << "\"dynamic_remover_type\":\"offline\","
            << "\"model\":\"free_dom\"}}\n";
  }
}

class IsolatedRunner final : public test::RuntimePortFixture {
 public:
  IsolatedRunner(AgentId agent, bool block)
      : RuntimePortFixture({agent}), block_(block) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext& context) override {
    cancellation_ = context.cancellation;
    return Run();
  }
  Result<void> Run() {
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

  std::atomic<bool> entered{false};
  std::atomic<bool> exited{false};
  std::atomic<bool> released{false};

 private:
  bool block_;
  std::shared_ptr<CancellationToken> cancellation_;
};

class FatalRunner final : public test::RuntimePortFixture {
 public:
  FatalRunner() : RuntimePortFixture({Id("fatal-agent")}) {}
  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    return Result<void>::Failure(
        Error::IoError("fatal session fixture").MarkFatalSession());
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
             -> Result<std::shared_ptr<StageRuntimePort>> {
        const bool first = request.label == "one";
        auto runner = std::make_shared<IsolatedRunner>(
            first ? Id("agent1") : Id("agent2"), true);
        std::lock_guard lock(factory_mutex);
        outputs.push_back(output);
        runners.emplace(request.label, runner);
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(runner);
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
  std::atomic<bool> callback_close_attempted{false};
  std::atomic<bool> callback_close_rejected{false};
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
        if (event.event.type == EventType::kJobCompleted ||
            event.event.type == EventType::kJobCancelled) {
          callback_close_rejected =
              !service.CloseSession(event.session_id,
                                    CloseMode::kRejectIfRunning);
          callback_close_attempted = true;
        }
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
  WaitUntil([&] { return callback_close_attempted.load(); },
            "terminal callback close attempt returns without deadlock");
  Check(callback_close_rejected.load() && service.Snapshot(two.Value()),
        "callback session close is safely rejected without removing session");
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
               -> Result<std::shared_ptr<StageRuntimePort>> {
          runner = std::make_shared<IsolatedRunner>(Id("agent1"), true);
          return Result<std::shared_ptr<StageRuntimePort>>::Ok(runner);
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
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
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

void TestDefaultCloseReleasesResidentReservation() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_service_resident_close_tests";
  fs::remove_all(root);
  WriteDefaultRuntimeFixture(root / "config", root / "data", root / "output");
  RuntimeService service(ResourceBudget{1, 1, 1, 64ULL * 1024ULL * 1024ULL});
  auto session = service.CreateSession({root / "config", "resident-close"});
  Check(session.IsOk(), "default RuntimeService session bootstraps");
  ExecutionRequest request;
  request.kind = ExecutionRequestKind::kStage;
  request.stage = StageId::kDataLoad;
  auto job = service.Submit(session.Value(), request);
  Check(job.IsOk(), "default RuntimeService DataLoad submitted");
  WaitUntil(
      [&] {
        auto snapshot = service.Snapshot(session.Value());
        return snapshot && snapshot.Value().state == RuntimeSessionState::kReady &&
               snapshot.Value().pipeline.job &&
               snapshot.Value().pipeline.job->state == JobState::kSucceeded;
      },
      "default RuntimeService DataLoad completed");
  Check(service.Governor().ReservedMemoryBytes(
            MemoryClass::kResidentPayload) > 0,
        "committed runtime session owns resident memory");
  Check(service.CloseSession(session.Value(), CloseMode::kRejectIfRunning).IsOk(),
        "default RuntimeService session closes after DataLoad");
  Check(service.Governor().ActiveSessions() == 0 &&
            service.Governor().ReservedMemoryBytes() == 0 &&
            service.Governor().ReservedMemoryBytes(
                MemoryClass::kResidentPayload) == 0,
        "CloseSession releases the default port resident reservation");
  fs::remove_all(root);
}

}  // namespace

int main() {
  TestMultiSessionIsolationAndLifecycle();
  TestShutdownStressCancelsAndJoins();
  TestFatalStateAndCompletedClose();
  TestDefaultCloseReleasesResidentReservation();
  std::cout << "runtime service multi-session tests passed\n";
  return 0;
}
