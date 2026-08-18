#include <open_lmm/server/runtime_service.hpp>
#include <open_lmm/server/runtime_client.hpp>
#include "test_runtime_port.hpp"

#include <atomic>
#include <chrono>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <mutex>
#include <thread>
#include <type_traits>
#include <utility>
#include <vector>

namespace {
using namespace open_lmm;
namespace fs = std::filesystem;

static_assert(!std::is_aggregate_v<BootstrapConfigSnapshot>);
static_assert(std::is_same_v<
              decltype(std::declval<const BootstrapConfigSnapshot&>()
                           .OutputRoot()),
              const fs::path&>);

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

class InteractiveRunner final : public test::RuntimePortFixture {
 public:
  InteractiveRunner() : RuntimePortFixture({Id("interactive")}) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext& context) override {
    AlignmentFeedbackSnapshot request;
    request.proposal.target_agent = Id("interactive");
    request.proposal.source_agent = Id("interactive");
    auto response = context.alignment_feedback->Request(
        std::move(request), context.cancellation);
    if (!response) return Result<void>::Failure(response.GetError());
    return Result<void>::Ok();
  }

  Result<VisualizationSnapshot> CreateVisualization(
      const AgentId& agent) const override {
    VisualizationSnapshot result;
    result.agent = agent;
    result.revision = 77;
    result.map_available = true;
    result.points.push_back({1.0F, 2.0F, 3.0F, 4.0F});
    return Result<VisualizationSnapshot>::Ok(std::move(result));
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
      2, [&](const BootstrapConfigSnapshot& bootstrap, const fs::path& output)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        const bool first =
            bootstrap.ConfigDirectory().filename() == "config-one";
        auto runner = std::make_shared<IsolatedRunner>(
            first ? Id("agent1") : Id("agent2"), true);
        std::lock_guard lock(factory_mutex);
        outputs.push_back(output);
        runners.emplace(first ? "one" : "two", runner);
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

void TestBootstrapConfigIsAnImmutableValueSnapshot() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_bootstrap_snapshot_tests";
  fs::remove_all(root);
  const fs::path config = root / "config";
  const fs::path first_output = root / "output-first";
  const fs::path second_output = root / "output-second";
  WriteRootConfig(config, first_output, "agent1");

  auto loaded = LoadBootstrapConfig(config);
  Check(loaded && loaded.Value().ConfigDirectory() == config &&
            loaded.Value().OutputRoot() == first_output &&
            loaded.Value().DataSubdirectories() ==
                std::vector<std::string>{"agent1"},
        "bootstrap loader returns the resolved validated value");

  WriteRootConfig(config, second_output, "agent2");
  Check(loaded.Value().OutputRoot() == first_output &&
            loaded.Value().DataSubdirectories() ==
                std::vector<std::string>{"agent1"},
        "bootstrap snapshot does not observe later filesystem changes");
  Check(!LoadBootstrapConfig({}) && !LoadBootstrapConfig(root / "missing"),
        "bootstrap loader rejects empty and missing roots as Results");

  const fs::path oversized = root / "oversized";
  fs::create_directories(oversized);
  {
    std::ofstream output(oversized / "config.json", std::ios::binary);
    output << std::string(SchemaLimits{}.maximum_document_bytes + 1U, ' ');
  }
  auto oversized_result = LoadBootstrapConfig(oversized);
  Check(!oversized_result &&
            oversized_result.GetError().code == Error::Code::kInvalidArgument &&
            oversized_result.GetError().message.find("byte limit") !=
                std::string::npos,
        "bootstrap root is bounded before JSON parsing");

  const fs::path relative_config = root / "relative-config";
  fs::create_directories(relative_config);
  {
    std::ofstream output(relative_config / "config.json");
    output << R"({
      "global": {
        "config_map_server": "modules/map.json",
        "config_data_loader": "modules/loader.json",
        "config_loop_detector": "modules/loop.json",
        "config_backend_optimizer": "modules/optimizer.json",
        "config_dynamic_remover": "modules/remover.json"
      },
      "directory": {
        "root_dir_path": "data",
        "sub_dir_list": ["agent-relative"],
        "root_save_dir": "output"
      }
    })";
  }
  const fs::path original_cwd = fs::current_path();
  fs::current_path(root);
  auto relative = LoadBootstrapConfig("relative-config");
  fs::current_path(original_cwd);
  const auto expected_config = fs::weakly_canonical(relative_config);
  Check(relative && relative.Value().ConfigDirectory() == expected_config &&
            relative.Value().MapServerConfig() ==
                expected_config / "modules/map.json" &&
            relative.Value().DataRoot() == expected_config / "data" &&
            relative.Value().OutputRoot() == expected_config / "output" &&
            relative.Value().ConfigDirectory().is_absolute() &&
            relative.Value().MapServerConfig().is_absolute() &&
            relative.Value().DataRoot().is_absolute() &&
            relative.Value().OutputRoot().is_absolute(),
        "relative bootstrap paths resolve once against the config directory");
  fs::current_path(fs::temp_directory_path());
  Check(relative.Value().MapServerConfig() ==
            expected_config / "modules/map.json" &&
            relative.Value().DataRoot() == expected_config / "data" &&
            relative.Value().OutputRoot() == expected_config / "output",
        "bootstrap path values are invariant under later CWD changes");
  fs::current_path(original_cwd);
  fs::remove_all(root);
}

void TestRuntimeServiceExposesSessionScopedControlPlane() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_control_plane_tests";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "interactive");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<InteractiveRunner>());
      });
  auto session = service.CreateSession({root / "config", "control-plane"});
  Check(session.IsOk(), "control-plane session created");
  const SessionId id = session.Value();
  auto descriptors = service.NodeDescriptors(id);
  Check(descriptors && !descriptors.Value().empty(),
        "node descriptors are session-keyed");
  auto visualization = service.VisualizationSnapshot(id, Id("interactive"));
  Check(visualization && visualization.Value().revision == 77 &&
            visualization.Value().map_available &&
            visualization.Value().points.size() == 1,
        "visualization is queried through the session facade");
  Check(service.SetAlignmentFeedbackEnabled(id, true).IsOk(),
        "session alignment feedback is enabled through the facade");
  auto before_config = service.Snapshot(id);
  Check(before_config && before_config.Value().pipeline.session_revision == 1 &&
            before_config.Value().pipeline.config_revision == 1,
        "runtime snapshot exposes both optimistic concurrency revisions");
  ConfigCandidate candidate;
  candidate.domain = ConfigDomain::kMapSave;
  candidate.document_json = "{}";
  auto applied = service.ApplyConfig(id, candidate, {1, 1});
  Check(applied && applied.Value().base_session_revision == 1 &&
            applied.Value().session_revision == 2 &&
            applied.Value().previous_config_revision == 1 &&
            applied.Value().config_revision == 2,
        "config candidate receipt crosses the session facade unchanged");
  auto after_config = service.Snapshot(id);
  Check(after_config && after_config.Value().pipeline.session_revision == 2 &&
            after_config.Value().pipeline.config_revision == 2 &&
            !service.ApplyConfig(id, candidate, {1, 1}),
        "committed revisions advance and stale config candidates are rejected");

  ExecutionRequest request;
  request.kind = ExecutionRequestKind::kNode;
  request.node = NodeId::kDataLoad;
  request.agent = Id("interactive");
  auto job = service.Submit(id, request);
  Check(job.IsOk(), "interactive node submitted");
  WaitUntil([&] {
    auto feedback = service.AlignmentFeedbackSnapshot(id);
    return feedback && feedback.Value().has_value();
  }, "published alignment feedback is visible through its session");
  auto feedback = service.AlignmentFeedbackSnapshot(id);
  Check(feedback && feedback.Value(), "alignment feedback snapshot acquired");
  const uint64_t request_id = feedback.Value()->proposal.request_id;
  Check(service.RespondToAlignment(
            id, job.Value(),
            {request_id, AlignmentDecision::kAccept, std::nullopt})
            .IsOk(),
        "alignment response is routed to the owning session");
  Check(service.Wait(id, job.Value()).IsOk(),
        "runtime wait observes the terminal journal barrier");
  auto cleared = service.AlignmentFeedbackSnapshot(id);
  Check(cleared && !cleared.Value(),
        "completed alignment feedback is no longer exposed");
  Check(!service.RespondToAlignment(
            id, job.Value(),
            {request_id, AlignmentDecision::kAccept, std::nullopt}),
        "stale alignment response is rejected");
  Check(service.CloseSession(id, CloseMode::kRejectIfRunning).IsOk(),
        "control-plane session closes");
  Check(!service.Wait(id, job.Value()) && !service.NodeDescriptors(id) &&
            !service.VisualizationSnapshot(id, Id("interactive")) &&
            !service.AlignmentFeedbackSnapshot(id) &&
            !service.SubscribeEvents(id, [](const SessionExecutionEvent&) {}),
        "all control-plane methods reject a retired session id");

  std::ifstream root_input(root / "config/config.json");
  std::string root_json((std::istreambuf_iterator<char>(root_input)),
                        std::istreambuf_iterator<char>());
  const std::string disk_output = (root / "output").string();
  const std::string candidate_output = (root / "candidate-output").string();
  const auto output_position = root_json.find(disk_output);
  Check(output_position != std::string::npos,
        "root candidate fixture locates the disk output path");
  root_json.replace(output_position, disk_output.size(), candidate_output);
  ConfigCandidate root_candidate;
  root_candidate.domain = ConfigDomain::kGlobal;
  root_candidate.document_json = std::move(root_json);
  auto candidate_session = service.CreateSession(
      {root / "config", "candidate-session"}, root_candidate);
  auto candidate_snapshot = candidate_session
      ? service.Snapshot(candidate_session.Value())
      : Result<RuntimeSessionSnapshot>::Failure(
            Error::InvalidArgument("candidate session creation failed"));
  Check(candidate_session && candidate_snapshot &&
            candidate_snapshot.Value().output_directory.parent_path() ==
                root / "candidate-output",
        "in-memory root candidate creates a replacement session without a pre-write");
  Check(service.CloseSession(candidate_session.Value(),
                             CloseMode::kRejectIfRunning).IsOk(),
        "candidate-root session closes");
  fs::remove_all(root);
}

std::string ReadText(const fs::path& path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input),
          std::istreambuf_iterator<char>()};
}

void TestAtomicReplacementTransfersAdmissionAndPersistsRoot() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_atomic_replacement_tests";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output-one", "interactive");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path& output)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        fs::create_directories(output);
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<InteractiveRunner>());
      });
  auto original = service.CreateSession({root / "config", "original"});
  Check(original.IsOk() && service.Governor().ActiveSessions() == 1,
        "single admitted source session is ready");
  std::string candidate_json = ReadText(root / "config/config.json");
  const auto old_output = (root / "output-one").string();
  const auto new_output = (root / "output-two").string();
  const auto output_position = candidate_json.find(old_output);
  Check(output_position != std::string::npos,
        "replacement fixture locates output root");
  candidate_json.replace(output_position, old_output.size(), new_output);
  ConfigCandidate candidate;
  candidate.domain = ConfigDomain::kGlobal;
  candidate.document_json = candidate_json;
  auto replaced = service.ReplaceSession(
      original.Value(), {root / "config", "replacement"}, candidate,
      [](const SessionExecutionEvent&) {});
  Check(replaced && replaced.Value().session_id != original.Value() &&
            service.Governor().ActiveSessions() == 1 &&
            !service.Snapshot(original.Value()) &&
            service.Snapshot(replaced.Value().session_id) &&
            ReadText(root / "config/config.json").find(new_output) !=
                std::string::npos,
        "replacement transfers one admission and installs canonical root");
  Check(service.CloseSession(replaced.Value().session_id,
                             CloseMode::kRejectIfRunning).IsOk() &&
            service.Governor().ActiveSessions() == 0,
        "replacement admission is released exactly once");
  fs::remove_all(root);
}

void TestReplacementInstallFailurePreservesOldSessionAndOutput() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_replacement_rollback_tests";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output-old", "interactive");
  RuntimeService service(
      1, [](const BootstrapConfigSnapshot&, const fs::path& output)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        fs::create_directories(output);
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<InteractiveRunner>());
      });
  auto original = service.CreateSession({root / "config", "original"});
  const std::string original_json = ReadText(root / "config/config.json");
  {
    std::ofstream stale_backup(
        root / "config/config.json.open_lmm_backup");
    stale_backup << "blocks replacement commit";
  }
  std::string candidate_json = original_json;
  const auto old_output = (root / "output-old").string();
  const auto failed_output = (root / "output-failed").string();
  candidate_json.replace(candidate_json.find(old_output), old_output.size(),
                         failed_output);
  ConfigCandidate candidate;
  candidate.domain = ConfigDomain::kGlobal;
  candidate.document_json = std::move(candidate_json);
  auto replaced = service.ReplaceSession(
      original.Value(), {root / "config", "failed"}, candidate,
      [](const SessionExecutionEvent&) {});
  std::size_t orphan_outputs = 0;
  if (fs::exists(root / "output-failed")) {
    for (const auto& entry : fs::directory_iterator(root / "output-failed")) {
      if (entry.path().filename().string().rfind("runtime-session-", 0) == 0) {
        ++orphan_outputs;
      }
    }
  }
  Check(!replaced && service.Snapshot(original.Value()) &&
            service.Governor().ActiveSessions() == 1 &&
            ReadText(root / "config/config.json") == original_json &&
            orphan_outputs == 0,
        "install failure preserves old disk/session and removes prepared output");
  Check(service.CloseSession(original.Value(), CloseMode::kRejectIfRunning)
            .IsOk(),
        "rollback source session remains closable");
  fs::remove_all(root);
}

void TestReplacementRejectsAnActiveJobAndWaitOperation() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_replacement_active_tests";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "active-agent");
  std::shared_ptr<IsolatedRunner> runner;
  RuntimeService service(
      1, [&](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        runner = std::make_shared<IsolatedRunner>(Id("active-agent"), true);
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(runner);
      });
  auto original = service.CreateSession({root / "config", "active"});
  auto job = original ? service.Submit(original.Value(), ExecutionRequest{})
                      : Result<JobId>::Failure(
                            Error::InvalidArgument("missing session"));
  Check(job.IsOk(), "active replacement fixture submits a blocking job");
  WaitUntil([&] { return runner && runner->entered.load(); },
            "active replacement fixture enters the command");

  std::atomic<bool> wait_started{false};
  Result<void> waited = Result<void>::Failure(
      Error::InvalidArgument("wait not started"));
  std::thread waiter([&] {
    wait_started = true;
    waited = service.Wait(original.Value(), job.Value());
  });
  WaitUntil([&] { return wait_started.load(); },
            "active replacement fixture starts Wait");

  ConfigCandidate candidate;
  candidate.domain = ConfigDomain::kGlobal;
  candidate.document_json = ReadText(root / "config/config.json");
  auto replaced = service.ReplaceSession(
      original.Value(), {root / "config", "rejected"}, candidate,
      [](const SessionExecutionEvent&) {});
  Check(!replaced && service.Snapshot(original.Value()) &&
            service.Governor().ActiveSessions() == 1,
        "replacement is rejected while a job and Wait operation are active");

  Check(service.Cancel(original.Value(), job.Value()).IsOk(),
        "active replacement fixture cancels its blocking job");
  waiter.join();
  Check((waited.IsOk() ||
         waited.GetError().code == Error::Code::kCancelled) &&
        service.CloseSession(original.Value(), CloseMode::kRejectIfRunning)
            .IsOk(),
        "rejected replacement leaves the original session usable");
  fs::remove_all(root);
}

void TestReplacementBlocksConcurrentMutatingCommands() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_replacement_command_race_tests";
  fs::remove_all(root);
  WriteRootConfig(root / "config", root / "output", "race-agent");
  std::atomic<int> factory_calls{0};
  std::atomic<bool> replacement_factory_entered{false};
  std::atomic<bool> release_replacement_factory{false};
  RuntimeService service(
      1, [&](const BootstrapConfigSnapshot&, const fs::path&)
             -> Result<std::shared_ptr<StageRuntimePort>> {
        if (++factory_calls == 2) {
          replacement_factory_entered = true;
          while (!release_replacement_factory.load(std::memory_order_acquire)) {
            std::this_thread::yield();
          }
        }
        return Result<std::shared_ptr<StageRuntimePort>>::Ok(
            std::make_shared<InteractiveRunner>());
      });
  auto original = service.CreateSession({root / "config", "race-source"});
  Check(original.IsOk(), "replacement command race source is created");
  ConfigCandidate root_candidate;
  root_candidate.domain = ConfigDomain::kGlobal;
  root_candidate.document_json = ReadText(root / "config/config.json");
  Result<RuntimeSessionReplacement> replacement =
      Result<RuntimeSessionReplacement>::Failure(
          Error::InvalidArgument("replacement not started"));
  std::thread replace_thread([&] {
    replacement = service.ReplaceSession(
        original.Value(), {root / "config", "race-replacement"},
        root_candidate, [](const SessionExecutionEvent&) {});
  });
  WaitUntil([&] { return replacement_factory_entered.load(); },
            "replacement command race reaches the prepared-port boundary");

  ConfigCandidate config_candidate;
  config_candidate.domain = ConfigDomain::kLoopDetector;
  config_candidate.document_json = "{}";
  Check(!service.Submit(original.Value(), ExecutionRequest{}) &&
            !service.ApplyConfig(original.Value(), config_candidate,
                                 ExpectedRevision{}),
        "Submit and ApplyConfig reject the source while replacement prepares");

  release_replacement_factory.store(true, std::memory_order_release);
  replace_thread.join();
  Check(replacement.IsOk() && !service.Snapshot(original.Value()) &&
            service.CloseSession(replacement.Value().session_id,
                                 CloseMode::kRejectIfRunning)
                .IsOk(),
        "command exclusion preserves a successful atomic replacement");
  fs::remove_all(root);
}

void TestLastRuntimeClientOwnerCanBeReleasedFromTerminalCallback() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_runtime_callback_owner_tests";
  fs::remove_all(root);
  WriteDefaultRuntimeFixture(root / "config", root / "data", root / "output");
  auto runtime = std::make_shared<RuntimeClient>(1);
  std::weak_ptr<RuntimeClient> weak_runtime = runtime;
  std::atomic<bool> submit_returned{false};
  auto session = runtime->CreateSession({root / "config", "callback-owner"});
  Check(session.IsOk(), "callback-owner session is created");
  std::atomic<bool> owner_released{false};
  auto subscribed = runtime->SubscribeEvents(
      session.Value(), [&](const SessionExecutionEvent& event) {
        if (event.event.type != EventType::kJobCompleted &&
            event.event.type != EventType::kJobCancelled) {
          return;
        }
        // Keep the last-owner release out of the Submit() call itself. A very
        // fast fixture may otherwise complete before Submit returns, which
        // would test destruction of an object with an active member call
        // rather than the callback-thread teardown boundary.
        while (!submit_returned.load(std::memory_order_acquire)) {
          std::this_thread::yield();
        }
        runtime.reset();
        owner_released = true;
      });
  Check(subscribed.IsOk(), "callback-owner terminal observer is installed");
  auto subscription = std::move(subscribed).Value();
  ExecutionRequest request;
  request.kind = ExecutionRequestKind::kStage;
  request.stage = StageId::kDataLoad;
  auto submitted = runtime->Submit(session.Value(), request);
  submit_returned.store(true, std::memory_order_release);
  Check(submitted.IsOk(),
        "callback-owner job is submitted");
  WaitUntil([&] { return owner_released.load(); },
            "terminal callback releases the last RuntimeClient owner");
  WaitUntil([&] { return weak_runtime.expired(); },
            "RuntimeClient public lifetime ends without worker self-join");
  subscription.Reset();
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
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
        1, [&](const BootstrapConfigSnapshot&, const fs::path&)
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
      1, [](const BootstrapConfigSnapshot&, const fs::path&)
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
  TestBootstrapConfigIsAnImmutableValueSnapshot();
  TestRuntimeServiceExposesSessionScopedControlPlane();
  TestAtomicReplacementTransfersAdmissionAndPersistsRoot();
  TestReplacementInstallFailurePreservesOldSessionAndOutput();
  TestReplacementRejectsAnActiveJobAndWaitOperation();
  TestReplacementBlocksConcurrentMutatingCommands();
  TestLastRuntimeClientOwnerCanBeReleasedFromTerminalCallback();
  TestMultiSessionIsolationAndLifecycle();
  TestShutdownStressCancelsAndJoins();
  TestFatalStateAndCompletedClose();
  TestDefaultCloseReleasesResidentReservation();
  std::cout << "runtime service multi-session tests passed\n";
  return 0;
}
