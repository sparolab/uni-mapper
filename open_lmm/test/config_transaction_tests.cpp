#include <open_lmm/server/artifact_repository.hpp>
#include <open_lmm/server/bootstrap/algorithm_factory.hpp>
#include <open_lmm/server/bootstrap/bootstrap_config.hpp>
#include <open_lmm/server/execution/stage_coordinator.hpp>
#include <open_lmm/server/output_repository.hpp>
#include <open_lmm/utils/config_schema.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <iterator>
#include <memory>
#include <string>
#include <thread>

namespace {
namespace fs = std::filesystem;
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

std::string Read(const fs::path& path) {
  std::ifstream input(path);
  return {std::istreambuf_iterator<char>(input), {}};
}

std::shared_ptr<const SessionState> MakeBase(const fs::path& root) {
  fs::create_directories(root / "server");
  const std::string root_json = R"({
    "global": {
      "config_map_server": "server/map.json",
      "config_data_loader": "data.json",
      "config_loop_detector": "loop.json",
      "config_backend_optimizer": "optimizer.json",
      "config_dynamic_remover": "remover.json"
    },
    "directory": {
      "root_dir_path": "data",
      "sub_dir_list": ["A"],
      "root_save_dir": "output"
    }
  })";
  const std::string map_json = R"({"map_server":{
    "enable_map_updater":false,
    "anchor_agent_index":0,
    "save_voxel_size":0.2,
    "parallel_data_load":false,
    "parallel_map_update":false,
    "max_parallel_agents":1
  }})";
  const auto& registry = BuiltinConfigSchemaRegistry();
  auto root_document = registry.ParseAndValidate(
      ConfigDocumentKind::kRoot, root_json, (root / "config.json").string());
  auto map_document = registry.ParseAndValidate(
      ConfigDocumentKind::kMapServer, map_json,
      (root / "server/map.json").string());
  Check(root_document && map_document, "base config documents must validate");
  std::ofstream(root / "config.json") << root_document.Value().CanonicalJson();
  std::ofstream(root / "server/map.json")
      << map_document.Value().CanonicalJson();

  auto map_config = DecodeMapSaveConfig(map_document.Value());
  OptimizerConfig optimizer_config;
  optimizer_config.type = "incremental";
  auto optimizer = AlgorithmFactory().CreateOptimizer(optimizer_config);
  Check(map_config && optimizer, "base typed config must be constructible");
  auto documents = std::make_shared<SessionConfigDocuments>();
  documents->root = {root / "config.json",
                     root_document.Value().CanonicalJson()};
  documents->map_server = {root / "server/map.json",
                           map_document.Value().CanonicalJson()};
  documents->data_loader.path = root / "data.json";
  documents->loop_detector.path = root / "loop.json";
  documents->optimizer.path = root / "optimizer.json";
  documents->dynamic_remover.path = root / "remover.json";

  auto config = std::make_shared<SessionConfig>();
  config->revision = 1;
  config->root.output_directory = root / "output";
  config->map_save = std::make_shared<const MapSaveConfig>(map_config.Value());
  config->optimizer = std::make_shared<const OptimizerConfig>();
  config->alignment_artifacts = std::make_shared<AlignmentArtifactMetadata>();
  config->documents = documents;
  config->schema_registry = std::shared_ptr<const SchemaRegistry>(
      &registry, [](const SchemaRegistry*) {});

  auto payload = std::make_shared<SessionPayload>();
  payload->database = std::make_shared<SharedDatabase>();
  payload->optimizer = std::move(optimizer).Value();
  const AgentId agent = AgentId::Parse("A").Value();
  ArtifactRepository artifacts;
  artifacts.Reset({agent});
  auto state = std::make_shared<SessionState>();
  state->revision = 1;
  state->config = std::move(config);
  state->ordered_agents = {agent};
  state->payload = std::move(payload);
  state->artifacts = artifacts.Snapshot();
  return state;
}

ConfigCandidate Candidate(std::string selected, double voxel) {
  return {ConfigDomain::kMapSave,
          "{\"map_server\":{\"enable_map_updater\":false,"
          "\"anchor_agent_index\":0,\"save_voxel_size\":" +
              std::to_string(voxel) +
              ",\"parallel_data_load\":false,"
              "\"parallel_map_update\":false,"
              "\"max_parallel_agents\":1}}",
          fs::path(std::move(selected))};
}

}  // namespace

int main() {
  const fs::path root = fs::temp_directory_path() /
                        "open_lmm_config_transaction_tests";
  std::error_code ignored;
  fs::remove_all(root, ignored);
  const auto initial = MakeBase(root);
  const std::string root_on_disk = Read(root / "config.json");
  const std::string root_candidate = R"({
    "global": {
      "config_map_server": "server/map.json",
      "config_data_loader": "data.json",
      "config_loop_detector": "loop.json",
      "config_backend_optimizer": "optimizer.json",
      "config_dynamic_remover": "remover.json"
    },
    "directory": {
      "root_dir_path": "candidate-data",
      "sub_dir_list": ["B", "C"],
      "root_save_dir": "candidate-output"
    }
  })";
  auto bootstrap_candidate =
      LoadBootstrapConfigCandidate(root, root_candidate);
  Check(bootstrap_candidate &&
            bootstrap_candidate.Value().DataSubdirectories() ==
                std::vector<std::string>({"B", "C"}) &&
            bootstrap_candidate.Value().DataRoot() == root / "candidate-data" &&
            bootstrap_candidate.Value().OutputRoot() ==
                root / "candidate-output" &&
            Read(root / "config.json") == root_on_disk,
        "in-memory root bootstrap must validate without pre-writing config.json");
  std::string oversized(SchemaLimits{}.maximum_document_bytes + 1, 'x');
  Check(!LoadBootstrapConfigCandidate(root, oversized),
        "in-memory root bootstrap must enforce the document byte cap");
  SessionManager sessions(initial);
  OutputRepository outputs;
  auto governor = std::make_shared<ResourceGovernor>(ResourceBudget{});
  StageCoordinator coordinator(sessions, outputs, governor);
  auto cancellation = std::make_shared<CancellationToken>();

  const auto first = Candidate("server/selected-a.json", 0.45);
  const auto second = Candidate("server/selected-b.json", 0.55);
  Result<ConfigApplyReceipt> first_result =
      Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument("unset"));
  Result<ConfigApplyReceipt> second_result =
      Result<ConfigApplyReceipt>::Failure(Error::InvalidArgument("unset"));
  std::thread one([&] {
    first_result = coordinator.ApplyConfig(
        initial, first, {1, 1}, {cancellation, {}, 1});
  });
  std::thread two([&] {
    second_result = coordinator.ApplyConfig(
        initial, second, {1, 1}, {cancellation, {}, 1});
  });
  one.join();
  two.join();
  Check(first_result.IsOk() != second_result.IsOk(),
        "exactly one concurrent expected-revision transaction must commit");
  const auto& winner = first_result ? first_result.Value()
                                    : second_result.Value();
  Check(winner.base_session_revision == 1 && winner.session_revision == 2 &&
            winner.previous_config_revision == 1 &&
            winner.config_revision == 2,
        "successful receipt must describe its committed candidate, not a later query");
  const auto committed = sessions.Snapshot();
  Check(committed->revision == 2 && committed->config->revision == 2 &&
            fs::is_regular_file(committed->config->documents->map_server.path),
        "file and state revisions must commit together");

  const std::string root_before_failure = Read(root / "config.json");
  auto blocked = Candidate("server/blocked.json", 0.65);
  std::ofstream(root / "server/blocked.json.open_lmm_backup") << "fault";
  auto failed = coordinator.ApplyConfig(
      committed, blocked, {2, 2}, {cancellation, {}, 2});
  Check(!failed && sessions.Snapshot().get() == committed.get() &&
            Read(root / "config.json") == root_before_failure &&
            !fs::exists(root / "server/blocked.json"),
        "file transaction failure must preserve disk and state");
  bool temporary_found = false;
  for (const auto& entry : fs::directory_iterator(root / "server")) {
    temporary_found |=
        entry.path().filename().string().find(".open_lmm_candidate_") !=
        std::string::npos;
  }
  Check(!temporary_found,
        "failed config transaction must clean every prepared temporary");

  auto stale = coordinator.ApplyConfig(
      committed, Candidate("server/stale.json", 0.75), {1, 2},
      {cancellation, {}, 2});
  Check(!stale && sessions.Snapshot().get() == committed.get() &&
            !fs::exists(root / "server/stale.json"),
        "expected revision conflict must perform no file mutation");

  auto wrong = Candidate("server/wrong.json", 0.85);
  wrong.domain = ConfigDomain::kLoopDetector;
  auto wrong_domain = coordinator.ApplyConfig(
      committed, wrong, {2, 2}, {cancellation, {}, 2});
  Check(!wrong_domain && sessions.Snapshot().get() == committed.get() &&
            !fs::exists(root / "server/wrong.json"),
        "wrong-domain candidate must preserve unrelated state and files");
  const std::string unrelated_before = "unrelated-loop-document";
  std::ofstream(root / "loop.json") << unrelated_before;
  auto aliased = coordinator.ApplyConfig(
      committed, Candidate("loop.json", 0.9), {2, 2},
      {cancellation, {}, 2});
  Check(!aliased && sessions.Snapshot().get() == committed.get() &&
            Read(root / "loop.json") == unrelated_before,
        "domain candidate must not overwrite another module document");

  // Backup cleanup happens after every final has been installed. A cleanup
  // fault must retain the new state/file-set commit and leave a recovery
  // manifest, rather than report a pre-commit failure with stale state.
  const fs::path cleanup_root = root / "cleanup-fault";
  fs::create_directories(cleanup_root / "second.final" / "retained-child");
  std::ofstream(cleanup_root / "first.final") << "old-one";
  std::ofstream(cleanup_root / "first.tmp") << "new-one";
  std::ofstream(cleanup_root / "second.tmp") << "new-two";
  PendingOutputSet cleanup_pending;
  cleanup_pending.Add(cleanup_root / "first.tmp",
                      cleanup_root / "first.final");
  cleanup_pending.Add(cleanup_root / "second.tmp",
                      cleanup_root / "second.final");
  SessionManager cleanup_sessions(initial);
  auto cleanup_candidate = std::make_shared<SessionState>(*initial);
  cleanup_candidate->revision = 2;
  auto cleanup_commit = cleanup_sessions.CommitWithBarrier(
      initial, cleanup_candidate,
      [&cleanup_pending] { return cleanup_pending.Commit(); });
  Check(cleanup_commit &&
            cleanup_sessions.Snapshot().get() == cleanup_candidate.get() &&
            Read(cleanup_root / "first.final") == "new-one" &&
            Read(cleanup_root / "second.final") == "new-two" &&
            fs::is_directory(cleanup_root /
                             "second.final.open_lmm_backup"),
        "post-install backup cleanup fault must keep new files and new state");
  bool recovery_manifest_found = false;
  for (const auto& entry : fs::directory_iterator(cleanup_root)) {
    recovery_manifest_found |=
        entry.path().filename().string().find(".open_lmm_recovery_") == 0;
  }
  Check(recovery_manifest_found,
        "post-commit cleanup fault must leave a recovery manifest");
  fs::remove_all(root, ignored);
  std::cout << "config transaction tests passed\n";
  return 0;
}
