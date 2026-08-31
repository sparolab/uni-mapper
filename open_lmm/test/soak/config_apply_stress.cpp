#include "support/soak/owner_stress_support.hpp"

#include <config/bootstrap/bootstrap_config.hpp>
#include <open_lmm/utils/config_schema.hpp>
#include <plugins/host/algorithm_factory.hpp>
#include <runtime/execution/stages/stage_coordinator.hpp>
#include <runtime/state/artifact_repository.hpp>
#include <storage/transactions/output_repository.hpp>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include <nlohmann/json.hpp>

#ifndef OPEN_LMM_SOAK_SANITIZER_NAME
#define OPEN_LMM_SOAK_SANITIZER_NAME "none"
#endif

namespace fs = std::filesystem;
namespace soak = open_lmm::test::soak;
using Json = nlohmann::json;

namespace {
using namespace open_lmm;

void Require(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

std::shared_ptr<const RuntimeState> MakeBase(const fs::path& root) {
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
      "root_dir_path": "data", "sub_dir_list": ["A"],
      "root_save_dir": "output"
    }
  })";
  const std::string map_json = R"({"map_server":{
    "enable_map_updater":false, "anchor_agent_index":0,
    "save_voxel_size":0.2, "parallel_data_load":false,
    "parallel_map_update":false, "max_parallel_agents":1
  }})";
  const auto& registry = BuiltinConfigSchemaRegistry();
  auto root_document = registry.ParseAndValidate(
      ConfigDocumentKind::kRoot, root_json, (root / "config.json").string());
  auto map_document = registry.ParseAndValidate(
      ConfigDocumentKind::kMapServer, map_json,
      (root / "server/map.json").string());
  Require(root_document && map_document, "base documents did not validate");
  std::ofstream(root / "config.json")
      << root_document.Value().CanonicalJson();
  std::ofstream(root / "server/map.json")
      << map_document.Value().CanonicalJson();
  auto map_config = DecodeMapSaveConfig(map_document.Value());
  OptimizerConfig optimizer_config;
  optimizer_config.type = "incremental";
  auto optimizer = AlgorithmFactory().CreateOptimizer(optimizer_config);
  Require(map_config && optimizer, "base typed config did not construct");

  auto documents = std::make_shared<RuntimeConfigDocuments>();
  documents->root = {root / "config.json",
                     root_document.Value().CanonicalJson()};
  documents->map_server = {root / "server/map.json",
                           map_document.Value().CanonicalJson()};
  documents->data_loader.path = root / "data.json";
  documents->loop_detector.path = root / "loop.json";
  documents->optimizer.path = root / "optimizer.json";
  documents->dynamic_remover.path = root / "remover.json";

  auto config = std::make_shared<RuntimeConfig>();
  config->revision = 1;
  config->root.output_directory = root / "output";
  config->map_save = std::make_shared<const MapSaveConfig>(map_config.Value());
  config->optimizer = std::make_shared<const OptimizerConfig>();
  config->alignment_artifacts = std::make_shared<AlignmentArtifactMetadata>();
  config->documents = documents;
  config->schema_registry = std::shared_ptr<const SchemaRegistry>(
      &registry, [](const SchemaRegistry*) {});

  auto payload = std::make_shared<RuntimePayload>();
  payload->database = std::make_shared<SharedDatabase>();
  payload->optimizer = std::move(optimizer).Value();
  const AgentId agent = AgentId::Parse("A").Value();
  ArtifactRepository artifacts;
  artifacts.Reset({agent});
  auto state = std::make_shared<RuntimeState>();
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

struct Fixture {
  explicit Fixture(const fs::path& root)
      : initial(MakeBase(root)),
        sessions(initial),
        governor(std::make_shared<ResourceGovernor>(ResourceBudget{})),
        coordinator(sessions, outputs, governor,
                    std::make_shared<AlgorithmFactory>()) {}
  std::shared_ptr<const RuntimeState> initial;
  RuntimeStateStore sessions;
  OutputRepository outputs;
  std::shared_ptr<ResourceGovernor> governor;
  StageCoordinator coordinator;
};

uint64_t RecoveryManifestCount(const fs::path& root) {
  uint64_t count = 0;
  for (const auto& item : fs::recursive_directory_iterator(root)) {
    const std::string name = item.path().filename().string();
    if (name.starts_with(".open_lmm_recovery_") && name.ends_with(".json"))
      ++count;
  }
  return count;
}

soak::ProcessSeries Run(const soak::RunOptions& options, Json& report) {
  soak::TemporaryDirectory temporary("open_lmm_config_apply_soak");
  soak::ProcessSeries series;
  for (uint64_t iteration = 0; iteration < options.iterations; ++iteration) {
    const fs::path success_root = temporary.Path() /
                                  (std::to_string(iteration) + "-success");
    {
      Fixture fixture(success_root);
      auto cancellation = std::make_shared<CancellationToken>();
      auto applied = fixture.coordinator.ApplyConfig(
          fixture.initial, Candidate("server/applied.json", 0.45), {1, 1},
          {cancellation, {}, 1});
      const auto committed_state = fixture.sessions.Snapshot();
      const auto artifact_state = [&](ArtifactType type) {
        const auto found = std::find_if(
            committed_state->artifacts.begin(), committed_state->artifacts.end(),
            [type](const ArtifactMetadata& artifact) {
              return artifact.key.type == type;
            });
        return found == committed_state->artifacts.end()
                   ? ArtifactState::kMissing
                   : found->state;
      };
      Require(applied && applied.Value().runtime_revision == 2 &&
                  applied.Value().config_revision == 2 &&
                  committed_state->revision == 2 &&
                  artifact_state(ArtifactType::kConfigSnapshot) ==
                      ArtifactState::kReady &&
                  artifact_state(ArtifactType::kGlobalMap) ==
                      ArtifactState::kStale &&
                  artifact_state(ArtifactType::kPcdFile) ==
                      ArtifactState::kStale &&
                  fs::is_regular_file(success_root / "server/applied.json"),
              "successful config apply did not publish file/state/artifact authority together");
      auto stale = fixture.coordinator.ApplyConfig(
          fixture.sessions.Snapshot(), Candidate("server/stale.json", 0.55),
          {1, 2}, {cancellation, {}, 2});
      Require(!stale && !fs::exists(success_root / "server/stale.json") &&
                  fixture.sessions.Snapshot()->revision == 2,
              "stale config apply mutated authoritative state or files");
    }
    fs::remove_all(success_root);

    const fs::path cancelled_root = temporary.Path() /
                                    (std::to_string(iteration) + "-cancel");
    {
      Fixture fixture(cancelled_root);
      auto cancellation = std::make_shared<CancellationToken>();
      cancellation->Request();
      auto cancelled = fixture.coordinator.ApplyConfig(
          fixture.initial, Candidate("server/cancelled.json", 0.65), {1, 1},
          {cancellation, {}, 1});
      Require(!cancelled && fixture.sessions.Snapshot().get() ==
                                fixture.initial.get() &&
                  !fs::exists(cancelled_root / "server/cancelled.json"),
              "cancel-before-commit changed config authority");
    }
    fs::remove_all(cancelled_root);

    const fs::path recovery_root = temporary.Path() /
                                   (std::to_string(iteration) + "-recovery");
    {
      Fixture fixture(recovery_root);
      fs::remove(recovery_root / "config.json");
      fs::create_directories(recovery_root / "config.json/retained");
      std::ofstream(recovery_root / "config.json/retained/original")
          << "original";
      auto cancellation = std::make_shared<CancellationToken>();
      auto recovered = fixture.coordinator.ApplyConfig(
          fixture.initial, Candidate("server/recovery.json", 0.75), {1, 1},
          {cancellation, {}, 1});
      const auto authority = fixture.sessions.AuthoritySnapshot();
      Require(recovered && authority.state && authority.state->revision == 2 &&
                  authority.recovery_required &&
                  authority.recovery_required->severity ==
                      Error::Severity::kFatalRuntime &&
                  fs::is_regular_file(recovery_root / "config.json") &&
                  fs::is_regular_file(recovery_root / "server/recovery.json") &&
                  RecoveryManifestCount(recovery_root) == 1,
              "committed config recovery was not authoritative");
      auto blocked = fixture.coordinator.ApplyConfig(
          authority.state, Candidate("server/blocked.json", 0.85), {2, 2},
          {cancellation, {}, 2});
      Require(!blocked &&
                  !fs::exists(recovery_root / "server/blocked.json"),
              "recovery-required authority allowed another mutation");
      Json owner = soak::EmptyOwnerMetrics();
      owner["output_recovery_manifests"] = 1;
      soak::AppendOwnerSample(report, iteration, "config_recovery_committed",
                              soak::SampleProcessMetrics(), std::move(owner));
    }
    fs::remove_all(recovery_root);

    Json idle = soak::EmptyOwnerMetrics();
    idle["output_final_files"] = 0;
    idle["output_temporary_files"] = 0;
    idle["output_backup_entries"] = 0;
    idle["output_recovery_manifests"] = 0;
    const auto process = soak::SampleProcessMetrics();
    soak::AppendOwnerSample(report, iteration, "config_owner_idle", process,
                            std::move(idle));
    soak::AddProcessPoint(series, iteration, process);
  }
  return series;
}

}  // namespace

int main(int argc, char** argv) {
  try {
    const auto options = soak::ParseRunOptions(argc, argv);
    Json report = soak::InitialOwnerReport(
        options, "config-apply-recovery", OPEN_LMM_SOAK_SANITIZER_NAME);
    try {
      const auto series = Run(options, report);
      soak::FinishOwnerReport(options, series, report);
    } catch (const std::exception& error) {
      report["failures"].push_back(
          {{"iteration", nullptr},
           {"phase", "config_apply"},
           {"message", error.what()}});
      report["result"] = "fail";
    }
    const auto validation = soak::ValidateSoakReport(report);
    if (!validation.Ok())
      throw std::runtime_error("invalid soak report:\n" +
                               validation.Summary());
    if (options.report) soak::WriteJsonExclusive(*options.report, report);
    const bool passed = report.at("result") == "pass";
    std::cout << "config apply stress=" << (passed ? "PASS" : "FAIL")
              << " iterations=" << options.iterations << '\n';
    return passed ? 0 : 1;
  } catch (const std::invalid_argument& error) {
    std::cerr << "invalid soak request: " << error.what() << '\n';
    return 2;
  } catch (const std::exception& error) {
    std::cerr << "soak infrastructure failure: " << error.what() << '\n';
    return 2;
  }
}
