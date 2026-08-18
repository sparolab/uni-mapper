#include <open_lmm/server/bootstrap/algorithm_factory.hpp>
#include <open_lmm/server/bootstrap/session_bootstrapper.hpp>
#include <open_lmm/server/execution/algorithm_context.hpp>
#include <open_lmm/server/transaction/session_reconfigurer.hpp>

#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>
#include <open_lmm/core/data_loader/data_loader_base.hpp>
#include <open_lmm/utils/config_schema.hpp>

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>

namespace {

namespace fs = std::filesystem;
using namespace open_lmm;

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(EXIT_FAILURE);
}

void WriteJson(const fs::path& path, const nlohmann::json& value) {
  fs::create_directories(path.parent_path());
  std::ofstream output(path);
  output << value.dump(2) << '\n';
}

fs::path TemporaryDirectory() {
  const fs::path path =
      fs::temp_directory_path() /
      ("open_lmm_bootstrap_" + std::to_string(std::random_device{}()));
  fs::create_directories(path);
  return path;
}

SessionBootstrapRequest Request(
    const fs::path& config, std::optional<fs::path> output,
    std::shared_ptr<CancellationToken> cancellation) {
  auto snapshot = LoadBootstrapConfig(config);
  Check(snapshot.IsOk(), "test bootstrap config snapshot must load");
  return {std::move(snapshot).Value(), std::move(output),
          std::move(cancellation)};
}

void WriteFixture(const fs::path& root) {
  const fs::path data = root / "data";
  const fs::path output = root / "output";
  for (const char* agent : {"follower", "anchor"}) {
    fs::create_directories(data / agent / "Scans");
    std::ofstream(data / agent / "poses.txt")
        << "1 0 0 0 0 1 0 0 0 0 1 0\n";
    std::ofstream(data / agent / "Scans/000000.pcd") << "fixture";
  }
  WriteJson(root / "config/config.json",
            {{"global",
              {{"config_map_server", "server/map.json"},
               {"config_data_loader", "core/data.json"},
               {"config_loop_detector", "core/loop.json"},
               {"config_backend_optimizer", "core/optimizer.json"},
               {"config_dynamic_remover", "core/remover.json"}}},
             {"directory",
              {{"root_dir_path", data.string()},
               {"sub_dir_list", {"follower", "anchor"}},
               {"root_save_dir", output.string()}}}});
  WriteJson(root / "config/server/map.json",
            {{"map_server",
              {{"enable_map_updater", false},
               {"anchor_agent_index", 1},
               {"save_voxel_size", 0.3},
               {"parallel_data_load", true},
               {"parallel_map_update", false},
               {"max_parallel_agents", 3}}}});
  WriteJson(root / "config/core/data.json",
            {{"data_loader",
              {{"data_loader_type", "file_based"},
               {"pose_format", "kitti"},
               {"pose_file_name", "poses.txt"},
               {"extrinsic", {0, 0, 0, 0, 0, 0, 1}},
               {"scan_type", "pcd"},
               {"scan_dir_name", "Scans"},
               {"voxel_size", 0.5},
               {"min_range", 1.0},
               {"max_range", 60.0},
               {"delimiter", " "}}}});
  WriteJson(root / "config/core/loop.json",
            {{"loop_detector",
              {{"loop_detector_type", "kdtree"},
               {"model", "scan_context"}}},
             {"database",
              {{"descriptor_vector_dim", 20},
               {"distance_threshold", 0.15},
               {"num_candidates", 3},
               {"rebuild_threshold", 50}}},
             {"alignment",
              {{"pcm_translation_threshold", 10.0},
               {"pcm_rotation_threshold_deg", 20.0},
               {"pcm_solver", "heuristic"},
               {"pcm_threads", 1},
               {"pcm_max_candidates", 0}}}});
  WriteJson(root / "config/core/optimizer.json",
            {{"backend_optimizer",
              {{"backend_optimizer_type", "incremental"},
               {"relinearizeThreshold", 0.1},
               {"relinearizeSkip", 1},
               {"isam_extra_updates", 1},
               {"min_loop_frame_gap", 30},
               {"icp_search_num", 1}}}});
  WriteJson(root / "config/core/remover.json",
            {{"dynamic_remover",
              {{"dynamic_remover_type", "offline"},
               {"model", "free_dom"}}}});
}

class TrackingFactory final : public AlgorithmFactory {
 public:
  Result<void> Preflight(const LoopDetectorConfig&,
                         const DynamicRemoverConfig&) const override {
    preflight_called = true;
    return Result<void>::Ok();
  }
  Result<void> PreflightDescriptor(const LoopDetectorConfig&) const override {
    return Result<void>::Ok();
  }
  Result<void> PreflightRemover(const DynamicRemoverConfig&) const override {
    return Result<void>::Ok();
  }

 protected:
  Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizerImpl(
      const OptimizerConfig& config) const override {
    optimizer_called = true;
    return AlgorithmFactory::CreateOptimizerImpl(config);
  }

 public:

  mutable bool preflight_called = false;
  mutable bool optimizer_called = false;
};

class FaultyFactory final : public AlgorithmFactory {
 public:
  enum class Mode { kStdThrow, kUnknownThrow, kNull };
  explicit FaultyFactory(Mode mode) : mode_(mode) {}

 protected:
  Result<std::unique_ptr<DataLoaderBase>> CreateDataLoaderImpl(
      const DataLoaderConfig&) const override {
    if (mode_ == Mode::kStdThrow) throw std::runtime_error("fixture");
    if (mode_ == Mode::kUnknownThrow) throw 7;
    return Result<std::unique_ptr<DataLoaderBase>>::Ok(nullptr);
  }

 private:
  Mode mode_;
};

void TestAlgorithmFactoryNormalizesFaults() {
  DataLoaderConfig config;
  for (const auto mode : {FaultyFactory::Mode::kStdThrow,
                          FaultyFactory::Mode::kUnknownThrow,
                          FaultyFactory::Mode::kNull}) {
    FaultyFactory factory(mode);
    auto result = factory.CreateDataLoader(config);
    Check(!result, "factory faults must become Result failures");
  }
}

void TestAlgorithmContextUsesDocumentFingerprint() {
  auto documents = std::make_shared<SessionConfigDocuments>();
  documents->data_loader.canonical_json = R"({"domain":"loader"})";
  documents->dynamic_remover.canonical_json = R"({"domain":"remover"})";
  auto config = std::make_shared<SessionConfig>();
  config->documents = documents;
  config->fingerprint = "alignment-wide";
  SessionState state;
  state.revision = 9;
  state.config = config;
  ExecutionContext command{std::make_shared<CancellationToken>(), {}, 9};
  auto loader = MakeAlgorithmExecutionContext(
      state, command, {}, documents->data_loader, "loader", "load", "file");
  auto remover = MakeAlgorithmExecutionContext(
      state, command, {}, documents->dynamic_remover, "remover", "remove",
      "offline");
  Check(loader.config->fingerprint != remover.config->fingerprint &&
            loader.config->fingerprint != config->fingerprint,
        "algorithm context must fingerprint its selected canonical document");
}

void TestBootstrapSnapshotAndCache() {
  const fs::path root = TemporaryDirectory();
  WriteFixture(root);
  const fs::path output = root / "chosen_output";
  auto factory = std::make_shared<TrackingFactory>();
  SessionBootstrapper bootstrapper(factory);
  const SessionBootstrapRequest request = Request(
      root / "config", output, std::make_shared<CancellationToken>());
  auto first = bootstrapper.Bootstrap(request);
  if (!first) std::cerr << first.GetError().Message() << '\n';
  Check(first.IsOk(), "valid fixture must bootstrap");
  const auto& result = first.Value();
  const auto& state = *result.initial_state;
  Check(factory->preflight_called && factory->optimizer_called,
        "bootstrap must use AlgorithmFactory for preflight and optimizer");
  Check(state.revision == 1 && state.config && state.payload &&
            state.payload->optimizer && state.payload->database,
        "bootstrap must return a complete immutable initial state");
  Check(state.ordered_agents.size() == 2 &&
            state.ordered_agents.front().Value() == "anchor" &&
            state.payload->contexts.front().agent.role == AgentRole::kAnchor,
        "anchor-first agent and context order must be preserved");
  Check(state.config->root.output_directory == output &&
            state.config->root.save_voxel_size == 0.3 &&
            state.config->root.max_parallel_agents == 3 &&
            result.suggested_resource_budget.max_agent_tasks == 3 &&
            result.suggested_resource_budget.max_cpu_threads == 3,
        "root policies and suggested budget must preserve config values");
  Check(state.config->documents &&
            !state.config->documents->root.canonical_json.empty() &&
            state.config->documents->loop_detector.path ==
                root / "config/core/loop.json",
        "validated config documents must be revisioned in the snapshot");
  Check(state.config->alignment_artifacts &&
            state.config->alignment_artifacts->input_fingerprints.size() == 2 &&
            state.config->alignment_artifacts->cache_path ==
                output / "map_alignment_cache.json" &&
            fs::is_regular_file(output / "agent_manifest.json"),
        "alignment metadata and agent manifest must be prepared");
  {
    std::ifstream manifest_input(output / "agent_manifest.json");
    nlohmann::json manifest_json;
    manifest_input >> manifest_json;
    Check(manifest_json["agents"].size() == 2 &&
              manifest_json["agents"][0]["id"] == "follower" &&
              manifest_json["agents"][1]["id"] == "anchor" &&
              manifest_json["agents"][0]["symbol_byte"] ==
                  state.agent_catalog->SymbolFor(
                      AgentId::Parse("follower").Value()).Value().Byte() &&
              manifest_json["agents"][1]["symbol_byte"] ==
                  state.agent_catalog->SymbolFor(
                      AgentId::Parse("anchor").Value()).Value().Byte(),
          "manifest identity order must remain config order while runtime is anchor-first");
  }

  const std::string fingerprint =
      state.config->alignment_artifacts->session_fingerprint;
  WriteJson(output / "map_alignment_cache.json",
            {{"version", 3},
             {"session_fingerprint", fingerprint},
             {"alignments",
              {{{"approval", "user"},
                {"source_agent", "follower"},
                {"target_agent", "anchor"},
                {"method", "manual"},
                {"accepted_global_T_agent",
                 {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}},
                {"accepted_at_unix_ms", 42}}}}});
  auto second = bootstrapper.Bootstrap(request);
  Check(second.IsOk() &&
            second.Value()
                    .initial_state->payload->database->stored_alignments.size() ==
                1,
        "matching v3 alignment cache must load into the initial database");

  std::error_code ignored;
  fs::remove_all(root, ignored);
}

void TestExternalPluginSchemaBootstrapsWithoutCentralCatalogEntry() {
  const fs::path root = TemporaryDirectory();
  WriteFixture(root);
  WriteJson(root / "config/core/loop.json",
            {{"loop_detector", {{"loop_detector_type", "kdtree"},
                                {"plugin_abi", "v2"},
                                {"model", "external_fixture"},
                                {"external_fixture", {{"tuning", 0.75}}}}},
             {"database", {{"descriptor_vector_dim", 1}}}});
  SessionBootstrapper bootstrapper(std::make_shared<TrackingFactory>());
  auto result = bootstrapper.Bootstrap(Request(
      root / "config", root / "external_output",
      std::make_shared<CancellationToken>()));
  if (!result) std::cerr << result.GetError().Message() << '\n';
  Check(result && result.Value().initial_state->config->schema_registry,
        "external self-described plugin did not bootstrap");
  bool found = false;
  for (const auto& fragment :
       result.Value().initial_state->config->schema_registry->Fragments(
           ConfigDocumentKind::kLoopDetector)) {
    found |= fragment.id == "descriptor.external_fixture.schema";
  }
  Check(found, "session-local registry omitted external plugin fragment");
  bool global_found = false;
  for (const auto& fragment : BuiltinConfigSchemaRegistry().Fragments(
           ConfigDocumentKind::kLoopDetector)) {
    global_found |= fragment.id == "descriptor.external_fixture.schema";
  }
  Check(!global_found, "external plugin mutated the builtin schema registry");
  const auto base = result.Value().initial_state;
  setenv("OPEN_LMM_SCHEMA_FIXTURE_QUERY_UNAVAILABLE", "1", 1);
  auto unavailable = SessionReconfigurer(std::make_shared<TrackingFactory>())
                         .Prepare(base, ConfigDomain::kDynamicRemover,
                                  base->config->revision + 1);
  unsetenv("OPEN_LMM_SCHEMA_FIXTURE_QUERY_UNAVAILABLE");
  Check(unavailable &&
            unavailable.Value().config->descriptor_plugin_schema ==
                base->config->descriptor_plugin_schema,
        "unrelated remover reconfigure queried an unavailable committed "
        "descriptor plugin");
  setenv("OPEN_LMM_SCHEMA_FIXTURE_MUTATED", "1", 1);
  auto unrelated = SessionReconfigurer(std::make_shared<TrackingFactory>())
                       .Prepare(base, ConfigDomain::kDynamicRemover,
                                base->config->revision + 1);
  unsetenv("OPEN_LMM_SCHEMA_FIXTURE_MUTATED");
  Check(unrelated &&
            unrelated.Value().config->descriptor_plugin_schema ==
                base->config->descriptor_plugin_schema,
        "unrelated remover reconfigure reopened or replaced the committed "
        "descriptor schema");
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

void TestPluginSchemaReconfigureIsTransactional() {
  const fs::path root = TemporaryDirectory();
  WriteFixture(root);
  auto factory = std::make_shared<TrackingFactory>();
  SessionBootstrapper bootstrapper(factory);
  auto boot = bootstrapper.Bootstrap(Request(
      root / "config", root / "reconfigure_output",
      std::make_shared<CancellationToken>()));
  Check(boot.IsOk(), "base session for plugin schema reconfigure did not bootstrap");
  const auto base = boot.Value().initial_state;
  const auto prior_registry = base->config->schema_registry;
  WriteJson(root / "config/core/loop.json",
            {{"loop_detector", {{"loop_detector_type", "kdtree"},
                                {"plugin_abi", "v2"},
                                {"model", "external_fixture"},
                                {"external_fixture", {{"tuning", 0.5}}}}},
             {"database", {{"descriptor_vector_dim", 1}}}});
  auto changed = SessionReconfigurer(factory).Prepare(
      base, ConfigDomain::kLoopDetector, base->config->revision + 1);
  Check(changed && changed.Value().config->schema_registry != prior_registry &&
            changed.Value().config->loop_detector->model == "external_fixture",
        "plugin selector change did not build a candidate session registry");

  WriteJson(root / "config/core/loop.json",
            {{"loop_detector",
              {{"loop_detector_type", "kdtree"}, {"plugin_abi", "v2"},
               {"model", "external_fixture"}}},
             {"database", {{"descriptor_vector_dim", 1}}}});
  auto rejected = SessionReconfigurer(factory).Prepare(
      base, ConfigDomain::kLoopDetector, base->config->revision + 1);
  Check(!rejected && base->config->schema_registry == prior_registry &&
            base->config->loop_detector->model == "scan_context" &&
            base->config->revision == 1,
        "failed plugin schema reconfigure mutated committed session state");
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

void TestCancellationBeforeSideEffects() {
  const fs::path root = TemporaryDirectory();
  WriteFixture(root);
  auto cancellation = std::make_shared<CancellationToken>();
  cancellation->Request();
  SessionBootstrapper bootstrapper(std::make_shared<TrackingFactory>());
  auto result = bootstrapper.Bootstrap(
      Request(root / "config", root / "cancelled_output", cancellation));
  Check(!result && result.GetError().code == Error::Code::kCancelled &&
            !fs::exists(root / "cancelled_output"),
        "pre-cancelled bootstrap must fail before output side effects");
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

void TestEveryModuleReadIsBounded() {
  const std::vector<fs::path> modules = {
      "config/server/map.json", "config/core/data.json",
      "config/core/loop.json", "config/core/optimizer.json",
      "config/core/remover.json"};
  for (const auto& module : modules) {
    const fs::path root = TemporaryDirectory();
    WriteFixture(root);
    {
      std::ofstream output(root / module, std::ios::binary | std::ios::trunc);
      output << std::string(SchemaLimits{}.maximum_document_bytes + 1U, ' ');
    }
    SessionBootstrapper bootstrapper(std::make_shared<TrackingFactory>());
    auto result = bootstrapper.Bootstrap(Request(
        root / "config", root / "bounded_output",
        std::make_shared<CancellationToken>()));
    Check(!result &&
              result.GetError().code == Error::Code::kInvalidArgument &&
              result.GetError().message.find("byte limit") != std::string::npos &&
              !fs::exists(root / "bounded_output"),
          "every module must be bounded before parse and output side effects");
    std::error_code ignored;
    fs::remove_all(root, ignored);
  }
}

void TestInvalidInputsDoNotPublishManifest() {
  const fs::path root = TemporaryDirectory();
  WriteFixture(root);
  const fs::path existing_output = root / "existing_output";
  fs::create_directories(existing_output);
  const fs::path manifest = existing_output / "agent_manifest.json";
  std::ofstream(manifest) << "existing manifest\n";
  fs::remove(root / "data/follower/Scans/000000.pcd");

  SessionBootstrapper bootstrapper(std::make_shared<TrackingFactory>());
  auto existing = bootstrapper.Bootstrap(Request(
      root / "config", existing_output,
      std::make_shared<CancellationToken>()));
  std::ifstream preserved(manifest);
  std::string contents;
  std::getline(preserved, contents);
  Check(!existing && contents == "existing manifest" &&
            !fs::exists(manifest.string() + ".tmp"),
        "invalid input cardinality must not replace an existing manifest");

  const fs::path absent_output = root / "absent_output";
  auto absent = bootstrapper.Bootstrap(Request(
      root / "config", absent_output, std::make_shared<CancellationToken>()));
  Check(!absent && !fs::exists(absent_output),
        "invalid input cardinality must not create an output directory");

  std::error_code ignored;
  fs::remove_all(root, ignored);
}

void TestManifestPreflightFailureCleansTemporaryFile() {
  const fs::path root = TemporaryDirectory();
  WriteFixture(root);
  const fs::path output = root / "blocked_output";
  fs::create_directories(output);
  std::ofstream(output / "agent_manifest.json") << "existing manifest\n";
  std::ofstream(output / "agent_manifest.json.open_lmm_backup")
      << "stale backup\n";

  SessionBootstrapper bootstrapper(std::make_shared<TrackingFactory>());
  auto result = bootstrapper.Bootstrap(Request(
      root / "config", output, std::make_shared<CancellationToken>()));
  Check(!result && !fs::exists(output / "agent_manifest.json.tmp"),
        "recoverable manifest commit failure must clean its temporary file");

  std::ifstream preserved(output / "agent_manifest.json");
  std::string contents;
  std::getline(preserved, contents);
  Check(contents == "existing manifest",
        "manifest preflight failure must preserve the existing manifest");
  std::error_code ignored;
  fs::remove_all(root, ignored);
}

}  // namespace

int main() {
  TestAlgorithmFactoryNormalizesFaults();
  TestAlgorithmContextUsesDocumentFingerprint();
  TestBootstrapSnapshotAndCache();
  TestExternalPluginSchemaBootstrapsWithoutCentralCatalogEntry();
  TestPluginSchemaReconfigureIsTransactional();
  TestCancellationBeforeSideEffects();
  TestEveryModuleReadIsBounded();
  TestInvalidInputsDoNotPublishManifest();
  TestManifestPreflightFailureCleansTemporaryFile();
  std::cout << "SessionBootstrapper tests passed\n";
  return EXIT_SUCCESS;
}
