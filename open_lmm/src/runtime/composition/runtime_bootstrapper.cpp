#include "runtime_bootstrapper.hpp"

#include <plugins/host/algorithm_factory.hpp>

#include <domain/support/pointcloud_utils.hpp>
#include <runtime/state/artifact_repository.hpp>
#include <storage/transactions/file_set_transaction.hpp>
#include <config/document/config.hpp>
#include <open_lmm/utils/config_schema.hpp>

#include <Eigen/Geometry>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace fs = std::filesystem;
namespace open_lmm {
namespace {

constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
constexpr uint64_t kFnvPrime = 1099511628211ULL;

struct LoadedConfig {
  RuntimeConfigDocument document;
  ValidatedConfigDocument validated;
};

Result<void> CheckCancelled(const RuntimeBootstrapRequest& request,
                            std::string_view boundary) {
  if (request.cancellation &&
      request.cancellation->IsCancellationRequested()) {
    return Result<void>::Failure(Error::Cancelled(boundary));
  }
  return Result<void>::Ok();
}

Result<LoadedConfig> ValidateSnapshot(ConfigDocumentKind kind,
                                      const fs::path& path,
                                      const Config& source,
                                      const SchemaRegistry& registry =
                                          BuiltinConfigSchemaRegistry()) {
  auto validated =
      registry.ParseAndValidate(kind, source.ToJson(), path.string());
  if (!validated) {
    return Result<LoadedConfig>::Failure(validated.GetError());
  }
  auto validated_document = std::move(validated).Value();
  const std::string canonical = validated_document.CanonicalJson();
  return Result<LoadedConfig>::Ok(
      {{path, canonical}, std::move(validated_document)});
}

void HashBytes(uint64_t& hash, const char* data, std::size_t size) {
  for (std::size_t index = 0; index < size; ++index) {
    hash ^= static_cast<unsigned char>(data[index]);
    hash *= kFnvPrime;
  }
}

void HashText(uint64_t& hash, std::string_view text) {
  HashBytes(hash, text.data(), text.size());
}

Result<void> HashFile(uint64_t& hash, const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    return Result<void>::Failure(Error::FileNotFound(path.string()));
  }
  char buffer[8192];
  while (input.read(buffer, sizeof(buffer)) || input.gcount() > 0) {
    HashBytes(hash, buffer, static_cast<std::size_t>(input.gcount()));
  }
  if (input.bad()) {
    return Result<void>::Failure(
        Error::IoError("failed to fingerprint " + path.string()));
  }
  return Result<void>::Ok();
}

std::string HexFingerprint(uint64_t hash) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

std::string AlignmentConfigFingerprint(std::string_view data_loader,
                                       std::string_view loop_detector,
                                       std::string_view optimizer,
                                       int anchor_agent_index) {
  uint64_t hash = kFnvOffset;
  HashText(hash, data_loader);
  HashText(hash, loop_detector);
  HashText(hash, optimizer);
  HashText(hash, std::to_string(anchor_agent_index));
  return HexFingerprint(hash);
}

Result<std::map<AgentId, std::string>> InputFingerprints(
    const std::vector<AgentId>& configured_agents,
    const std::vector<fs::path>& data_directories,
    const DataLoaderConfig& config) {
  std::map<AgentId, std::string> fingerprints;
  for (std::size_t index = 0; index < data_directories.size(); ++index) {
    uint64_t hash = kFnvOffset;
    const fs::path& directory = data_directories[index];
    HashText(hash, directory.string());
    auto pose = HashFile(hash, directory / config.pose_file_name);
    if (!pose) {
      return Result<std::map<AgentId, std::string>>::Failure(pose.GetError());
    }

    const fs::path scan_directory = directory / config.scan_dir_name;
    std::error_code error;
    fs::directory_iterator iterator(scan_directory, error);
    if (error) {
      return Result<std::map<AgentId, std::string>>::Failure(Error::IoError(
          "failed to enumerate scan directory " + scan_directory.string() +
          ": " + error.message()));
    }
    std::vector<fs::path> scans;
    for (const auto& entry : iterator) {
      const bool regular = entry.is_regular_file(error);
      if (error) {
        return Result<std::map<AgentId, std::string>>::Failure(Error::IoError(
            "failed to inspect scan entry " + entry.path().string() + ": " +
            error.message()));
      }
      if (regular && entry.path().extension() == "." + config.scan_type) {
        scans.push_back(entry.path());
      }
    }
    std::sort(scans.begin(), scans.end());
    for (const fs::path& scan : scans) {
      HashText(hash, scan.filename().string());
      const auto size = fs::file_size(scan, error);
      if (error) {
        return Result<std::map<AgentId, std::string>>::Failure(Error::IoError(
            "failed to inspect scan size " + scan.string() + ": " +
            error.message()));
      }
      HashText(hash, std::to_string(size));
      const auto modified = fs::last_write_time(scan, error);
      if (error) {
        return Result<std::map<AgentId, std::string>>::Failure(Error::IoError(
            "failed to inspect scan timestamp " + scan.string() + ": " +
            error.message()));
      }
      HashText(hash, std::to_string(modified.time_since_epoch().count()));
    }
    fingerprints.emplace(configured_agents[index], HexFingerprint(hash));
  }
  return Result<std::map<AgentId, std::string>>::Ok(std::move(fingerprints));
}

Result<void> ValidateInputCardinality(
    const std::vector<AgentId>& configured_agents,
    const std::vector<fs::path>& data_directories,
    const DataLoaderConfig& config) {
  for (std::size_t index = 0; index < data_directories.size(); ++index) {
    const fs::path& directory = data_directories[index];
    const fs::path pose_path = directory / config.pose_file_name;
    const fs::path scan_path = directory / config.scan_dir_name;
    std::error_code error;
    const bool pose_is_file = fs::is_regular_file(pose_path, error);
    if (error || !pose_is_file) {
      return Result<void>::Failure(Error::FileNotFound(
          "agent " + configured_agents[index].Value() + " pose " +
          pose_path.string()));
    }
    error.clear();
    const bool scan_is_directory = fs::is_directory(scan_path, error);
    if (error || !scan_is_directory) {
      return Result<void>::Failure(Error::FileNotFound(
          "agent " + configured_agents[index].Value() + " scans " +
          scan_path.string()));
    }

    std::ifstream poses(pose_path);
    if (!poses) {
      return Result<void>::Failure(Error::IoError(
          "failed to read pose file " + pose_path.string()));
    }
    std::size_t pose_count = 0;
    for (std::string line; std::getline(poses, line);) {
      if (!line.empty()) ++pose_count;
    }
    if (poses.bad()) {
      return Result<void>::Failure(Error::IoError(
          "failed while reading pose file " + pose_path.string()));
    }

    std::size_t scan_count = 0;
    fs::directory_iterator scans(scan_path, error);
    if (error) {
      return Result<void>::Failure(Error::IoError(
          "failed to enumerate scans for agent " +
          configured_agents[index].Value() + ": " + error.message()));
    }
    for (const auto& entry : scans) {
      const bool regular = entry.is_regular_file(error);
      if (error) {
        return Result<void>::Failure(Error::IoError(
            "failed to inspect scan for agent " +
            configured_agents[index].Value() + ": " + error.message()));
      }
      if (regular && entry.path().extension() == "." + config.scan_type) {
        ++scan_count;
      }
    }
    if (pose_count == 0 || scan_count == 0 || pose_count != scan_count) {
      return Result<void>::Failure(Error::InvalidArgument(
          "agent " + configured_agents[index].Value() +
          " input count mismatch: poses=" + std::to_string(pose_count) +
          ", scans=" + std::to_string(scan_count)));
    }
  }
  return Result<void>::Ok();
}

std::optional<Eigen::Isometry3d> MatrixFromJson(const nlohmann::json& value) {
  if (!value.is_array() || value.size() != 16) return std::nullopt;
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int column = 0; column < 4; ++column) {
      const auto& element = value[row * 4 + column];
      if (!element.is_number()) return std::nullopt;
      matrix(row, column) = element.get<double>();
    }
  }
  Eigen::Isometry3d transform(matrix);
  return ValidateRigidTransform(transform, "alignment cache transform")
             ? std::optional<Eigen::Isometry3d>(transform)
             : std::nullopt;
}

std::map<AgentId, StoredAlignment> LoadAlignmentCache(
    const fs::path& path, const std::string& runtime_fingerprint,
    const AgentSymbolCatalogHandle& catalog) {
  std::map<AgentId, StoredAlignment> alignments;
  std::ifstream input(path);
  if (!input) return alignments;
  try {
    nlohmann::json root;
    input >> root;
    if (root.value("version", 0) != 3 ||
        root.value("runtime_fingerprint", std::string()) !=
            runtime_fingerprint) {
      return {};
    }
    for (const auto& item : root.at("alignments")) {
      if (item.value("approval", std::string()) != "user") continue;
      auto source = AgentId::Parse(
          item.value("source_agent", std::string()));
      auto target = AgentId::Parse(
          item.value("target_agent", std::string()));
      if (!source || !target || !catalog->SymbolFor(source.Value()) ||
          !catalog->SymbolFor(target.Value())) {
        continue;
      }
      auto transform = MatrixFromJson(item["accepted_global_T_agent"]);
      if (!transform) continue;
      StoredAlignment stored;
      stored.proposal.source_agent = source.Value();
      stored.proposal.target_agent = target.Value();
      const std::string method = item.value("method", std::string());
      if (method == "manual") {
        stored.proposal.method = AlignmentMethod::kManual;
      } else if (method == "descriptor") {
        stored.proposal.method = AlignmentMethod::kDescriptor;
      } else if (method == "kiss_matcher") {
        stored.proposal.method = AlignmentMethod::kKissMatcher;
      } else {
        continue;
      }
      stored.proposal.target_T_source = *transform;
      if (item.contains("metrics")) {
        const auto& metrics = item["metrics"];
        stored.proposal.metrics.correspondence_count =
            metrics.value("correspondence_count", 0UL);
        stored.proposal.metrics.rotation_inliers =
            metrics.value("rotation_inliers", 0UL);
        stored.proposal.metrics.final_inliers =
            metrics.value("final_inliers", 0UL);
        stored.proposal.metrics.consensus_size =
            metrics.value("consensus_size", 0UL);
        if (metrics.contains("fitness")) {
          stored.proposal.metrics.fitness = metrics["fitness"].get<double>();
        }
        if (metrics.contains("overlap_ratio")) {
          stored.proposal.metrics.overlap_ratio =
              metrics["overlap_ratio"].get<double>();
        }
      }
      stored.approval = AlignmentApproval::kUser;
      stored.accepted_at_unix_ms = item.value("accepted_at_unix_ms", 0ULL);
      alignments[source.Value()] = std::move(stored);
    }
  } catch (const std::exception&) {
    return {};
  }
  return alignments;
}

Result<void> WriteAgentManifest(const fs::path& output_directory,
                                const AgentSymbolCatalog& catalog) {
  nlohmann::json manifest;
  manifest["version"] = AgentSymbolCatalog::kVersion;
  manifest["agents"] = nlohmann::json::array();
  for (const AgentId& id : catalog.OrderedIds()) {
    auto symbol = catalog.SymbolFor(id);
    if (!symbol) return Result<void>::Failure(symbol.GetError());
    manifest["agents"].push_back(
        {{"id", id.Value()}, {"symbol_byte", symbol.Value().Byte()}});
  }
  const fs::path destination = output_directory / "agent_manifest.json";
  fs::path temporary = destination;
  temporary += ".tmp";
  {
    std::ofstream output(temporary);
    if (!output) {
      std::error_code ignored;
      fs::remove(temporary, ignored);
      return Result<void>::Failure(
          Error::IoError("failed to open agent manifest " +
                         temporary.string()));
    }
    output << manifest.dump(2) << '\n';
    if (!output) {
      output.close();
      std::error_code ignored;
      fs::remove(temporary, ignored);
      return Result<void>::Failure(
          Error::IoError("failed to write agent manifest " +
                         temporary.string()));
    }
  }
  auto committed = CommitFileSet({{temporary, destination}});
  if (!committed &&
      committed.GetError().severity != Error::Severity::kFatalRuntime) {
    std::error_code ignored;
    fs::remove(temporary, ignored);
  }
  if (!committed) return Result<void>::Failure(committed.GetError());
  return Result<void>::Ok();
}

}  // namespace

RuntimeBootstrapper::RuntimeBootstrapper(
    std::shared_ptr<const AlgorithmProvider> algorithms)
    : algorithms_(algorithms ? std::move(algorithms)
                             : std::make_shared<AlgorithmFactory>()) {}

Result<RuntimeBootstrapResult> RuntimeBootstrapper::Bootstrap(
    const RuntimeBootstrapRequest& request) const {
  try {
  auto cancelled = CheckCancelled(request, "before runtime bootstrap");
  if (!cancelled) {
    return Result<RuntimeBootstrapResult>::Failure(cancelled.GetError());
  }
  const auto& bootstrap = request.bootstrap_config;
  const auto& root_document = bootstrap.Root().Document();

  const auto load_module = [](const fs::path& path) {
    return LoadConfigFileBounded(path,
                                 SchemaLimits{}.maximum_document_bytes);
  };
  auto map_source = load_module(bootstrap.MapServerConfig());
  auto data_source = load_module(bootstrap.DataLoaderConfig());
  auto loop_source = load_module(bootstrap.LoopDetectorConfig());
  auto optimizer_source = load_module(bootstrap.BackendOptimizerConfig());
  auto remover_source = load_module(bootstrap.DynamicRemoverConfig());
  for (const auto* loaded : {&map_source, &data_source, &loop_source,
                             &optimizer_source, &remover_source}) {
    if (!*loaded) {
      return Result<RuntimeBootstrapResult>::Failure(loaded->GetError());
    }
  }
  auto runtime_registry = std::make_shared<const SchemaRegistry>(
      BuiltinConfigSchemaRegistry());

  auto map_loaded =
      ValidateSnapshot(ConfigDocumentKind::kMapServer,
                       bootstrap.MapServerConfig(), map_source.Value(),
                       *runtime_registry);
  auto data_loaded =
      ValidateSnapshot(ConfigDocumentKind::kDataLoader,
                       bootstrap.DataLoaderConfig(), data_source.Value(),
                       *runtime_registry);
  auto loop_loaded =
      ValidateSnapshot(ConfigDocumentKind::kLoopDetector,
                       bootstrap.LoopDetectorConfig(), loop_source.Value(),
                       *runtime_registry);
  auto optimizer_loaded =
      ValidateSnapshot(ConfigDocumentKind::kBackendOptimizer,
                       bootstrap.BackendOptimizerConfig(),
                       optimizer_source.Value(), *runtime_registry);
  auto remover_loaded =
      ValidateSnapshot(ConfigDocumentKind::kDynamicRemover,
                       bootstrap.DynamicRemoverConfig(), remover_source.Value(),
                       *runtime_registry);
  if (!map_loaded)
    return Result<RuntimeBootstrapResult>::Failure(map_loaded.GetError());
  if (!data_loaded)
    return Result<RuntimeBootstrapResult>::Failure(data_loaded.GetError());
  if (!loop_loaded)
    return Result<RuntimeBootstrapResult>::Failure(loop_loaded.GetError());
  if (!optimizer_loaded)
    return Result<RuntimeBootstrapResult>::Failure(optimizer_loaded.GetError());
  if (!remover_loaded)
    return Result<RuntimeBootstrapResult>::Failure(remover_loaded.GetError());
  LoadedConfig map = std::move(map_loaded).Value();
  LoadedConfig data = std::move(data_loaded).Value();
  LoadedConfig loop = std::move(loop_loaded).Value();
  LoadedConfig optimizer = std::move(optimizer_loaded).Value();
  LoadedConfig remover = std::move(remover_loaded).Value();

  auto root_schema = runtime_registry->Validate(
      ConfigDocumentKind::kRoot, root_document,
      (bootstrap.ConfigDirectory() / "config.json").string());
  auto map_schema = runtime_registry->ParseAndValidate(
      ConfigDocumentKind::kMapServer, map.document.canonical_json,
      map.document.path.string());
  if (!root_schema) {
    return Result<RuntimeBootstrapResult>::Failure(root_schema.GetError());
  }
  if (!map_schema) {
    return Result<RuntimeBootstrapResult>::Failure(map_schema.GetError());
  }
  auto runtime_schema = ValidateRuntimeConfigDocuments(root_schema.Value(),
                                                        map_schema.Value());
  if (!runtime_schema) {
    return Result<RuntimeBootstrapResult>::Failure(runtime_schema.GetError());
  }

  const fs::path& root_data = bootstrap.DataRoot();
  const auto& subdirectories = bootstrap.DataSubdirectories();
  const fs::path& root_output = bootstrap.OutputRoot();
  if (root_data.empty() || root_output.empty() || subdirectories.empty()) {
    return Result<RuntimeBootstrapResult>::Failure(Error::InvalidArgument(
        "directory root_dir_path, root_save_dir, and sub_dir_list must be non-empty"));
  }

  std::vector<AgentId> configured_agents;
  std::vector<fs::path> data_directories;
  configured_agents.reserve(subdirectories.size());
  data_directories.reserve(subdirectories.size());
  for (const std::string& name : subdirectories) {
    auto agent = AgentId::Parse(name);
    if (!agent) {
      return Result<RuntimeBootstrapResult>::Failure(agent.GetError());
    }
    const fs::path directory = root_data / name;
    std::error_code error;
    const bool is_directory = fs::is_directory(directory, error);
    if (error) {
      return Result<RuntimeBootstrapResult>::Failure(Error::IoError(
          "failed to inspect agent directory " + directory.string() + ": " +
          error.message()));
    }
    if (!is_directory) {
      return Result<RuntimeBootstrapResult>::Failure(
          Error::FileNotFound(directory.string()));
    }
    configured_agents.push_back(std::move(agent).Value());
    data_directories.push_back(directory);
  }
  auto catalog_result = AgentSymbolCatalog::Build(configured_agents);
  if (!catalog_result) {
    return Result<RuntimeBootstrapResult>::Failure(catalog_result.GetError());
  }
  auto catalog = std::make_shared<const AgentSymbolCatalog>(
      std::move(catalog_result).Value());

  const auto& map_document = map.validated.Document().at("map_server");
  const bool map_enabled = map_document.at("enable_map_updater").get<bool>();
  const int anchor = map_document.at("anchor_agent_index").get<int>();
  if (anchor < 0 || anchor >= static_cast<int>(configured_agents.size())) {
    return Result<RuntimeBootstrapResult>::Failure(Error::InvalidArgument(
        "map_server/anchor_agent_index is outside the configured agent range"));
  }
  const double save_voxel = map_document.at("save_voxel_size").get<double>();
  if (save_voxel <= 0.0) {
    return Result<RuntimeBootstrapResult>::Failure(Error::InvalidArgument(
        "map_server/save_voxel_size must be greater than zero"));
  }
  const bool parallel_load = map_document.at("parallel_data_load").get<bool>();
  const bool parallel_map = map_document.at("parallel_map_update").get<bool>();
  const int max_parallel = map_document.at("max_parallel_agents").get<int>();
  if (max_parallel <= 0) {
    return Result<RuntimeBootstrapResult>::Failure(Error::InvalidArgument(
        "map_server/max_parallel_agents must be greater than zero"));
  }

  auto data_config = DecodeDataLoaderConfig(data.validated);
  auto loop_config = DecodeLoopDetectorConfig(loop.validated);
  auto optimizer_config = DecodeOptimizerConfig(optimizer.validated);
  auto remover_config = DecodeDynamicRemoverConfig(remover.validated);
  auto map_config = DecodeMapSaveConfig(map.validated);
  if (!data_config)
    return Result<RuntimeBootstrapResult>::Failure(data_config.GetError());
  if (!loop_config)
    return Result<RuntimeBootstrapResult>::Failure(loop_config.GetError());
  if (!optimizer_config)
    return Result<RuntimeBootstrapResult>::Failure(optimizer_config.GetError());
  if (!remover_config)
    return Result<RuntimeBootstrapResult>::Failure(remover_config.GetError());
  if (!map_config)
    return Result<RuntimeBootstrapResult>::Failure(map_config.GetError());

  auto input_cardinality = ValidateInputCardinality(
      configured_agents, data_directories, data_config.Value());
  if (!input_cardinality) {
    return Result<RuntimeBootstrapResult>::Failure(
        input_cardinality.GetError());
  }

  auto preflight = algorithms_->Preflight(loop_config.Value(),
                                          remover_config.Value());
  if (!preflight) {
    return Result<RuntimeBootstrapResult>::Failure(preflight.GetError());
  }
  auto optimizer_instance = algorithms_->CreateOptimizer(optimizer_config.Value());
  if (!optimizer_instance) {
    return Result<RuntimeBootstrapResult>::Failure(
        optimizer_instance.GetError());
  }

  const std::string config_fingerprint = AlignmentConfigFingerprint(
      data.document.canonical_json, loop.document.canonical_json,
      optimizer.document.canonical_json, anchor);
  auto input_fingerprints = InputFingerprints(
      configured_agents, data_directories, data_config.Value());
  if (!input_fingerprints) {
    return Result<RuntimeBootstrapResult>::Failure(
        input_fingerprints.GetError());
  }
  uint64_t runtime_hash = kFnvOffset;
  HashText(runtime_hash, config_fingerprint);
  for (const auto& [agent, fingerprint] : input_fingerprints.Value()) {
    HashText(runtime_hash, agent.Value());
    HashText(runtime_hash, fingerprint);
  }
  const std::string runtime_fingerprint = HexFingerprint(runtime_hash);
  const fs::path cache_root =
      request.output_directory ? *request.output_directory : root_output;
  const fs::path cache_path = cache_root / "map_alignment_cache.json";

  const fs::path output = request.output_directory
                              ? *request.output_directory
                              : root_output /
                                    Config::FromJson(bootstrap.Root().CanonicalJson())
                                        .create_date();
  std::vector<AgentId> ordered_agents;
  ordered_agents.reserve(configured_agents.size());
  ordered_agents.push_back(configured_agents[static_cast<std::size_t>(anchor)]);
  for (std::size_t index = 0; index < configured_agents.size(); ++index) {
    if (index != static_cast<std::size_t>(anchor)) {
      ordered_agents.push_back(configured_agents[index]);
    }
  }

  auto database = std::make_shared<SharedDatabase>();
  database->stored_alignments =
      LoadAlignmentCache(cache_path, runtime_fingerprint, catalog);
  auto payload = std::make_shared<RuntimePayload>();
  for (std::size_t index = 0; index < configured_agents.size(); ++index) {
    AgentPipelineCtx context;
    context.agent = {.id = configured_agents[index],
                     .symbol = catalog->SymbolFor(configured_agents[index]).Value(),
                     .catalog = catalog,
                     .role = index == static_cast<std::size_t>(anchor)
                                 ? AgentRole::kAnchor
                                 : AgentRole::kFollower,
                     .order = static_cast<int>(index)};
    context.data_dir = data_directories[index];
    context.cancellation = request.cancellation;
    payload->contexts.push_back(std::move(context));
  }
  std::stable_sort(payload->contexts.begin(), payload->contexts.end(),
                   [](const AgentPipelineCtx& lhs,
                      const AgentPipelineCtx& rhs) {
                     return static_cast<int>(lhs.agent.role) <
                            static_cast<int>(rhs.agent.role);
                   });
  payload->database = std::move(database);
  payload->optimizer = std::move(optimizer_instance).Value();

  auto documents = std::make_shared<RuntimeConfigDocuments>();
  documents->root = {bootstrap.ConfigDirectory() / "config.json",
                     bootstrap.Root().CanonicalJson()};
  documents->map_server = std::move(map.document);
  documents->data_loader = std::move(data.document);
  documents->loop_detector = std::move(loop.document);
  documents->optimizer = std::move(optimizer.document);
  documents->dynamic_remover = std::move(remover.document);
  auto alignment = std::make_shared<AlignmentArtifactMetadata>();
  alignment->cache_path = cache_path;
  alignment->input_fingerprints = std::move(input_fingerprints).Value();
  alignment->runtime_fingerprint = runtime_fingerprint;

  auto config = std::make_shared<RuntimeConfig>();
  config->root = {data_directories,
                  output,
                  anchor,
                  map_enabled,
                  save_voxel,
                  parallel_load,
                  parallel_map,
                  static_cast<std::size_t>(max_parallel)};
  config->data_loader = std::make_shared<const DataLoaderConfig>(
      std::move(data_config).Value());
  config->loop_detector = std::make_shared<const LoopDetectorConfig>(
      std::move(loop_config).Value());
  config->optimizer = std::make_shared<const OptimizerConfig>(
      std::move(optimizer_config).Value());
  config->dynamic_remover = std::make_shared<const DynamicRemoverConfig>(
      std::move(remover_config).Value());
  config->map_save =
      std::make_shared<const MapSaveConfig>(std::move(map_config).Value());
  config->fingerprint = config_fingerprint;
  config->documents = std::move(documents);
  config->alignment_artifacts = std::move(alignment);
  config->schema_registry = std::move(runtime_registry);

  ArtifactRepository initial_artifacts;
  initial_artifacts.Reset(ordered_agents);
  auto state = std::make_shared<RuntimeState>();
  state->revision = 1;
  state->config = std::move(config);
  state->ordered_agents = ordered_agents;
  state->agent_catalog = std::move(catalog);
  state->payload = std::move(payload);
  state->artifacts = initial_artifacts.Snapshot();

  // Output publication is the final bootstrap step. All config, input, and
  // in-memory state validation must succeed before an existing manifest can be
  // replaced or a new runtime output directory can be created.
  cancelled = CheckCancelled(request, "before agent manifest");
  if (!cancelled) {
    return Result<RuntimeBootstrapResult>::Failure(cancelled.GetError());
  }
  std::error_code directory_error;
  const bool output_existed = fs::exists(output, directory_error);
  if (directory_error) {
    return Result<RuntimeBootstrapResult>::Failure(Error::IoError(
        "failed to inspect output directory " + output.string() + ": " +
        directory_error.message()));
  }
  fs::create_directories(output, directory_error);
  if (directory_error) {
    return Result<RuntimeBootstrapResult>::Failure(Error::IoError(
        "failed to create output directory " + output.string() + ": " +
        directory_error.message()));
  }
  const bool output_is_directory = fs::is_directory(output, directory_error);
  if (directory_error || !output_is_directory) {
    return Result<RuntimeBootstrapResult>::Failure(Error::IoError(
        "failed to access output directory " + output.string() +
        (directory_error ? ": " + directory_error.message() : "")));
  }
  auto manifest = WriteAgentManifest(output, *state->agent_catalog);
  if (!manifest) {
    if (!output_existed &&
        manifest.GetError().severity != Error::Severity::kFatalRuntime) {
      std::error_code ignored;
      fs::remove(output, ignored);
    }
    return Result<RuntimeBootstrapResult>::Failure(manifest.GetError());
  }

  ResourceBudget budget;
  budget.max_agent_tasks = static_cast<std::size_t>(max_parallel);
  budget.max_cpu_threads = static_cast<std::size_t>(max_parallel);
  return Result<RuntimeBootstrapResult>::Ok(
      {std::move(state), budget, algorithms_});
  } catch (const std::exception& error) {
    return Result<RuntimeBootstrapResult>::Failure(
        Error::ParseError(error.what()).WithConfig(
            request.bootstrap_config.ConfigDirectory().string()));
  } catch (...) {
    return Result<RuntimeBootstrapResult>::Failure(
        Error::ParseError("unknown bootstrap exception")
            .WithConfig(request.bootstrap_config.ConfigDirectory().string()));
  }
}

}  // namespace open_lmm
