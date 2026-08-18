#include "session_bootstrapper.hpp"

#include "algorithm_factory.hpp"

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/server/artifact_repository.hpp>
#include <open_lmm/server/file_set_transaction.hpp>
#include <open_lmm/utils/config.hpp>
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
  Config config;
  SessionConfigDocument document;
};

Result<void> CheckCancelled(const SessionBootstrapRequest& request,
                            std::string_view boundary) {
  if (request.cancellation &&
      request.cancellation->IsCancellationRequested()) {
    return Result<void>::Failure(Error::Cancelled(boundary));
  }
  return Result<void>::Ok();
}

Result<LoadedConfig> LoadValidated(ConfigDocumentKind kind,
                                   const fs::path& path) {
  Config source(path.string());
  if (!source.is_valid()) {
    return Result<LoadedConfig>::Failure(
        Error::ParseError(source.error_message()).WithConfig(path.string()));
  }
  auto validated = BuiltinConfigSchemaRegistry().ParseAndValidate(
      kind, source.ToJson(), path.string());
  if (!validated) {
    return Result<LoadedConfig>::Failure(validated.GetError());
  }
  const std::string canonical = validated.Value().CanonicalJson();
  return Result<LoadedConfig>::Ok(
      {Config::FromJson(canonical, path.string()), {path, canonical}});
}

void HashBytes(uint64_t& hash, const char* data, std::size_t size) {
  for (std::size_t index = 0; index < size; ++index) {
    hash ^= static_cast<unsigned char>(data[index]);
    hash *= kFnvPrime;
  }
}

void HashText(uint64_t& hash, const std::string& text) {
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

std::string AlignmentConfigFingerprint(const Config& data_loader,
                                       const Config& loop_detector,
                                       const Config& optimizer,
                                       int anchor_agent_index) {
  uint64_t hash = kFnvOffset;
  HashText(hash, data_loader.ToJson());
  HashText(hash, loop_detector.ToJson());
  HashText(hash, optimizer.ToJson());
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
  return IsFiniteRigidTransform(transform)
             ? std::optional<Eigen::Isometry3d>(transform)
             : std::nullopt;
}

std::map<AgentId, StoredAlignment> LoadAlignmentCache(
    const fs::path& path, const std::string& session_fingerprint,
    const AgentSymbolCatalogHandle& catalog) {
  std::map<AgentId, StoredAlignment> alignments;
  std::ifstream input(path);
  if (!input) return alignments;
  try {
    nlohmann::json root;
    input >> root;
    if (root.value("version", 0) != 3 ||
        root.value("session_fingerprint", std::string()) !=
            session_fingerprint) {
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
      committed.GetError().severity != Error::Severity::kFatalSession) {
    std::error_code ignored;
    fs::remove(temporary, ignored);
  }
  return committed;
}

}  // namespace

SessionBootstrapper::SessionBootstrapper(
    std::shared_ptr<const AlgorithmFactory> algorithms)
    : algorithms_(algorithms ? std::move(algorithms)
                             : std::make_shared<AlgorithmFactory>()) {}

Result<SessionBootstrapResult> SessionBootstrapper::Bootstrap(
    const SessionBootstrapRequest& request) const {
  try {
  auto cancelled = CheckCancelled(request, "before session bootstrap");
  if (!cancelled) {
    return Result<SessionBootstrapResult>::Failure(cancelled.GetError());
  }
  if (request.config_directory.empty()) {
    return Result<SessionBootstrapResult>::Failure(
        Error::InvalidArgument("config directory must be non-empty"));
  }

  auto root_loaded = LoadValidated(ConfigDocumentKind::kRoot,
                                   request.config_directory / "config.json");
  if (!root_loaded) {
    return Result<SessionBootstrapResult>::Failure(root_loaded.GetError());
  }
  LoadedConfig root = std::move(root_loaded).Value();
  const auto config_path = [&](const char* name) {
    return request.config_directory /
           root.config.param<std::string>("global", name,
                                           std::string(name) + ".json");
  };
  for (const char* name : {"config_map_server", "config_data_loader",
                           "config_loop_detector",
                           "config_backend_optimizer",
                           "config_dynamic_remover"}) {
    if (root.config.param_cast<std::string>("global", name).empty()) {
      return Result<SessionBootstrapResult>::Failure(Error::InvalidArgument(
          std::string("global/") + name + " must be non-empty"));
    }
  }

  auto map_loaded =
      LoadValidated(ConfigDocumentKind::kMapServer,
                    config_path("config_map_server"));
  auto data_loaded =
      LoadValidated(ConfigDocumentKind::kDataLoader,
                    config_path("config_data_loader"));
  auto loop_loaded =
      LoadValidated(ConfigDocumentKind::kLoopDetector,
                    config_path("config_loop_detector"));
  auto optimizer_loaded =
      LoadValidated(ConfigDocumentKind::kBackendOptimizer,
                    config_path("config_backend_optimizer"));
  auto remover_loaded =
      LoadValidated(ConfigDocumentKind::kDynamicRemover,
                    config_path("config_dynamic_remover"));
  if (!map_loaded)
    return Result<SessionBootstrapResult>::Failure(map_loaded.GetError());
  if (!data_loaded)
    return Result<SessionBootstrapResult>::Failure(data_loaded.GetError());
  if (!loop_loaded)
    return Result<SessionBootstrapResult>::Failure(loop_loaded.GetError());
  if (!optimizer_loaded)
    return Result<SessionBootstrapResult>::Failure(optimizer_loaded.GetError());
  if (!remover_loaded)
    return Result<SessionBootstrapResult>::Failure(remover_loaded.GetError());
  LoadedConfig map = std::move(map_loaded).Value();
  LoadedConfig data = std::move(data_loaded).Value();
  LoadedConfig loop = std::move(loop_loaded).Value();
  LoadedConfig optimizer = std::move(optimizer_loaded).Value();
  LoadedConfig remover = std::move(remover_loaded).Value();

  auto root_schema = BuiltinConfigSchemaRegistry().ParseAndValidate(
      ConfigDocumentKind::kRoot, root.document.canonical_json,
      root.document.path.string());
  auto map_schema = BuiltinConfigSchemaRegistry().ParseAndValidate(
      ConfigDocumentKind::kMapServer, map.document.canonical_json,
      map.document.path.string());
  if (!root_schema) {
    return Result<SessionBootstrapResult>::Failure(root_schema.GetError());
  }
  if (!map_schema) {
    return Result<SessionBootstrapResult>::Failure(map_schema.GetError());
  }
  auto session_schema = ValidateSessionConfigDocuments(root_schema.Value(),
                                                        map_schema.Value());
  if (!session_schema) {
    return Result<SessionBootstrapResult>::Failure(session_schema.GetError());
  }

  const fs::path root_data = root.config.param_cast<std::string>(
      "directory", "root_dir_path");
  const auto subdirectories = root.config.param_cast<std::vector<std::string>>(
      "directory", "sub_dir_list");
  const fs::path root_output = root.config.param_cast<std::string>(
      "directory", "root_save_dir");
  if (root_data.empty() || root_output.empty() || subdirectories.empty()) {
    return Result<SessionBootstrapResult>::Failure(Error::InvalidArgument(
        "directory root_dir_path, root_save_dir, and sub_dir_list must be non-empty"));
  }

  std::vector<AgentId> configured_agents;
  std::vector<fs::path> data_directories;
  configured_agents.reserve(subdirectories.size());
  data_directories.reserve(subdirectories.size());
  for (const std::string& name : subdirectories) {
    auto agent = AgentId::Parse(name);
    if (!agent) {
      return Result<SessionBootstrapResult>::Failure(agent.GetError());
    }
    const fs::path directory = root_data / name;
    std::error_code error;
    const bool is_directory = fs::is_directory(directory, error);
    if (error) {
      return Result<SessionBootstrapResult>::Failure(Error::IoError(
          "failed to inspect agent directory " + directory.string() + ": " +
          error.message()));
    }
    if (!is_directory) {
      return Result<SessionBootstrapResult>::Failure(
          Error::FileNotFound(directory.string()));
    }
    configured_agents.push_back(std::move(agent).Value());
    data_directories.push_back(directory);
  }
  auto catalog_result = AgentSymbolCatalog::Build(configured_agents);
  if (!catalog_result) {
    return Result<SessionBootstrapResult>::Failure(catalog_result.GetError());
  }
  auto catalog = std::make_shared<const AgentSymbolCatalog>(
      std::move(catalog_result).Value());

  const bool map_enabled = map.config.param<bool>(
      "map_server", "enable_map_updater", true);
  const int anchor = map.config.param<int>("map_server", "anchor_agent_index", 0);
  if (anchor < 0 || anchor >= static_cast<int>(configured_agents.size())) {
    return Result<SessionBootstrapResult>::Failure(Error::InvalidArgument(
        "map_server/anchor_agent_index is outside the configured agent range"));
  }
  const double save_voxel =
      map.config.param<double>("map_server", "save_voxel_size", 0.2);
  if (save_voxel <= 0.0) {
    return Result<SessionBootstrapResult>::Failure(Error::InvalidArgument(
        "map_server/save_voxel_size must be greater than zero"));
  }
  const bool parallel_load = map.config.param<bool>(
      "map_server", "parallel_data_load", false);
  const bool parallel_map = map.config.param<bool>(
      "map_server", "parallel_map_update", false);
  const int max_parallel = map.config.param<int>(
      "map_server", "max_parallel_agents", 1);
  if (max_parallel <= 0) {
    return Result<SessionBootstrapResult>::Failure(Error::InvalidArgument(
        "map_server/max_parallel_agents must be greater than zero"));
  }

  auto data_config = ParseDataLoaderConfig(data.config);
  auto loop_config = ParseLoopDetectorConfig(loop.config);
  auto optimizer_config = ParseOptimizerConfig(optimizer.config);
  auto remover_config = ParseDynamicRemoverConfig(remover.config);
  auto map_config = ParseMapSaveConfig(map.config);
  if (!data_config)
    return Result<SessionBootstrapResult>::Failure(data_config.GetError());
  if (!loop_config)
    return Result<SessionBootstrapResult>::Failure(loop_config.GetError());
  if (!optimizer_config)
    return Result<SessionBootstrapResult>::Failure(optimizer_config.GetError());
  if (!remover_config)
    return Result<SessionBootstrapResult>::Failure(remover_config.GetError());
  if (!map_config)
    return Result<SessionBootstrapResult>::Failure(map_config.GetError());

  auto input_cardinality = ValidateInputCardinality(
      configured_agents, data_directories, data_config.Value());
  if (!input_cardinality) {
    return Result<SessionBootstrapResult>::Failure(
        input_cardinality.GetError());
  }

  auto preflight = algorithms_->Preflight(loop_config.Value(),
                                          remover_config.Value());
  if (!preflight) {
    return Result<SessionBootstrapResult>::Failure(preflight.GetError());
  }
  auto optimizer_instance = algorithms_->CreateOptimizer(optimizer_config.Value());
  if (!optimizer_instance) {
    return Result<SessionBootstrapResult>::Failure(
        optimizer_instance.GetError());
  }

  const std::string config_fingerprint = AlignmentConfigFingerprint(
      data.config, loop.config, optimizer.config, anchor);
  auto input_fingerprints = InputFingerprints(
      configured_agents, data_directories, data_config.Value());
  if (!input_fingerprints) {
    return Result<SessionBootstrapResult>::Failure(
        input_fingerprints.GetError());
  }
  uint64_t session_hash = kFnvOffset;
  HashText(session_hash, config_fingerprint);
  for (const auto& [agent, fingerprint] : input_fingerprints.Value()) {
    HashText(session_hash, agent.Value());
    HashText(session_hash, fingerprint);
  }
  const std::string session_fingerprint = HexFingerprint(session_hash);
  const fs::path cache_root =
      request.output_directory ? *request.output_directory : root_output;
  const fs::path cache_path = cache_root / "map_alignment_cache.json";

  const fs::path output = request.output_directory
                              ? *request.output_directory
                              : root_output / root.config.create_date();
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
      LoadAlignmentCache(cache_path, session_fingerprint, catalog);
  auto payload = std::make_shared<SessionPayload>();
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

  auto documents = std::make_shared<SessionConfigDocuments>();
  documents->root = std::move(root.document);
  documents->map_server = std::move(map.document);
  documents->data_loader = std::move(data.document);
  documents->loop_detector = std::move(loop.document);
  documents->optimizer = std::move(optimizer.document);
  documents->dynamic_remover = std::move(remover.document);
  auto alignment = std::make_shared<AlignmentArtifactMetadata>();
  alignment->cache_path = cache_path;
  alignment->input_fingerprints = std::move(input_fingerprints).Value();
  alignment->session_fingerprint = session_fingerprint;

  auto config = std::make_shared<SessionConfig>();
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

  ArtifactRepository initial_artifacts;
  initial_artifacts.Reset(ordered_agents);
  auto state = std::make_shared<SessionState>();
  state->revision = 1;
  state->config = std::move(config);
  state->ordered_agents = ordered_agents;
  state->agent_catalog = std::move(catalog);
  state->payload = std::move(payload);
  state->artifacts = initial_artifacts.Snapshot();

  // Output publication is the final bootstrap step. All config, input, and
  // in-memory state validation must succeed before an existing manifest can be
  // replaced or a new runtime-session directory can be created.
  cancelled = CheckCancelled(request, "before agent manifest");
  if (!cancelled) {
    return Result<SessionBootstrapResult>::Failure(cancelled.GetError());
  }
  std::error_code directory_error;
  const bool output_existed = fs::exists(output, directory_error);
  if (directory_error) {
    return Result<SessionBootstrapResult>::Failure(Error::IoError(
        "failed to inspect output directory " + output.string() + ": " +
        directory_error.message()));
  }
  fs::create_directories(output, directory_error);
  if (directory_error) {
    return Result<SessionBootstrapResult>::Failure(Error::IoError(
        "failed to create output directory " + output.string() + ": " +
        directory_error.message()));
  }
  const bool output_is_directory = fs::is_directory(output, directory_error);
  if (directory_error || !output_is_directory) {
    return Result<SessionBootstrapResult>::Failure(Error::IoError(
        "failed to access output directory " + output.string() +
        (directory_error ? ": " + directory_error.message() : "")));
  }
  auto manifest = WriteAgentManifest(output, *state->agent_catalog);
  if (!manifest) {
    if (!output_existed &&
        manifest.GetError().severity != Error::Severity::kFatalSession) {
      std::error_code ignored;
      fs::remove(output, ignored);
    }
    return Result<SessionBootstrapResult>::Failure(manifest.GetError());
  }

  ResourceBudget budget;
  budget.max_active_sessions = 1;
  budget.max_agent_tasks = static_cast<std::size_t>(max_parallel);
  budget.max_cpu_threads = static_cast<std::size_t>(max_parallel);
  return Result<SessionBootstrapResult>::Ok(
      {std::move(state), budget});
  } catch (const std::exception& error) {
    return Result<SessionBootstrapResult>::Failure(
        Error::ParseError(error.what()).WithConfig(
            request.config_directory.string()));
  } catch (...) {
    return Result<SessionBootstrapResult>::Failure(
        Error::ParseError("unknown bootstrap exception")
            .WithConfig(request.config_directory.string()));
  }
}

}  // namespace open_lmm
