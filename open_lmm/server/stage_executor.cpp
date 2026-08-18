
#include "stage_executor.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/profiling.hpp>
#include <open_lmm/utils/config.hpp>
#include <open_lmm/utils/load_module.hpp>
#include <open_lmm/utils/logging.hpp>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <nlohmann/json.hpp>
#include <sstream>

// Pipeline nodes
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <open_lmm/server/nodes/data_load_node.hpp>
#include <open_lmm/server/nodes/loop_detect_node.hpp>
#include <open_lmm/server/nodes/map_update_node.hpp>
#include <open_lmm/server/nodes/optimize_node.hpp>
#include <open_lmm/server/file_set_transaction.hpp>

namespace open_lmm {
namespace {
class ExecutionLease {
 public:
  explicit ExecutionLease(std::atomic_flag& active)
      : active_(active), acquired_(!active_.test_and_set(std::memory_order_acquire)) {}
  ~ExecutionLease() {
    if (acquired_) active_.clear(std::memory_order_release);
  }
  [[nodiscard]] explicit operator bool() const { return acquired_; }

 private:
  std::atomic_flag& active_;
  bool acquired_;
};

void HashBytes(uint64_t& hash, const char* data, std::size_t size) {
  constexpr uint64_t kFnvPrime = 1099511628211ULL;
  for (std::size_t i = 0; i < size; ++i) {
    hash ^= static_cast<unsigned char>(data[i]);
    hash *= kFnvPrime;
  }
}

void HashText(uint64_t& hash, const std::string& text) {
  HashBytes(hash, text.data(), text.size());
}

bool HashFile(uint64_t& hash, const fs::path& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) return false;
  char buffer[8192];
  while (input.read(buffer, sizeof(buffer)) || input.gcount() > 0) {
    HashBytes(hash, buffer, static_cast<std::size_t>(input.gcount()));
  }
  return true;
}

std::string HexFingerprint(uint64_t hash) {
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

std::optional<Eigen::Isometry3d> MatrixFromJson(const nlohmann::json& value) {
  if (!value.is_array() || value.size() != 16) return std::nullopt;
  Eigen::Matrix4d matrix;
  for (int row = 0; row < 4; ++row) {
    for (int col = 0; col < 4; ++col) {
      if (!value[row * 4 + col].is_number()) return std::nullopt;
      matrix(row, col) = value[row * 4 + col].get<double>();
    }
  }
  Eigen::Isometry3d transform(matrix);
  return IsFiniteRigidTransform(transform)
             ? std::optional<Eigen::Isometry3d>(transform)
             : std::nullopt;
}
}  // namespace

StageExecutor::StageExecutor() {
  InitializeLogging();
  try {
    parseConfig();
  } catch (const std::exception& e) {
    initialization_error_ = Error::ParseError(e.what());
  }
}
StageExecutor::~StageExecutor() {}

void StageExecutor::parseConfig() {
  auto* global = GlobalConfig::instance();
  if (!global->is_valid()) {
    initialization_error_ = Error::ParseError(global->error_message());
    return;
  }
  for (const char* config_name : {"config_map_server", "config_data_loader",
                                  "config_loop_detector",
                                  "config_backend_optimizer",
                                  "config_dynamic_remover"}) {
    const auto path = global->param_cast<std::string>("global", config_name);
    if (path.empty()) {
      initialization_error_ = Error::InvalidArgument(
          std::string("global/") + config_name + " must be non-empty");
      return;
    }
  }

  const fs::path root_data_dir = global->param_cast<std::string>(
      "directory", "root_dir_path");
  const auto sub_dirs = global->param_cast<std::vector<std::string>>(
      "directory", "sub_dir_list");
  const fs::path root_save_dir = global->param_cast<std::string>(
      "directory", "root_save_dir");
  if (root_data_dir.empty() || root_save_dir.empty() || sub_dirs.empty()) {
    initialization_error_ = Error::InvalidArgument(
        "directory root_dir_path, root_save_dir, and sub_dir_list must be non-empty");
    return;
  }
  for (const std::string& sub_dir : sub_dirs) {
    if (sub_dir.empty()) {
      initialization_error_ = Error::InvalidArgument(
          "directory/sub_dir_list must not contain empty agent paths");
      return;
    }
    const fs::path data_dir = root_data_dir / sub_dir;
    if (!fs::is_directory(data_dir)) {
      initialization_error_ = Error::FileNotFound(data_dir.string());
      return;
    }
    data_dir_list_.push_back(data_dir);
  }
  agent_num_ = data_dir_list_.size();

  output_save_dir_ = (root_save_dir / global->date).string();
  std::error_code directory_error;
  fs::create_directories(output_save_dir_, directory_error);
  if (directory_error || !fs::is_directory(output_save_dir_)) {
    initialization_error_ = Error::IoError(
        "failed to create output directory " + output_save_dir_ + ": " +
        directory_error.message());
    return;
  }

  config_map_server_ =
      Config(GlobalConfig::get_global_config_path("config_map_server"));
  if (!config_map_server_->is_valid()) {
    initialization_error_ = Error::ParseError(config_map_server_->error_message());
    return;
  }
  enable_map_updater_ =
      config_map_server_->param<bool>("map_server", "enable_map_updater", true);
  anchor_agent_index_ =
      config_map_server_->param<int>("map_server", "anchor_agent_index", 0);
  if (anchor_agent_index_ < 0 || anchor_agent_index_ >= agent_num_) {
    initialization_error_ = Error::InvalidArgument(
        "map_server/anchor_agent_index is outside the configured agent range");
    return;
  }
  agent_ids_.clear();
  agent_ids_.reserve(data_dir_list_.size());
  agent_ids_.push_back(static_cast<char>('A' + anchor_agent_index_));
  for (int i = 0; i < agent_num_; ++i) {
    if (i != anchor_agent_index_) agent_ids_.push_back(static_cast<char>('A' + i));
  }
  save_voxel_size_ =
      config_map_server_->param<double>("map_server", "save_voxel_size", 0.2);
  if (save_voxel_size_ <= 0.0) {
    initialization_error_ = Error::InvalidArgument(
        "map_server/save_voxel_size must be greater than zero");
    return;
  }

  config_data_loader_ =
      Config(GlobalConfig::get_global_config_path("config_data_loader"));
  if (!config_data_loader_->is_valid()) {
    initialization_error_ = Error::ParseError(config_data_loader_->error_message());
    return;
  }
  config_loop_detector_ =
      Config(GlobalConfig::get_global_config_path("config_loop_detector"));
  if (!config_loop_detector_->is_valid()) {
    initialization_error_ = Error::ParseError(config_loop_detector_->error_message());
    return;
  }
  config_dynamic_remover_ =
      Config(GlobalConfig::get_global_config_path("config_dynamic_remover"));
  if (!config_dynamic_remover_->is_valid()) {
    initialization_error_ = Error::ParseError(
        config_dynamic_remover_->error_message());
    return;
  }

  config_backend_optimizer_ =
      Config(GlobalConfig::get_global_config_path("config_backend_optimizer"));
  if (!config_backend_optimizer_->is_valid()) {
    initialization_error_ = Error::ParseError(
        config_backend_optimizer_->error_message());
    return;
  }
  auto data_loader_config = ParseDataLoaderConfig(*config_data_loader_);
  if (!data_loader_config) {
    initialization_error_ = data_loader_config.GetError();
    return;
  }
  auto loop_detector_config = ParseLoopDetectorConfig(*config_loop_detector_);
  if (!loop_detector_config) {
    initialization_error_ = loop_detector_config.GetError();
    return;
  }
  auto dynamic_remover_config =
      ParseDynamicRemoverConfig(*config_dynamic_remover_);
  if (!dynamic_remover_config) {
    initialization_error_ = dynamic_remover_config.GetError();
    return;
  }
  auto optimizer_config = ParseOptimizerConfig(*config_backend_optimizer_);
  if (!optimizer_config) {
    initialization_error_ = optimizer_config.GetError();
    return;
  }
  auto map_save_config = ParseMapSaveConfig(*config_map_server_);
  if (!map_save_config) {
    initialization_error_ = map_save_config.GetError();
    return;
  }
  auto descriptor_plugin = inspect_plugin_v1(
      "libcreate_" + loop_detector_config.Value().model + ".so",
      "descriptor");
  if (!descriptor_plugin) {
    initialization_error_ = descriptor_plugin.GetError();
    return;
  }
  const std::string remover_kind =
      dynamic_remover_config.Value().type == "online"
          ? "dynamic_remover_online"
          : "dynamic_remover_offline";
  auto remover_plugin = inspect_plugin_v1(
      "libcreate_" + dynamic_remover_config.Value().model + ".so",
      remover_kind);
  if (!remover_plugin) {
    initialization_error_ = remover_plugin.GetError();
    return;
  }
  auto optimizer_result =
      BackendOptimizerBase::createInstance(optimizer_config.Value());
  if (!optimizer_result) {
    initialization_error_ = optimizer_result.GetError();
    return;
  }
  auto optimizer = std::shared_ptr<BackendOptimizerBase>(
      std::move(optimizer_result).Value());
  computeAlignmentFingerprints();
  loadAlignmentCache();

  auto database = std::make_shared<SharedDatabase>();
  installStoredAlignments(*database);
  auto payload = std::make_shared<SessionPayload>();
  payload->contexts = buildContexts();
  payload->database = std::move(database);
  payload->optimizer = std::move(optimizer);

  auto session_config = std::make_shared<SessionConfig>();
  session_config->root.data_directories = data_dir_list_;
  session_config->root.output_directory = output_save_dir_;
  session_config->root.anchor_agent_index = anchor_agent_index_;
  session_config->root.enable_map_updater = enable_map_updater_;
  session_config->root.save_voxel_size = save_voxel_size_;
  session_config->data_loader =
      std::make_shared<const DataLoaderConfig>(
          std::move(data_loader_config).Value());
  session_config->loop_detector =
      std::make_shared<const LoopDetectorConfig>(
          std::move(loop_detector_config).Value());
  session_config->optimizer =
      std::make_shared<const OptimizerConfig>(
          std::move(optimizer_config).Value());
  session_config->dynamic_remover =
      std::make_shared<const DynamicRemoverConfig>(
          std::move(dynamic_remover_config).Value());
  session_config->map_save =
      std::make_shared<const MapSaveConfig>(
          std::move(map_save_config).Value());
  session_config->fingerprint = alignment_config_fingerprint_;

  ArtifactRepository initial_artifacts;
  initial_artifacts.Reset(agent_ids_);
  auto state = std::make_shared<SessionState>();
  state->revision = 1;
  state->config = std::move(session_config);
  state->ordered_agents = agent_ids_;
  state->payload = std::move(payload);
  state->artifacts = initial_artifacts.Snapshot();
  session_manager_.Initialize(std::move(state));
}

void StageExecutor::computeAlignmentFingerprints() {
  constexpr uint64_t kFnvOffset = 14695981039346656037ULL;
  uint64_t config_hash = kFnvOffset;
  for (const char* name : {"config_map_server", "config_data_loader",
                           "config_loop_detector", "config_backend_optimizer",
                           "config_dynamic_remover"}) {
    const auto path = GlobalConfig::get_global_config_path(name);
    HashText(config_hash, path);
    HashFile(config_hash, path);
  }
  HashText(config_hash, std::to_string(anchor_agent_index_));
  alignment_config_fingerprint_ = HexFingerprint(config_hash);

  alignment_input_fingerprints_.clear();
  const auto pose_name = config_data_loader_->param<std::string>(
      "data_loader", "pose_file_name", "");
  const auto scan_dir_name = config_data_loader_->param<std::string>(
      "data_loader", "scan_dir_name", "");
  const auto scan_type = config_data_loader_->param<std::string>(
      "data_loader", "scan_type", "");
  for (std::size_t i = 0; i < data_dir_list_.size(); ++i) {
    uint64_t input_hash = kFnvOffset;
    HashText(input_hash, data_dir_list_[i].string());
    HashFile(input_hash, data_dir_list_[i] / pose_name);
    std::vector<fs::path> scans;
    const fs::path scan_dir = data_dir_list_[i] / scan_dir_name;
    for (const auto& entry : fs::directory_iterator(scan_dir)) {
      if (entry.is_regular_file() &&
          entry.path().extension() == "." + scan_type) {
        scans.push_back(entry.path());
      }
    }
    std::sort(scans.begin(), scans.end());
    for (const auto& scan : scans) {
      std::error_code error;
      HashText(input_hash, scan.filename().string());
      HashText(input_hash, std::to_string(fs::file_size(scan, error)));
      error.clear();
      const auto modified = fs::last_write_time(scan, error);
      if (!error) HashText(input_hash, std::to_string(modified.time_since_epoch().count()));
    }
    alignment_input_fingerprints_[static_cast<char>('A' + i)] =
        HexFingerprint(input_hash);
  }
  uint64_t session_hash = config_hash;
  for (const auto& [agent, fingerprint] : alignment_input_fingerprints_) {
    HashText(session_hash, std::string(1, agent));
    HashText(session_hash, fingerprint);
  }
  alignment_session_fingerprint_ = HexFingerprint(session_hash);
  const auto root_save_dir = GlobalConfig::instance()->param<std::string>(
      "directory", "root_save_dir", "");
  alignment_cache_path_ = fs::path(root_save_dir) / "map_alignment_cache.json";
}

void StageExecutor::loadAlignmentCache() {
  cached_alignments_.clear();
  std::ifstream input(alignment_cache_path_);
  if (!input) return;
  try {
    nlohmann::json root;
    input >> root;
    if (root.value("version", 0) != 2 ||
        root.value("session_fingerprint", std::string()) !=
            alignment_session_fingerprint_) {
      return;
    }
    for (const auto& item : root.at("alignments")) {
      if (item.value("approval", std::string()) != "user") continue;
      const auto source = item.value("source_agent", std::string());
      const auto target = item.value("target_agent", std::string());
      if (source.size() != 1 || target.size() != 1) continue;
      auto transform = MatrixFromJson(item["accepted_global_T_agent"]);
      if (!transform) continue;
      StoredAlignment stored;
      stored.proposal.source_agent = source.front();
      stored.proposal.target_agent = target.front();
      const auto method = item.value("method", std::string());
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
      cached_alignments_[source.front()] = std::move(stored);
    }
  } catch (const std::exception&) {
    cached_alignments_.clear();
  }
}

void StageExecutor::installStoredAlignments(SharedDatabase& database) const {
  database.stored_alignments = cached_alignments_;
}

std::vector<AgentPipelineCtx> StageExecutor::buildContexts() const {
  std::vector<AgentPipelineCtx> ctxs;
  for (int i = 0; i < agent_num_; i++) {
    AgentPipelineCtx ctx;
    ctx.agent = {
        .id    = static_cast<char>('A' + i),
        .role  = (i == anchor_agent_index_) ? AgentRole::kAnchor
                                            : AgentRole::kFollower,
        .order = i,
    };
    ctx.data_dir = data_dir_list_[i];
    ctx.cancellation = cancellation_;
    ctxs.push_back(std::move(ctx));
  }
  // anchor를 앞으로 (stable_sort로 follower 순서 유지)
  std::stable_sort(ctxs.begin(), ctxs.end(),
                   [](const AgentPipelineCtx& a, const AgentPipelineCtx& b) {
                     return static_cast<int>(a.agent.role) <
                            static_cast<int>(b.agent.role);
                   });
  return ctxs;
}

Result<void> StageExecutor::process() {
  OPEN_LMM_ZONE_N("MapServer.Process");
  OPEN_LMM_PLOT("agent.count", agent_num_);
  for (StageId stage : PipelineStages()) {
    auto result = RunStage(stage);
    if (!result) return result;
  }
  LogInfo("[pipeline] all stages completed");
  return Result<void>::Ok();
}

Result<void> StageExecutor::ensureReady() {
  if (initialization_error_) {
    Error fatal = *initialization_error_;
    fatal.MarkFatalSession();
    return Result<void>::Failure(std::move(fatal));
  }
  if (agent_num_ <= 0) {
    return Result<void>::Failure(Error::InvalidArgument("No agents configured"));
  }
  if (agent_num_ > 26) {
    return Result<void>::Failure(Error::InvalidArgument(
        "At most 26 agents are supported by character agent IDs"));
  }
  if (anchor_agent_index_ < 0 || anchor_agent_index_ >= agent_num_) {
    return Result<void>::Failure(Error::InvalidArgument(
        "anchor_agent_index " + std::to_string(anchor_agent_index_) +
        " is out of range for " + std::to_string(agent_num_) + " agents"));
  }
  if (!inputs_validated_) {
    auto inputs = validateInputFiles();
    if (!inputs) return inputs;
    inputs_validated_ = true;
  }
  if (!committedState()) {
    return Result<void>::Failure(
        Error::InvalidArgument("session state is unavailable"));
  }
  return Result<void>::Ok();
}

Result<void> StageExecutor::validateInputFiles() const {
  if (!config_data_loader_) {
    return Result<void>::Failure(Error::InvalidArgument(
        "data loader config is unavailable"));
  }
  const auto pose_name = config_data_loader_->param<std::string>(
      "data_loader", "pose_file_name", "");
  const auto scan_dir_name = config_data_loader_->param<std::string>(
      "data_loader", "scan_dir_name", "");
  const auto scan_type = config_data_loader_->param<std::string>(
      "data_loader", "scan_type", "");
  for (std::size_t i = 0; i < data_dir_list_.size(); ++i) {
    const char agent = static_cast<char>('A' + i);
    const auto pose_path = data_dir_list_[i] / pose_name;
    const auto scan_path = data_dir_list_[i] / scan_dir_name;
    if (!fs::is_regular_file(pose_path)) {
      return Result<void>::Failure(Error::FileNotFound(
          "agent " + std::string(1, agent) + " pose " + pose_path.string()));
    }
    if (!fs::is_directory(scan_path)) {
      return Result<void>::Failure(Error::FileNotFound(
          "agent " + std::string(1, agent) + " scans " + scan_path.string()));
    }
    std::ifstream poses(pose_path);
    std::size_t pose_count = 0;
    for (std::string line; std::getline(poses, line);)
      if (!line.empty()) ++pose_count;
    std::size_t scan_count = 0;
    for (const auto& entry : fs::directory_iterator(scan_path)) {
      if (entry.is_regular_file() &&
          entry.path().extension() == "." + scan_type) ++scan_count;
    }
    if (pose_count == 0 || scan_count == 0 || pose_count != scan_count) {
      return Result<void>::Failure(Error::InvalidArgument(
          "agent " + std::string(1, agent) + " input count mismatch: poses=" +
          std::to_string(pose_count) + ", scans=" + std::to_string(scan_count)));
    }
  }
  return Result<void>::Ok();
}

Result<void> StageExecutor::ValidateReady() {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<void>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  return ensureReady();
}

Result<void> StageExecutor::Reconfigure(ConfigDomain domain, uint64_t revision) {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<void>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  const auto config_directory = GlobalConfig::config_directory();
  if (config_directory.empty()) {
    return Result<void>::Failure(
        Error::InvalidArgument("global config directory is unavailable"));
  }
  auto reloaded = GlobalConfig::reload(config_directory);
  if (!reloaded) return reloaded;
  const auto base = committedState();
  SessionTransaction transaction(base);
  auto next_config = std::make_shared<SessionConfig>(*base->config);
  if (revision <= base->config->revision) {
    return Result<void>::Failure(
        Error::InvalidArgument("config revision must increase"));
  }
  next_config->revision = revision;

  if (domain == ConfigDomain::kLoopDetector ||
      domain == ConfigDomain::kOptimizer) {
    Config loop_config(
        GlobalConfig::get_global_config_path("config_loop_detector"));
    if (!loop_config.is_valid()) {
      return Result<void>::Failure(
          Error::ParseError(loop_config.error_message()));
    }
    Config optimizer_config(
        GlobalConfig::get_global_config_path("config_backend_optimizer"));
    if (!optimizer_config.is_valid()) {
      return Result<void>::Failure(
          Error::ParseError(optimizer_config.error_message()));
    }
    auto typed_loop_config = ParseLoopDetectorConfig(loop_config);
    if (!typed_loop_config) {
      return Result<void>::Failure(typed_loop_config.GetError());
    }
    auto typed_optimizer_config = ParseOptimizerConfig(optimizer_config);
    if (!typed_optimizer_config) {
      return Result<void>::Failure(typed_optimizer_config.GetError());
    }
    auto optimizer = BackendOptimizerBase::createInstance(
        typed_optimizer_config.Value());
    if (!optimizer) return Result<void>::Failure(optimizer.GetError());

    config_loop_detector_ = std::move(loop_config);
    config_backend_optimizer_ = std::move(optimizer_config);
    next_config->loop_detector =
        std::make_shared<const LoopDetectorConfig>(
            std::move(typed_loop_config).Value());
    next_config->optimizer =
        std::make_shared<const OptimizerConfig>(
            std::move(typed_optimizer_config).Value());
    auto contexts = base->payload->contexts;
    std::shared_ptr<SharedDatabase> database;
    if (domain == ConfigDomain::kLoopDetector) {
      database = std::make_shared<SharedDatabase>();
      database->raw_data = base->payload->database->raw_data;
      for (auto& context : contexts) context.loop_output.reset();
    } else {
      database = std::make_shared<SharedDatabase>(
          *base->payload->database);
      database->optimized_data.clear();
    }
    computeAlignmentFingerprints();
    cached_alignments_.clear();
    loadAlignmentCache();
    installStoredAlignments(*database);
    auto payload = std::make_shared<SessionPayload>();
    payload->contexts = std::move(contexts);
    payload->database = std::move(database);
    payload->optimizer = std::shared_ptr<BackendOptimizerBase>(
        std::move(optimizer).Value());
    next_config->fingerprint = alignment_config_fingerprint_;
    transaction.Working().config = std::move(next_config);
    transaction.SetPayload(std::move(payload));
    auto artifacts = artifactEditor(*base);
    artifacts->ApplyConfig(domain, transaction.Working().config->revision);
    transaction.Working().artifacts = artifacts->Snapshot();
    auto committed = commitTransaction(
        std::move(transaction), nullptr, false);
    if (!committed) return committed;
    publishEmptyVisualizationState(committedState()->revision);
    return Result<void>::Ok();
  }

  if (domain == ConfigDomain::kDynamicRemover ||
      domain == ConfigDomain::kMapSave) {
    Config remover_config(
        GlobalConfig::get_global_config_path("config_dynamic_remover"));
    if (!remover_config.is_valid()) {
      return Result<void>::Failure(
          Error::ParseError(remover_config.error_message()));
    }
    auto typed_remover_config = ParseDynamicRemoverConfig(remover_config);
    if (!typed_remover_config) {
      return Result<void>::Failure(typed_remover_config.GetError());
    }
    config_dynamic_remover_ = std::move(remover_config);
    Config map_config(
        GlobalConfig::get_global_config_path("config_map_server"));
    if (!map_config.is_valid()) {
      return Result<void>::Failure(
          Error::ParseError(map_config.error_message()));
    }
    auto typed_map_config = ParseMapSaveConfig(map_config);
    if (!typed_map_config) {
      return Result<void>::Failure(typed_map_config.GetError());
    }
    const double save_voxel_size =
        map_config.param<double>("map_server", "save_voxel_size", 0.2);
    if (save_voxel_size <= 0.0) {
      return Result<void>::Failure(Error::InvalidArgument(
          "map_server/save_voxel_size must be greater than zero"));
    }
    const bool enable_map_updater =
        map_config.param<bool>("map_server", "enable_map_updater", true);
    config_map_server_ = std::move(map_config);
    save_voxel_size_ = save_voxel_size;
    enable_map_updater_ = enable_map_updater;
    next_config->map_save =
        std::make_shared<const MapSaveConfig>(
            std::move(typed_map_config).Value());
    next_config->dynamic_remover =
        std::make_shared<const DynamicRemoverConfig>(
            std::move(typed_remover_config).Value());
    next_config->root.save_voxel_size = save_voxel_size_;
    next_config->root.enable_map_updater = enable_map_updater_;
    transaction.Working().config = std::move(next_config);
    auto artifacts = artifactEditor(*base);
    artifacts->ApplyConfig(domain, transaction.Working().config->revision);
    transaction.Working().artifacts = artifacts->Snapshot();
    auto committed = commitTransaction(
        std::move(transaction), nullptr, false);
    if (!committed) return committed;
    publishEmptyVisualizationState(committedState()->revision);
    return Result<void>::Ok();
  }

  return Result<void>::Failure(Error::InvalidArgument(
      "data/global config requires a new pipeline session"));
}

std::vector<char> StageExecutor::AgentIds() const {
  return agent_ids_;
}

std::shared_ptr<const SessionState> StageExecutor::committedState() const {
  return session_manager_.Snapshot();
}

std::optional<CommittedSessionSnapshot> StageExecutor::SessionSnapshot() const {
  const auto state = committedState();
  if (!state) return std::nullopt;
  CommittedSessionSnapshot snapshot{
      state->revision,
      state->config ? state->config->revision : 0,
      state->ordered_agents,
      state->artifacts,
  };
  if (state->payload && state->payload->database) {
    const auto& descriptors = state->payload->database->descriptor_store;
    snapshot.descriptor_count =
        descriptors.total_db ? descriptors.total_db->getSize() : 0;
    for (const auto& [agent, database] : descriptors.per_agent_db) {
      snapshot.per_agent_descriptor_count[agent] =
          database ? database->getSize() : 0;
    }
  }
  return snapshot;
}

Result<std::shared_ptr<BackendOptimizerBase>> StageExecutor::createOptimizer() const {
  const auto state = committedState();
  if (!state || !state->config || !state->config->optimizer) {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Failure(
        Error::InvalidArgument("optimizer session config is unavailable"));
  }
  auto created = BackendOptimizerBase::createInstance(*state->config->optimizer);
  if (!created) {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Failure(
        created.GetError());
  }
  return Result<std::shared_ptr<BackendOptimizerBase>>::Ok(
      std::shared_ptr<BackendOptimizerBase>(std::move(created).Value()));
}

std::unique_ptr<ArtifactRepository> StageExecutor::artifactEditor(
    const SessionState& state) const {
  auto editor = std::make_unique<ArtifactRepository>();
  editor->Reset(state.ordered_agents);
  editor->Restore(state.artifacts);
  return editor;
}

Result<void> StageExecutor::commitTransaction(SessionTransaction transaction,
                                          PendingOutputSet* pending,
                                          bool check_cancellation) {
  const auto base = transaction.Base();
  std::shared_ptr<CancellationToken> cancellation;
  if (check_cancellation) {
    std::lock_guard lock(state_mutex_);
    cancellation = cancellation_;
  }
  auto finalized = std::move(transaction).Finalize(cancellation);
  if (!finalized) return Result<void>::Failure(finalized.GetError());
  auto candidate = std::move(finalized).Value();
  if (!session_manager_.Matches(base)) {
    return Result<void>::Failure(
        Error::InvalidArgument("session transaction revision conflict"));
  }
  if (pending) {
    auto files = pending->Commit();
    if (!files) return files;
  }
  // execution_active_ serializes writers, so the successful conflict check
  // above remains valid while the recoverable file-set barrier runs.
  return session_manager_.Commit(base, std::move(candidate));
}

void StageExecutor::SetCancellationToken(
    std::shared_ptr<CancellationToken> token) {
  std::lock_guard lock(state_mutex_);
  cancellation_ = std::move(token);
}

void StageExecutor::SetAlignmentFeedbackBroker(
    std::shared_ptr<AlignmentFeedbackBroker> broker) {
  std::lock_guard lock(state_mutex_);
  alignment_feedback_ = std::move(broker);
}

Result<void> StageExecutor::RunStage(StageId stage) {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<void>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  auto ready = ensureReady();
  if (!ready) return ready;
  Result<void> result = Result<void>::Failure(
      Error::InvalidArgument("unknown stage"));
  switch (stage) {
    case StageId::kDataLoad: result = runDataLoadStage(); break;
    case StageId::kAlignment: result = runAlignmentStage(); break;
    case StageId::kMapUpdate: result = runMapUpdateStage(); break;
    case StageId::kSave: result = runSaveStage(); break;
  }
  if (!result) return result;
  if (stage == StageId::kDataLoad) {
    publishEmptyVisualizationState(committedState()->revision);
  } else {
    publishVisualizationState(
        committedState(), stage == StageId::kMapUpdate || stage == StageId::kSave);
  }
  return result;
}

Result<void> StageExecutor::RunNode(NodeId node, std::optional<char> agent) {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<void>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  auto ready = ensureReady();
  if (!ready) return ready;
  if (!agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("node execution requires an agent"));
  }
  const auto base = committedState();
  auto it = std::find_if(base->payload->contexts.begin(),
                         base->payload->contexts.end(),
                         [agent](const AgentPipelineCtx& ctx) {
                           return ctx.agent.id == *agent;
                         });
  if (it == base->payload->contexts.end()) {
    return Result<void>::Failure(Error::InvalidArgument("unknown agent"));
  }

  if (node == NodeId::kDataLoad) {
    SessionTransaction transaction(base);
    auto optimizer = createOptimizer();
    if (!optimizer) return Result<void>::Failure(optimizer.GetError());
    auto database = std::make_shared<SharedDatabase>(
        *base->payload->database);
    std::shared_ptr<CancellationToken> cancellation;
    {
      std::lock_guard lock(state_mutex_);
      cancellation = cancellation_;
      database->alignment_feedback = alignment_feedback_;
    }
    auto contexts = base->payload->contexts;
    auto working = std::find_if(
        contexts.begin(), contexts.end(), [agent](const AgentPipelineCtx& ctx) {
          return ctx.agent.id == *agent;
        });
    working->flow = ControlFlow::kContinue;
    working->cancellation = cancellation;
    auto loader = DataLoaderBase::createInstance(*base->config->data_loader);
    if (!loader) return Result<void>::Failure(loader.GetError());
    DataLoadNode load_node(std::move(loader).Value());
    auto result = load_node.Process(*working, *database);
    if (!result) return Result<void>::Failure(result.GetError());
    const auto changed = std::distance(contexts.begin(), working);
    for (std::size_t i = static_cast<std::size_t>(changed);
         i < contexts.size(); ++i) {
      contexts[i].loop_output.reset();
      database->optimized_data.erase(contexts[i].agent.id);
    }
    auto payload = std::make_shared<SessionPayload>();
    payload->contexts = std::move(contexts);
    payload->database = std::move(database);
    payload->optimizer = std::move(optimizer).Value();
    transaction.SetPayload(std::move(payload));
    auto artifacts = artifactEditor(*base);
    artifacts->BeginNode(node, agent);
    artifacts->CompleteNode(node, agent);
    transaction.Working().artifacts = artifacts->Snapshot();
    auto committed = commitTransaction(std::move(transaction));
    if (!committed) return committed;
    publishEmptyVisualizationState(committedState()->revision);
    return Result<void>::Ok();
  }
  if (node == NodeId::kLoopDetect) {
    auto committed = runLoopDetectThrough(*agent);
    if (!committed) return committed;
    publishEmptyVisualizationState(committedState()->revision);
    return Result<void>::Ok();
  }
  if (node == NodeId::kOptimize) {
    auto result = runOptimizeThrough(*agent);
    if (result) publishVisualizationState(committedState(), false);
    return result;
  }
  if (node == NodeId::kMapUpdate) {
    if (base->payload->database->optimized_data.find(*agent) ==
        base->payload->database->optimized_data.end()) {
      return Result<void>::Failure(
          Error::InvalidArgument("OptimizedPoses is required for MapUpdate"));
    }
    SessionTransaction transaction(base);
    auto loader = DataLoaderBase::createInstance(*base->config->data_loader);
    if (!loader) return Result<void>::Failure(loader.GetError());
    auto database = std::make_shared<SharedDatabase>();
    database->raw_data = base->payload->database->raw_data;
    database->optimized_data = base->payload->database->optimized_data;
    auto context = *it;
    {
      std::lock_guard lock(state_mutex_);
      context.cancellation = cancellation_;
    }
    context.flow = ControlFlow::kContinue;
    MapUpdateNode update_node(
        std::move(loader).Value(),
        [cfg = base->config->dynamic_remover]() {
          return DynamicRemoverBase::createInstance(*cfg);
        },
        output_save_dir_, save_voxel_size_, true);
    auto pending = output_repository_.Begin();
    const fs::path destination = fs::path(output_save_dir_) /
        ("global_map_" + std::string{*agent} + ".pcd");
    pending.Add(destination.string() + ".tmp", destination);
    auto result = update_node.Process(context, *database);
    if (!result) return Result<void>::Failure(result.GetError());
    auto artifacts = artifactEditor(*base);
    artifacts->BeginNode(node, agent);
    artifacts->CompleteNode(node, agent);
    uint64_t hash = 14695981039346656037ULL;
    HashFile(hash, destination.string() + ".tmp");
    for (ArtifactType type : {ArtifactType::kGlobalMap,
                              ArtifactType::kPcdFile}) {
      artifacts->RecordExternalFile(type, *agent, destination.string(),
                                    HexFingerprint(hash));
    }
    transaction.Working().artifacts = artifacts->Snapshot();
    auto committed = commitTransaction(std::move(transaction), &pending);
    if (!committed) return committed;
    publishVisualizationState(committedState(), true);
    return Result<void>::Ok();
  }
  if (node == NodeId::kPoseSave) {
    SessionTransaction transaction(base);
    auto pending = output_repository_.Begin();
    auto artifacts = artifactEditor(*base);
    artifacts->BeginNode(node, agent);
    auto prepared = prepareOptimizedPoses(
        *base, output_save_dir_, pending, *artifacts);
    if (!prepared) return prepared;
    transaction.Working().artifacts = artifacts->Snapshot();
    return commitTransaction(std::move(transaction), &pending);
  }
  return Result<void>::Failure(Error::InvalidArgument("unknown node"));
}

Result<void> StageExecutor::runLoopDetectThrough(char target_agent) {
  OPEN_LMM_ZONE_N("MapServer.LoopDetectReplay");
  const auto base = committedState();
  auto prefix = OrderedAgentPrefix(base->ordered_agents, target_agent);
  if (!prefix) return Result<void>::Failure(prefix.GetError());
  if (base->payload->database->raw_data.size() !=
      base->payload->contexts.size()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "DataLoad must complete before loop-detection replay"));
  }

  SessionTransaction transaction(base);
  auto optimizer = createOptimizer();
  if (!optimizer) return Result<void>::Failure(optimizer.GetError());
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = base->payload->database->raw_data;
  database->stored_alignments = base->payload->database->stored_alignments;
  auto contexts = base->payload->contexts;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(state_mutex_);
    cancellation = cancellation_;
    database->alignment_feedback = alignment_feedback_;
  }
  for (auto& context : contexts) {
    context.cancellation = cancellation;
    context.flow = ControlFlow::kContinue;
    context.loop_output.reset();
  }

  LoopDetectNode loop_node([cfg = base->config->loop_detector]() {
    return LoopDetectorBase::createInstance(*cfg);
  });
  OptimizeNode optimize_node(optimizer.Value());
  const auto& replay_agents = prefix.Value();
  for (std::size_t index = 0; index < replay_agents.size(); ++index) {
    const char agent_id = replay_agents[index];
    auto context = std::find_if(
        contexts.begin(), contexts.end(), [agent_id](const auto& item) {
          return item.agent.id == agent_id;
        });
    if (context == contexts.end() || !context->raw_data) {
      return Result<void>::Failure(Error::InvalidArgument(
          "ordered replay raw payload is unavailable"));
    }
    auto loop = loop_node.Process(*context, *database);
    if (!loop) return Result<void>::Failure(loop.GetError());
    if (index + 1 == replay_agents.size()) break;
    auto optimized = optimize_node.Process(*context, *database);
    if (!optimized) return Result<void>::Failure(optimized.GetError());
  }

  auto payload = std::make_shared<SessionPayload>();
  payload->contexts = std::move(contexts);
  payload->database = std::move(database);
  payload->optimizer = std::move(optimizer).Value();
  transaction.SetPayload(std::move(payload));
  auto artifacts = artifactEditor(*base);
  artifacts->CompleteLoopDetectThrough(target_agent, base->ordered_agents);
  transaction.Working().artifacts = artifacts->Snapshot();
  return commitTransaction(std::move(transaction));
}

Result<void> StageExecutor::runDataLoadStage() {
  OPEN_LMM_ZONE_N("MapServer.DataLoadStage");
  const auto base = committedState();
  SessionTransaction transaction(base);
  auto optimizer = createOptimizer();
  if (!optimizer) return Result<void>::Failure(optimizer.GetError());
  auto database = std::make_shared<SharedDatabase>();
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<AlignmentFeedbackBroker> feedback;
  {
    std::lock_guard lock(state_mutex_);
    cancellation = cancellation_;
    feedback = alignment_feedback_;
  }
  database->alignment_feedback = feedback;
  installStoredAlignments(*database);
  auto contexts = buildContexts();
  for (auto& context : contexts) context.cancellation = cancellation;
  auto loader = DataLoaderBase::createInstance(*base->config->data_loader);
  if (!loader) return Result<void>::Failure(loader.GetError());
  Pipeline pipeline;
  for (NodeId node : StageNodes(StageId::kDataLoad)) {
    if (node != NodeId::kDataLoad) {
      return Result<void>::Failure(
          Error::InvalidArgument("unsupported DataLoad execution spec"));
    }
    pipeline.AddNode(std::make_unique<DataLoadNode>(std::move(loader).Value()));
  }
  auto result = pipeline.Run(contexts, *database);
  if (!result) return result;

  auto payload = std::make_shared<SessionPayload>();
  payload->contexts = std::move(contexts);
  payload->database = std::move(database);
  payload->optimizer = std::move(optimizer).Value();
  transaction.SetPayload(std::move(payload));
  auto artifacts = artifactEditor(*base);
  artifacts->BeginStage(StageId::kDataLoad);
  artifacts->CompleteStage(StageId::kDataLoad);
  transaction.Working().artifacts = artifacts->Snapshot();
  return commitTransaction(std::move(transaction));
}

Result<void> StageExecutor::runAlignmentStage() {
  OPEN_LMM_ZONE_N("MapServer.AlignmentStage");
  const auto base = committedState();
  if (base->payload->database->raw_data.size() !=
      base->payload->contexts.size()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "DataLoad stage must complete before Alignment"));
  }
  SessionTransaction transaction(base);
  auto optimizer = createOptimizer();
  if (!optimizer) return Result<void>::Failure(optimizer.GetError());
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = base->payload->database->raw_data;
  database->stored_alignments = base->payload->database->stored_alignments;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(state_mutex_);
    cancellation = cancellation_;
    database->alignment_feedback = alignment_feedback_;
  }
  auto contexts = base->payload->contexts;
  for (auto& ctx : contexts) {
    ctx.cancellation = cancellation;
    ctx.flow = ControlFlow::kContinue;
    ctx.loop_output.reset();
  }
  Pipeline pipeline;
  for (NodeId node : StageNodes(StageId::kAlignment)) {
    if (node == NodeId::kLoopDetect) {
      pipeline.AddNode(std::make_unique<LoopDetectNode>(
          [cfg = base->config->loop_detector]() {
            return LoopDetectorBase::createInstance(*cfg);
          }));
    } else if (node == NodeId::kOptimize) {
      pipeline.AddNode(std::make_unique<OptimizeNode>(optimizer.Value()));
    } else {
      return Result<void>::Failure(
          Error::InvalidArgument("unsupported Alignment execution spec"));
    }
  }
  auto result = pipeline.Run(contexts, *database);
  if (!result) return result;

  auto payload = std::make_shared<SessionPayload>();
  payload->contexts = std::move(contexts);
  payload->database = std::move(database);
  payload->optimizer = std::move(optimizer).Value();
  transaction.SetPayload(std::move(payload));
  auto artifacts = artifactEditor(*base);
  artifacts->BeginStage(StageId::kAlignment);
  artifacts->CompleteStage(StageId::kAlignment);
  auto pending = output_repository_.Begin();
  auto prepared = prepareAlignmentArtifacts(
      transaction.Working(), pending, *artifacts);
  if (!prepared) return prepared;
  transaction.Working().artifacts = artifacts->Snapshot();
  return commitTransaction(std::move(transaction), &pending);
}

Result<void> StageExecutor::prepareAlignmentArtifacts(
    const SessionState& state, PendingOutputSet& pending,
    ArtifactRepository& artifacts) const {
  const auto& contexts = state.payload->contexts;
  const auto& shared_data = *state.payload->database;
  nlohmann::json root;
  root["version"] = 2;
  root["transform_convention"] = "global_T_agent";
  root["generated_at_unix_ms"] = static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
          std::chrono::system_clock::now().time_since_epoch()).count());
  root["config_fingerprint"] = alignment_config_fingerprint_;
  root["session_fingerprint"] = alignment_session_fingerprint_;
  root["input_fingerprints"] = nlohmann::json::object();
  for (const auto& [agent, fingerprint] : alignment_input_fingerprints_) {
    root["input_fingerprints"][std::string(1, agent)] = fingerprint;
  }
  root["alignments"] = nlohmann::json::array();
  const auto method_name = [](AlignmentMethod method) {
    switch (method) {
      case AlignmentMethod::kPending: return "pending";
      case AlignmentMethod::kKissMatcher: return "kiss_matcher";
      case AlignmentMethod::kDescriptor: return "descriptor";
      case AlignmentMethod::kManual: return "manual";
    }
    return "unknown";
  };
  const auto matrix_json = [](const Eigen::Isometry3d& transform) {
    nlohmann::json values = nlohmann::json::array();
    for (int row = 0; row < 4; ++row) {
      for (int col = 0; col < 4; ++col) {
        values.push_back(transform.matrix()(row, col));
      }
    }
    return values;
  };
  const auto approval_name = [](AlignmentApproval approval) {
    return approval == AlignmentApproval::kUser ? "user" : "automatic";
  };

  for (const auto& ctx : contexts) {
    if (!ctx.loop_output || !ctx.loop_output->accepted_global_T_agent ||
        !ctx.loop_output->accepted_alignment_method ||
        !ctx.loop_output->accepted_alignment_approval) {
      return Result<void>::Failure(Error::IoError(
          "cannot save incomplete alignment artifact for agent " +
          std::string(1, ctx.agent.id)));
    }
    nlohmann::json item;
    item["agent"] = std::string(1, ctx.agent.id);
    item["source_agent"] = std::string(1, ctx.agent.id);
    item["target_agent"] =
        std::string(1, ctx.loop_output->accepted_target_agent);
    item["method"] = method_name(*ctx.loop_output->accepted_alignment_method);
    item["approval"] = approval_name(
        *ctx.loop_output->accepted_alignment_approval);
    item["accepted_at_unix_ms"] = ctx.loop_output->accepted_at_unix_ms;
    item["accepted_global_T_agent"] =
        matrix_json(*ctx.loop_output->accepted_global_T_agent);
    const auto optimized = shared_data.descriptor_store.aligned_maps.find(
        ctx.agent.id);
    if (optimized != shared_data.descriptor_store.aligned_maps.end()) {
      item["optimized_global_T_agent"] =
          matrix_json(optimized->second.global_T_agent);
      item["map_revision"] = optimized->second.revision;
    }
    const auto& metrics = ctx.loop_output->accepted_alignment_metrics;
    item["metrics"] = {
        {"correspondence_count", metrics.correspondence_count},
        {"rotation_inliers", metrics.rotation_inliers},
        {"final_inliers", metrics.final_inliers},
        {"consensus_size", metrics.consensus_size},
    };
    if (metrics.fitness) item["metrics"]["fitness"] = *metrics.fitness;
    if (metrics.overlap_ratio) {
      item["metrics"]["overlap_ratio"] = *metrics.overlap_ratio;
    }
    root["alignments"].push_back(std::move(item));
  }

  const fs::path destination =
      fs::path(output_save_dir_) / "map_alignments.json";
  const fs::path temporary = destination.string() + ".tmp";
  pending.Add(temporary, destination);
  {
    std::ofstream output(temporary);
    if (!output) {
      return Result<void>::Failure(Error::IoError(
          "failed to open alignment artifact " + temporary.string()));
    }
    output << root.dump(2) << '\n';
    if (!output) {
      return Result<void>::Failure(Error::IoError(
          "failed to write alignment artifact " + temporary.string()));
    }
  }
  for (char agent : state.ordered_agents) {
    artifacts.RecordExternalFile(ArtifactType::kMapAlignment, agent,
                                 destination.string(), HexFingerprint([&] {
                                   uint64_t hash = 14695981039346656037ULL;
                                   HashText(hash, root.dump());
                                   return hash;
                                 }()));
  }
  const bool has_user_approval = std::any_of(
      contexts.begin(), contexts.end(), [](const auto& ctx) {
        return ctx.loop_output &&
               ctx.loop_output->accepted_alignment_approval ==
                   AlignmentApproval::kUser;
      });
  if (has_user_approval) {
    const fs::path cache_temporary = alignment_cache_path_.string() + ".tmp";
    pending.Add(cache_temporary, alignment_cache_path_);
    {
      std::ofstream cache(cache_temporary);
      if (!cache) {
        return Result<void>::Failure(Error::IoError(
            "failed to open alignment cache " + cache_temporary.string()));
      }
      cache << root.dump(2) << '\n';
      if (!cache) {
        return Result<void>::Failure(Error::IoError(
            "failed to write alignment cache " + cache_temporary.string()));
      }
    }
  }
  return Result<void>::Ok();
}

Result<void> StageExecutor::RunOptimizeThrough(char target_agent) {
  ExecutionLease execution(execution_active_);
  if (!execution) {
    return Result<void>::Failure(
        Error::InvalidArgument("another MapServer operation is active"));
  }
  auto result = runOptimizeThrough(target_agent);
  if (result) publishVisualizationState(committedState(), false);
  return result;
}

Result<void> StageExecutor::runOptimizeThrough(char target_agent) {
  OPEN_LMM_ZONE_N("MapServer.OptimizeReplay");
  OPEN_LMM_PLOT("optimizer.target_agent", static_cast<int>(target_agent));
  auto ready = ensureReady();
  if (!ready) return ready;
  const auto base = committedState();
  if (base->payload->database->raw_data.size() !=
      base->payload->contexts.size()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "DataLoad and loop detection must complete before optimizer replay"));
  }
  auto prefix = OrderedAgentPrefix(base->ordered_agents, target_agent);
  if (!prefix) return Result<void>::Failure(prefix.GetError());
  SessionTransaction transaction(base);
  auto optimizer = createOptimizer();
  if (!optimizer) return Result<void>::Failure(optimizer.GetError());
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = base->payload->database->raw_data;
  database->stored_alignments = base->payload->database->stored_alignments;
  auto contexts = base->payload->contexts;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(state_mutex_);
    cancellation = cancellation_;
    database->alignment_feedback = alignment_feedback_;
  }
  bool anchor_descriptor = true;
  for (char agent_id : prefix.Value()) {
    const auto context = std::find_if(
        contexts.begin(), contexts.end(), [agent_id](const auto& item) {
          return item.agent.id == agent_id;
        });
    if (context == contexts.end() || !context->loop_output) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Alignment loop artifacts are missing for agent " +
          std::string{agent_id}));
    }
    if (anchor_descriptor) {
      database->descriptor_store.set_anchor_descriptor(
          agent_id, context->loop_output->agent_descriptors);
      anchor_descriptor = false;
    } else {
      database->descriptor_store.merge_descriptor_db(
          agent_id, context->loop_output->agent_descriptors);
    }
  }
  OptimizeNode optimize_node(optimizer.Value());
  for (char agent_id : prefix.Value()) {
    auto context = std::find_if(
        contexts.begin(), contexts.end(), [agent_id](const auto& item) {
          return item.agent.id == agent_id;
        });
    context->cancellation = cancellation;
    context->flow = ControlFlow::kContinue;
    auto result = optimize_node.Process(*context, *database);
    if (!result) return Result<void>::Failure(result.GetError());
  }
  const auto target = std::find(
      base->ordered_agents.begin(), base->ordered_agents.end(), target_agent);
  for (auto suffix = std::next(target); suffix != base->ordered_agents.end();
       ++suffix) {
    const auto context = std::find_if(
        contexts.begin(), contexts.end(), [agent_id = *suffix](const auto& item) {
          return item.agent.id == agent_id;
        });
    if (context != contexts.end()) context->loop_output.reset();
  }
  auto payload = std::make_shared<SessionPayload>();
  payload->contexts = std::move(contexts);
  payload->database = std::move(database);
  payload->optimizer = std::move(optimizer).Value();
  transaction.SetPayload(std::move(payload));
  auto artifacts = artifactEditor(*base);
  artifacts->CompleteOptimizeThrough(target_agent, base->ordered_agents);
  transaction.Working().artifacts = artifacts->Snapshot();
  return commitTransaction(std::move(transaction));
}

Result<VisualizationSnapshot> StageExecutor::CreateVisualizationSnapshot(
    char agent) const {
  std::shared_ptr<const VisualizationState> state;
  {
    std::lock_guard lock(state_mutex_);
    state = visualization_state_;
  }
  const auto found = state->agents.find(agent);
  if (found == state->agents.end()) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("optimized poses are not available for agent"));
  }
  VisualizationSnapshot snapshot = found->second;
  const auto map = state->map_paths.find(agent);
  if (map == state->map_paths.end()) {
    return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
  }
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPCDFile(map->second.string(), cloud) < 0) {
    return Result<VisualizationSnapshot>::Failure(
        Error::IoError("failed to read visualization map " +
                       map->second.string()));
  }
  snapshot.points.reserve(cloud.size());
  for (const auto& point : cloud) {
    if (!pcl::isFinite(point)) continue;
    snapshot.points.push_back({point.x, point.y, point.z, point.intensity});
    const Eigen::Vector3f position(point.x, point.y, point.z);
    if (!snapshot.has_bounds) {
      snapshot.min_bound = snapshot.max_bound = position;
      snapshot.has_bounds = true;
    } else {
      snapshot.min_bound = snapshot.min_bound.cwiseMin(position);
      snapshot.max_bound = snapshot.max_bound.cwiseMax(position);
    }
  }
  snapshot.map_available = true;
  return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
}

void StageExecutor::publishEmptyVisualizationState(uint64_t session_revision) {
  auto state = std::make_shared<VisualizationState>();
  state->revision = session_revision;
  std::lock_guard lock(state_mutex_);
  visualization_state_ = std::move(state);
}

void StageExecutor::publishVisualizationState(
    const std::shared_ptr<const SessionState>& session, bool include_maps) {
  auto state = std::make_shared<VisualizationState>();
  if (!session || !session->payload || !session->payload->database) {
    publishEmptyVisualizationState(session ? session->revision : 0);
    return;
  }
  state->revision = session->revision;
  const auto& database = *session->payload->database;
  for (const auto& [agent, optimized] : database.optimized_data) {
    auto& snapshot = state->agents[agent];
    snapshot.agent = agent;
    snapshot.poses.reserve(optimized->optimized_poses.size());
    for (const auto& [index, pose] : optimized->optimized_poses) {
      snapshot.poses.push_back({index, pose.cast<float>()});
    }
    for (std::size_t i = 1; i < snapshot.poses.size(); ++i) {
      snapshot.edges.push_back(
          {agent, static_cast<std::size_t>(snapshot.poses[i - 1].index),
           agent, static_cast<std::size_t>(snapshot.poses[i].index),
           VisualizationEdgeType::kTrajectory});
    }
    const auto context = std::find_if(
        session->payload->contexts.begin(), session->payload->contexts.end(),
        [agent_id = agent](const AgentPipelineCtx& ctx) {
          return ctx.agent.id == agent_id;
        });
    if (context != session->payload->contexts.end() && context->loop_output) {
      const auto append_loops = [&snapshot](const LoopPairVec& loops,
                                            VisualizationEdgeType type) {
        for (const auto& loop : loops) {
          snapshot.edges.push_back({loop.from.first, loop.from.second,
                                    loop.to.first, loop.to.second, type});
        }
      };
      append_loops(context->loop_output->intra_loops,
                   VisualizationEdgeType::kIntraLoop);
      append_loops(context->loop_output->inter_loops,
                   VisualizationEdgeType::kInterLoop);
    }
    if (include_maps) {
      const fs::path map_path = fs::path(output_save_dir_) /
          ("global_map_" + std::string{agent} + ".pcd");
      if (fs::is_regular_file(map_path)) state->map_paths[agent] = map_path;
    }
  }
  std::lock_guard lock(state_mutex_);
  for (auto& [agent, snapshot] : state->agents) {
    (void)agent;
    snapshot.revision = state->revision;
  }
  visualization_state_ = std::move(state);
}

Result<void> StageExecutor::runMapUpdateStage() {
  OPEN_LMM_ZONE_N("MapServer.MapUpdateStage");
  if (!enable_map_updater_) return Result<void>::Ok();
  const auto base = committedState();
  if (base->payload->database->optimized_data.empty()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "Alignment stage must complete before MapUpdate"));
  }
  SessionTransaction transaction(base);
  auto loader = DataLoaderBase::createInstance(*base->config->data_loader);
  if (!loader) return Result<void>::Failure(loader.GetError());
  auto contexts = base->payload->contexts;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(state_mutex_);
    cancellation = cancellation_;
  }
  for (auto& ctx : contexts) {
    ctx.flow = ControlFlow::kContinue;
    ctx.cancellation = cancellation;
  }
  auto database = std::make_shared<SharedDatabase>();
  database->raw_data = base->payload->database->raw_data;
  database->optimized_data = base->payload->database->optimized_data;
  Pipeline pipeline;
  for (NodeId node : StageNodes(StageId::kMapUpdate)) {
    if (node != NodeId::kMapUpdate) {
      return Result<void>::Failure(
          Error::InvalidArgument("unsupported MapUpdate execution spec"));
    }
    pipeline.AddNode(std::make_unique<MapUpdateNode>(
        std::move(loader).Value(),
        [cfg = base->config->dynamic_remover]() {
          return DynamicRemoverBase::createInstance(*cfg);
        },
        output_save_dir_, save_voxel_size_, true));
  }

  auto pending = output_repository_.Begin();
  for (const auto& ctx : contexts) {
    fs::path final_path = fs::path(output_save_dir_) /
        ("global_map_" + std::string{ctx.agent.id} + ".pcd");
    fs::path temporary = final_path;
    temporary += ".tmp";
    pending.Add(std::move(temporary), std::move(final_path));
  }
  auto result = pipeline.Run(contexts, *database);
  if (!result) return result;
  auto artifacts = artifactEditor(*base);
  artifacts->BeginStage(StageId::kMapUpdate);
  artifacts->CompleteStage(StageId::kMapUpdate);
  for (const auto& ctx : contexts) {
    fs::path final_path = fs::path(output_save_dir_) /
        ("global_map_" + std::string{ctx.agent.id} + ".pcd");
    fs::path temporary = final_path;
    temporary += ".tmp";
    uint64_t hash = 14695981039346656037ULL;
    HashFile(hash, temporary);
    for (ArtifactType type : {ArtifactType::kGlobalMap,
                              ArtifactType::kPcdFile}) {
      artifacts->RecordExternalFile(type, ctx.agent.id, final_path.string(),
                                    HexFingerprint(hash));
    }
  }
  transaction.Working().artifacts = artifacts->Snapshot();
  return commitTransaction(std::move(transaction), &pending);
}

Result<void> StageExecutor::runSaveStage() {
  OPEN_LMM_ZONE_N("MapServer.SaveStage");
  const auto save_nodes = StageNodes(StageId::kSave);
  if (save_nodes != std::vector<NodeId>{NodeId::kPoseSave}) {
    return Result<void>::Failure(
        Error::InvalidArgument("unsupported Save execution spec"));
  }
  const auto base = committedState();
  if (base->payload->database->optimized_data.empty()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "Alignment stage must complete before Save"));
  }
  LogInfo("[save] saving optimized poses and maps to " + output_save_dir_);
  SessionTransaction transaction(base);
  auto pending = output_repository_.Begin();
  auto artifacts = artifactEditor(*base);
  artifacts->BeginStage(StageId::kSave);
  artifacts->CompleteStage(StageId::kSave);
  auto pose_result = prepareOptimizedPoses(
      *base, output_save_dir_, pending, *artifacts);
  if (!pose_result) return pose_result;
  if (!enable_map_updater_) {
    auto map_result = prepareOptimizedMap(
        *base, output_save_dir_, pending, *artifacts);
    if (!map_result) return map_result;
  }
  transaction.Working().artifacts = artifacts->Snapshot();
  return commitTransaction(std::move(transaction), &pending);
}

Result<void> StageExecutor::prepareOptimizedPoses(
    const SessionState& state, const std::string& output_save_dir,
    PendingOutputSet& pending, ArtifactRepository& artifacts) {
  OPEN_LMM_ZONE_N("Save.Poses");
  const auto& database = *state.payload->database;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(state_mutex_);
    cancellation = cancellation_;
  }
  for (const auto& [agent_id, opt_data] : database.optimized_data) {
    if (opt_data->optimized_poses.empty()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "No optimized poses to save for agent " + std::string{agent_id}));
    }
    fs::path final_path = fs::path(output_save_dir) /
        ("optimized_poses_" + std::string{agent_id} + ".txt");
    fs::path temporary = final_path;
    temporary += ".tmp";
    std::error_code ignored;
    fs::remove(temporary, ignored);
    pending.Add(temporary, final_path);

    std::ofstream file(temporary);
    if (!file) {
      return Result<void>::Failure(Error::IoError(
          "failed to open temporary pose output: " + temporary.string()));
    }
    for (const auto& pose : opt_data->optimized_poses) {
      if (cancellation && cancellation->IsCancellationRequested()) {
        file.close();
        return Result<void>::Failure(Error::Cancelled("during pose write"));
      }
      int scan_idx = pose.first;
      Eigen::Matrix4d pose_matrix = pose.second.matrix();
      Eigen::Vector3d translation = pose_matrix.block<3, 1>(0, 3);
      Eigen::Quaterniond quaternion(pose_matrix.block<3, 3>(0, 0));
      file << scan_idx << "," << translation.x() << "," << translation.y()
           << "," << translation.z() << "," << quaternion.x() << ","
           << quaternion.y() << "," << quaternion.z() << "," << quaternion.w()
           << "\n";
    }
    file.flush();
    if (!file) {
      file.close();
      return Result<void>::Failure(Error::IoError(
          "failed to write pose output: " + temporary.string()));
    }
    uint64_t hash = 14695981039346656037ULL;
    HashFile(hash, temporary);
    artifacts.RecordExternalFile(ArtifactType::kPoseFile, agent_id,
                                 final_path.string(), HexFingerprint(hash));
  }
  if (cancellation && cancellation->IsCancellationRequested()) {
    return Result<void>::Failure(Error::Cancelled("before pose file commit"));
  }
  return Result<void>::Ok();
}

Result<void> StageExecutor::prepareOptimizedMap(
    const SessionState& state, const std::string& output_save_dir,
    PendingOutputSet& pending, ArtifactRepository& artifacts) {
  OPEN_LMM_ZONE_N("Save.FallbackMap");
  const auto& database = *state.payload->database;
  std::shared_ptr<CancellationToken> cancellation;
  {
    std::lock_guard lock(state_mutex_);
    cancellation = cancellation_;
  }
  for (const auto& [agent_id, opt_data] : database.optimized_data) {
    const auto raw_it = database.raw_data.find(agent_id);
    if (raw_it == database.raw_data.end()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Optimized agent has no corresponding raw data"));
    }
    const auto& filtered_scans = raw_it->second->filtered_scans;

    pcl::PointCloud<pcl::PointXYZI>::Ptr optimized_map(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& pose : opt_data->optimized_poses) {
      if (cancellation && cancellation->IsCancellationRequested()) {
        return Result<void>::Failure(Error::Cancelled("during map assembly"));
      }
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan(
          new pcl::PointCloud<pcl::PointXYZI>);
      int scan_idx = pose.first;
      if (scan_idx < 0 || static_cast<std::size_t>(scan_idx) >=
                              filtered_scans.size()) {
        return Result<void>::Failure(Error::InvalidArgument(
            "Optimized pose scan index is out of range"));
      }
      Eigen::Matrix4d pose_matrix = pose.second.matrix();
      pcl::transformPointCloud(*filtered_scans.at(scan_idx),
                               *transformed_scan, pose_matrix);
      *optimized_map += *transformed_scan;
    }

    if (optimized_map->empty()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Optimized map is empty for agent " + std::string{agent_id}));
    }

    fs::path output_map_file =
        fs::path(output_save_dir) /
        ("global_map_" + std::string{agent_id} + ".pcd");
    fs::path temporary = output_map_file;
    temporary += ".tmp";
    pending.Add(temporary, output_map_file);
    std::error_code ignored;
    fs::remove(temporary, ignored);
    if (pcl::io::savePCDFileBinaryCompressed(temporary, *optimized_map) != 0) {
      fs::remove(temporary, ignored);
      return Result<void>::Failure(Error::IoError(
          "failed to save temporary map output: " + temporary.string()));
    }
    if (cancellation && cancellation->IsCancellationRequested()) {
      return Result<void>::Failure(Error::Cancelled("before map file commit"));
    }
    uint64_t hash = 14695981039346656037ULL;
    HashFile(hash, temporary);
    for (ArtifactType type : {ArtifactType::kGlobalMap,
                              ArtifactType::kPcdFile}) {
      artifacts.RecordExternalFile(type, agent_id, output_map_file.string(),
                                   HexFingerprint(hash));
    }
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
