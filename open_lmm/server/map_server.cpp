
#include "map_server.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/profiling.hpp>
#include <open_lmm/utils/config.hpp>
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

namespace open_lmm {
namespace {
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

MapServer::MapServer() {
  InitializeLogging();
  try {
    parseConfig();
  } catch (const std::exception& e) {
    initialization_error_ = Error::ParseError(e.what());
  }
}
MapServer::~MapServer() {}

void MapServer::parseConfig() {
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

  Config config_backend_optimizer =
      Config(GlobalConfig::get_global_config_path("config_backend_optimizer"));
  if (!config_backend_optimizer.is_valid()) {
    initialization_error_ = Error::ParseError(
        config_backend_optimizer.error_message());
    return;
  }
  auto optimizer_result =
      BackendOptimizerBase::createInstance(config_backend_optimizer);
  if (!optimizer_result) {
    initialization_error_ = optimizer_result.GetError();
    return;
  }
  backend_optimizer_ = std::shared_ptr<BackendOptimizerBase>(
      std::move(optimizer_result).Value());
  computeAlignmentFingerprints();
  loadAlignmentCache();
}

void MapServer::computeAlignmentFingerprints() {
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

void MapServer::loadAlignmentCache() {
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

void MapServer::installStoredAlignments() {
  shared_data_->stored_alignments = cached_alignments_;
}

std::vector<AgentPipelineCtx> MapServer::buildContexts() const {
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

Result<void> MapServer::process() {
  OPEN_LMM_ZONE_N("MapServer.Process");
  OPEN_LMM_PLOT("agent.count", agent_num_);
  for (StageId stage : {StageId::kDataLoad, StageId::kAlignment,
                        StageId::kMapUpdate, StageId::kSave}) {
    auto result = RunStage(stage);
    if (!result) return result;
  }
  LogInfo("[pipeline] all stages completed");
  return Result<void>::Ok();
}

Result<void> MapServer::ensureReady() {
  if (initialization_error_) return Result<void>::Failure(*initialization_error_);
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
  if (contexts_.empty()) contexts_ = buildContexts();
  return Result<void>::Ok();
}

Result<void> MapServer::validateInputFiles() const {
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

Result<void> MapServer::ValidateReady() {
  std::lock_guard lock(state_mutex_);
  return ensureReady();
}

std::vector<char> MapServer::AgentIds() const {
  std::lock_guard lock(state_mutex_);
  std::vector<char> ids;
  for (const auto& ctx : contexts_.empty() ? buildContexts() : contexts_) {
    ids.push_back(ctx.agent.id);
  }
  return ids;
}

void MapServer::SetCancellationToken(std::shared_ptr<CancellationToken> token) {
  std::lock_guard lock(state_mutex_);
  cancellation_ = std::move(token);
  for (auto& ctx : contexts_) ctx.cancellation = cancellation_;
}

void MapServer::SetAlignmentFeedbackBroker(
    std::shared_ptr<AlignmentFeedbackBroker> broker) {
  std::lock_guard lock(state_mutex_);
  alignment_feedback_ = std::move(broker);
  shared_data_->alignment_feedback = alignment_feedback_;
}

Result<void> MapServer::RunStage(StageId stage) {
  std::lock_guard lock(state_mutex_);
  auto ready = ensureReady();
  if (!ready) return ready;
  switch (stage) {
    case StageId::kDataLoad: return runDataLoadStage();
    case StageId::kAlignment: return runAlignmentStage();
    case StageId::kMapUpdate: return runMapUpdateStage();
    case StageId::kSave: return runSaveStage();
  }
  return Result<void>::Failure(Error::InvalidArgument("unknown stage"));
}

Result<void> MapServer::RunNode(NodeId node, std::optional<char> agent) {
  std::lock_guard lock(state_mutex_);
  auto ready = ensureReady();
  if (!ready) return ready;
  if (!agent) {
    return Result<void>::Failure(
        Error::InvalidArgument("node execution requires an agent"));
  }
  auto it = std::find_if(contexts_.begin(), contexts_.end(),
                         [agent](const AgentPipelineCtx& ctx) {
                           return ctx.agent.id == *agent;
                         });
  if (it == contexts_.end()) {
    return Result<void>::Failure(Error::InvalidArgument("unknown agent"));
  }
  it->flow = ControlFlow::kContinue;

  if (node == NodeId::kDataLoad) {
    auto loader = DataLoaderBase::createInstance(config_data_loader_.value());
    if (!loader) return Result<void>::Failure(loader.GetError());
    DataLoadNode load_node(std::move(loader).Value());
    auto result = load_node.Process(*it, *shared_data_);
    if (!result) return Result<void>::Failure(result.GetError());
    const auto changed = std::distance(contexts_.begin(), it);
    shared_data_->descriptor_store.clear();
    shared_data_->optimized_data.clear();
    backend_optimizer_->Reset();
    for (std::size_t i = static_cast<std::size_t>(changed);
         i < contexts_.size(); ++i) {
      contexts_[i].loop_output.reset();
    }
    return Result<void>::Ok();
  }
  if (node == NodeId::kLoopDetect) {
    if (!it->raw_data) {
      return Result<void>::Failure(
          Error::InvalidArgument("RawData is required for LoopDetect"));
    }
    LoopDetectNode loop_node([cfg = config_loop_detector_.value()]() {
      return LoopDetectorBase::createInstance(cfg);
    });
    auto result = loop_node.Process(*it, *shared_data_);
    if (!result) return Result<void>::Failure(result.GetError());
    return Result<void>::Ok();
  }
  if (node == NodeId::kOptimize) return RunOptimizeThrough(*agent);
  if (node == NodeId::kMapUpdate) {
    if (shared_data_->optimized_data.find(*agent) ==
        shared_data_->optimized_data.end()) {
      return Result<void>::Failure(
          Error::InvalidArgument("OptimizedPoses is required for MapUpdate"));
    }
    auto loader = DataLoaderBase::createInstance(config_data_loader_.value());
    if (!loader) return Result<void>::Failure(loader.GetError());
    MapUpdateNode update_node(
        std::move(loader).Value(),
        [cfg = config_dynamic_remover_.value()]() {
          return DynamicRemoverBase::createInstance(cfg);
        },
        output_save_dir_, save_voxel_size_);
    auto result = update_node.Process(*it, *shared_data_);
    if (!result) return Result<void>::Failure(result.GetError());
    return Result<void>::Ok();
  }
  if (node == NodeId::kPoseSave) {
    // Pose files are deterministic and cheap; the current writer commits all
    // ready agents so a partial retry cannot leave a mixed output set.
    return saveOptimizedPoses(output_save_dir_);
  }
  return Result<void>::Failure(Error::InvalidArgument("unknown node"));
}

Result<void> MapServer::runDataLoadStage() {
  OPEN_LMM_ZONE_N("MapServer.DataLoadStage");
  shared_data_ = std::make_shared<SharedDatabase>();
  shared_data_->alignment_feedback = alignment_feedback_;
  installStoredAlignments();
  backend_optimizer_->Reset();
  contexts_ = buildContexts();
  auto loader = DataLoaderBase::createInstance(config_data_loader_.value());
  if (!loader) return Result<void>::Failure(loader.GetError());
  Pipeline pipeline;
  pipeline.AddNode(std::make_unique<DataLoadNode>(std::move(loader).Value()));
  return pipeline.Run(contexts_, *shared_data_);
}

Result<void> MapServer::runAlignmentStage() {
  OPEN_LMM_ZONE_N("MapServer.AlignmentStage");
  if (shared_data_->raw_data.size() != contexts_.size()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "DataLoad stage must complete before Alignment"));
  }
  shared_data_->descriptor_store.clear();
  shared_data_->optimized_data.clear();
  backend_optimizer_->Reset();
  for (auto& ctx : contexts_) {
    ctx.flow = ControlFlow::kContinue;
    ctx.loop_output.reset();
  }
  Pipeline pipeline;
  pipeline
      .AddNode(std::make_unique<LoopDetectNode>(
          [cfg = config_loop_detector_.value()]() {
            return LoopDetectorBase::createInstance(cfg);
          }))
      .AddNode(std::make_unique<OptimizeNode>(backend_optimizer_));
  auto result = pipeline.Run(contexts_, *shared_data_);
  if (!result) return result;
  return saveAlignmentArtifacts();
}

Result<void> MapServer::saveAlignmentArtifacts() const {
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

  for (const auto& ctx : contexts_) {
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
    const auto optimized = shared_data_->descriptor_store.aligned_maps.find(
        ctx.agent.id);
    if (optimized != shared_data_->descriptor_store.aligned_maps.end()) {
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
  std::error_code error;
  fs::rename(temporary, destination, error);
  if (error) {
    fs::remove(temporary);
    return Result<void>::Failure(Error::IoError(
        "failed to commit alignment artifact " + destination.string() +
        ": " + error.message()));
  }
  const bool has_user_approval = std::any_of(
      contexts_.begin(), contexts_.end(), [](const auto& ctx) {
        return ctx.loop_output &&
               ctx.loop_output->accepted_alignment_approval ==
                   AlignmentApproval::kUser;
      });
  if (has_user_approval) {
    const fs::path cache_temporary = alignment_cache_path_.string() + ".tmp";
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
    error.clear();
    fs::rename(cache_temporary, alignment_cache_path_, error);
    if (error) {
      fs::remove(cache_temporary);
      return Result<void>::Failure(Error::IoError(
          "failed to commit alignment cache " +
          alignment_cache_path_.string() + ": " + error.message()));
    }
  }
  return Result<void>::Ok();
}

Result<void> MapServer::RunOptimizeThrough(char target_agent) {
  std::lock_guard lock(state_mutex_);
  OPEN_LMM_ZONE_N("MapServer.OptimizeReplay");
  OPEN_LMM_PLOT("optimizer.target_agent", static_cast<int>(target_agent));
  auto ready = ensureReady();
  if (!ready) return ready;
  if (shared_data_->raw_data.size() != contexts_.size()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "DataLoad and loop detection must complete before optimizer replay"));
  }
  backend_optimizer_->Reset();
  shared_data_->optimized_data.clear();
  OptimizeNode optimize_node(backend_optimizer_);
  bool found = false;
  for (auto& ctx : contexts_) {
    if (!ctx.loop_output) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Alignment loop artifacts are missing for agent " +
          std::string{ctx.agent.id}));
    }
    ctx.flow = ControlFlow::kContinue;
    auto result = optimize_node.Process(ctx, *shared_data_);
    if (!result) return Result<void>::Failure(result.GetError());
    if (ctx.agent.id == target_agent) {
      found = true;
      break;
    }
  }
  if (!found) {
    return Result<void>::Failure(Error::InvalidArgument(
        "unknown optimizer replay target agent"));
  }
  return Result<void>::Ok();
}

Result<VisualizationSnapshot> MapServer::CreateVisualizationSnapshot(
    char agent, std::size_t max_points) const {
  std::lock_guard lock(state_mutex_);
  if (max_points == 0) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("visualization point budget must be non-zero"));
  }

  const auto optimized = shared_data_->optimized_data.find(agent);
  if (optimized == shared_data_->optimized_data.end()) {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("optimized poses are not available for agent"));
  }

  VisualizationSnapshot snapshot;
  snapshot.agent = agent;
  snapshot.poses.reserve(optimized->second.optimized_poses.size());
  for (const auto& [index, pose] : optimized->second.optimized_poses) {
    snapshot.poses.push_back({index, pose.cast<float>()});
  }
  for (std::size_t i = 1; i < snapshot.poses.size(); ++i) {
    snapshot.edges.push_back({agent,
                              static_cast<std::size_t>(snapshot.poses[i - 1].index),
                              agent,
                              static_cast<std::size_t>(snapshot.poses[i].index),
                              VisualizationEdgeType::kTrajectory});
  }

  const auto context = std::find_if(
      contexts_.begin(), contexts_.end(),
      [agent](const AgentPipelineCtx& ctx) { return ctx.agent.id == agent; });
  if (context != contexts_.end() && context->loop_output) {
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

  const fs::path map_path = fs::path(output_save_dir_) /
      ("global_map_" + std::string{agent} + ".pcd");
  if (!fs::is_regular_file(map_path)) {
    return Result<VisualizationSnapshot>::Ok(std::move(snapshot));
  }
  pcl::PointCloud<pcl::PointXYZI> cloud;
  if (pcl::io::loadPCDFile(map_path.string(), cloud) < 0) {
    return Result<VisualizationSnapshot>::Failure(
        Error::IoError("failed to read visualization map " +
                       map_path.string()));
  }
  const std::size_t stride = std::max<std::size_t>(
      1, (cloud.size() + max_points - 1) / max_points);
  snapshot.points.reserve(std::min(max_points, cloud.size()));
  for (std::size_t i = 0; i < cloud.size() &&
                          snapshot.points.size() < max_points; i += stride) {
    const auto& point = cloud[i];
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

Result<void> MapServer::runMapUpdateStage() {
  OPEN_LMM_ZONE_N("MapServer.MapUpdateStage");
  if (!enable_map_updater_) return Result<void>::Ok();
  if (shared_data_->optimized_data.empty()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "Alignment stage must complete before MapUpdate"));
  }
  auto loader = DataLoaderBase::createInstance(config_data_loader_.value());
  if (!loader) return Result<void>::Failure(loader.GetError());
  for (auto& ctx : contexts_) ctx.flow = ControlFlow::kContinue;
  Pipeline pipeline;
  pipeline.AddNode(std::make_unique<MapUpdateNode>(
      std::move(loader).Value(),
      [cfg = config_dynamic_remover_.value()]() {
        return DynamicRemoverBase::createInstance(cfg);
      },
      output_save_dir_, save_voxel_size_, true));

  const auto cleanup = [this]() {
    for (const auto& ctx : contexts_) {
      fs::path temporary = fs::path(output_save_dir_) /
          ("global_map_" + std::string{ctx.agent.id} + ".pcd.tmp");
      std::error_code ignored;
      fs::remove(temporary, ignored);
    }
  };
  auto result = pipeline.Run(contexts_, *shared_data_);
  if (!result) {
    cleanup();
    return result;
  }
  if (cancellation_ && cancellation_->IsCancellationRequested()) {
    cleanup();
    return Result<void>::Failure(Error::Cancelled("before MapUpdate commit"));
  }
  // Commit begins only after every agent succeeded. Cancellation is no longer
  // observed during this short deterministic rename barrier.
  for (const auto& ctx : contexts_) {
    fs::path final_path = fs::path(output_save_dir_) /
        ("global_map_" + std::string{ctx.agent.id} + ".pcd");
    fs::path temporary = final_path;
    temporary += ".tmp";
    std::error_code rename_error;
    fs::rename(temporary, final_path, rename_error);
    if (rename_error) {
      cleanup();
      return Result<void>::Failure(Error::IoError(
          "failed to commit MapUpdate stage: " + rename_error.message()));
    }
  }
  return Result<void>::Ok();
}

Result<void> MapServer::runSaveStage() {
  OPEN_LMM_ZONE_N("MapServer.SaveStage");
  if (shared_data_->optimized_data.empty()) {
    return Result<void>::Failure(Error::InvalidArgument(
        "Alignment stage must complete before Save"));
  }
  LogInfo("[save] saving optimized poses and maps to " + output_save_dir_);
  auto pose_result = saveOptimizedPoses(output_save_dir_);
  if (!pose_result) return pose_result;
  if (!enable_map_updater_) return saveOptimizedMap(output_save_dir_);
  return Result<void>::Ok();
}

Result<void> MapServer::saveOptimizedPoses(const std::string& output_save_dir) {
  OPEN_LMM_ZONE_N("Save.Poses");
  std::vector<std::pair<fs::path, fs::path>> pending_files;
  const auto cleanup = [&pending_files]() {
    for (const auto& [temporary, final_path] : pending_files) {
      std::error_code ignored;
      fs::remove(temporary, ignored);
    }
  };

  for (const auto& [agent_id, opt_data] : shared_data_->optimized_data) {
    if (opt_data.optimized_poses.empty()) {
      cleanup();
      return Result<void>::Failure(Error::InvalidArgument(
          "No optimized poses to save for agent " + std::string{agent_id}));
    }
    fs::path final_path = fs::path(output_save_dir) /
        ("optimized_poses_" + std::string{agent_id} + ".txt");
    fs::path temporary = final_path;
    temporary += ".tmp";
    std::error_code ignored;
    fs::remove(temporary, ignored);
    pending_files.emplace_back(temporary, final_path);

    std::ofstream file(temporary);
    if (!file) {
      cleanup();
      return Result<void>::Failure(Error::IoError(
          "failed to open temporary pose output: " + temporary.string()));
    }
    for (const auto& pose : opt_data.optimized_poses) {
      if (cancellation_ && cancellation_->IsCancellationRequested()) {
        file.close();
        cleanup();
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
      cleanup();
      return Result<void>::Failure(Error::IoError(
          "failed to write pose output: " + temporary.string()));
    }
  }

  if (cancellation_ && cancellation_->IsCancellationRequested()) {
    cleanup();
    return Result<void>::Failure(Error::Cancelled("before pose file commit"));
  }
  // Cancellation is intentionally not observed inside this short commit loop:
  // once commit starts, the complete deterministic file set is installed.
  for (const auto& [temporary, final_path] : pending_files) {
    std::error_code rename_error;
    fs::rename(temporary, final_path, rename_error);
    if (rename_error) {
      cleanup();
      return Result<void>::Failure(Error::IoError(
          "failed to commit pose output: " + rename_error.message()));
    }
  }
  return Result<void>::Ok();
}

Result<void> MapServer::saveOptimizedMap(const std::string& output_save_dir) {
  OPEN_LMM_ZONE_N("Save.FallbackMap");
  for (const auto& [agent_id, opt_data] : shared_data_->optimized_data) {
    const auto raw_it = shared_data_->raw_data.find(agent_id);
    if (raw_it == shared_data_->raw_data.end()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "Optimized agent has no corresponding raw data"));
    }
    const auto& filtered_scans = raw_it->second.filtered_scans;

    pcl::PointCloud<pcl::PointXYZI>::Ptr optimized_map(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& pose : opt_data.optimized_poses) {
      if (cancellation_ && cancellation_->IsCancellationRequested()) {
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
        fs::path(output_save_dir_) /
        ("global_map_" + std::string{agent_id} + ".pcd");
    fs::path temporary = output_map_file;
    temporary += ".tmp";
    std::error_code ignored;
    fs::remove(temporary, ignored);
    if (pcl::io::savePCDFileBinaryCompressed(temporary, *optimized_map) != 0) {
      fs::remove(temporary, ignored);
      return Result<void>::Failure(Error::IoError(
          "failed to save temporary map output: " + temporary.string()));
    }
    if (cancellation_ && cancellation_->IsCancellationRequested()) {
      fs::remove(temporary, ignored);
      return Result<void>::Failure(Error::Cancelled("before map file commit"));
    }
    std::error_code rename_error;
    fs::rename(temporary, output_map_file, rename_error);
    if (rename_error) {
      fs::remove(temporary, ignored);
      return Result<void>::Failure(Error::IoError(
          "failed to commit map output: " + rename_error.message()));
    }
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
