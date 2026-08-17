
#include "map_server.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/common/profiling.hpp>
#include <open_lmm/utils/config.hpp>

// Pipeline nodes
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <open_lmm/server/nodes/data_load_node.hpp>
#include <open_lmm/server/nodes/loop_detect_node.hpp>
#include <open_lmm/server/nodes/map_update_node.hpp>
#include <open_lmm/server/nodes/optimize_node.hpp>

namespace open_lmm {

MapServer::MapServer() {
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
  std::cout << "ALL PROCESSES DONE" << std::endl;
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
  if (contexts_.empty()) contexts_ = buildContexts();
  return Result<void>::Ok();
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
    shared_data_->descriptor_store.total_db.clear();
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

Result<void> MapServer::ResetSession() {
  std::lock_guard lock(state_mutex_);
  auto ready = ensureReady();
  if (!ready) return ready;
  shared_data_ = std::make_shared<SharedDatabase>();
  backend_optimizer_->Reset();
  contexts_ = buildContexts();
  return Result<void>::Ok();
}

Result<void> MapServer::runDataLoadStage() {
  OPEN_LMM_ZONE_N("MapServer.DataLoadStage");
  shared_data_ = std::make_shared<SharedDatabase>();
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
  shared_data_->descriptor_store.total_db.clear();
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
  return pipeline.Run(contexts_, *shared_data_);
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
  std::cout << "SAVING OPTIMIZED POSES & MAPS" << std::endl;
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
