
#include "map_server.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <open_lmm/common/pointcloud_utils.hpp>
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
  if (initialization_error_) {
    return Result<void>::Failure(*initialization_error_);
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
  auto contexts = buildContexts();

  auto align_loader_result =
      DataLoaderBase::createInstance(config_data_loader_.value());
  if (!align_loader_result) {
    return Result<void>::Failure(align_loader_result.GetError());
  }

  //! Phase 1: Alignment Pipeline (DataLoad → LoopDetect → Optimize)
  Pipeline align_pipeline;
  align_pipeline
    .AddNode(std::make_unique<DataLoadNode>(
        std::move(align_loader_result).Value()))
    .AddNode(std::make_unique<LoopDetectNode>(
        [cfg = config_loop_detector_.value()]() {
          return LoopDetectorBase::createInstance(cfg);
        }))
    .AddNode(std::make_unique<OptimizeNode>(backend_optimizer_));

  auto align_result = align_pipeline.Run(contexts, *shared_data_);
  if (!align_result) {
    return Result<void>::Failure(align_result.GetError());
  }

  //! Phase 2: Map Update Pipeline (optional, 모든 에이전트 Optimize 완료 후)
  if (enable_map_updater_) {
    auto update_loader_result =
        DataLoaderBase::createInstance(config_data_loader_.value());
    if (!update_loader_result) {
      return Result<void>::Failure(update_loader_result.GetError());
    }
    Pipeline update_pipeline;
    update_pipeline.AddNode(std::make_unique<MapUpdateNode>(
        std::move(update_loader_result).Value(),
        [cfg = config_dynamic_remover_.value()]() {
          return DynamicRemoverBase::createInstance(cfg);
        },
        output_save_dir_,
        save_voxel_size_));

    auto update_result = update_pipeline.Run(contexts, *shared_data_);
    if (!update_result) {
      return Result<void>::Failure(update_result.GetError());
    }
  }

  std::cout << "SAVING OPTIMIZED POSES & MAPS" << std::endl;
  auto pose_save_result = saveOptimizedPoses(output_save_dir_);
  if (!pose_save_result) return pose_save_result;
  if (!enable_map_updater_) {
    auto save_result = saveOptimizedMap(output_save_dir_);
    if (!save_result) return save_result;
  }
  shared_data_->descriptor_store.total_db.clear();
  shared_data_.reset();
  std::cout << "ALL PROCESSES DONE" << std::endl;
  return Result<void>::Ok();
}

Result<void> MapServer::saveOptimizedPoses(const std::string& output_save_dir) {
  for (const auto& [agent_id, opt_data] : shared_data_->optimized_data) {
    fs::path output_save_dir_path(output_save_dir);
    fs::path output_pose_file =
        output_save_dir_path /
        ("optimized_poses_" + std::string{agent_id} + ".txt");
    if (opt_data.optimized_poses.empty()) {
      return Result<void>::Failure(Error::InvalidArgument(
          "No optimized poses to save for agent " + std::string{agent_id}));
    }
    std::ofstream file(output_pose_file);
    if (!file) {
      return Result<void>::Failure(Error::IoError(
          "failed to open pose output: " + output_pose_file.string()));
    }

    for (const auto& pose : opt_data.optimized_poses) {
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
      return Result<void>::Failure(Error::IoError(
          "failed to write pose output: " + output_pose_file.string()));
    }
  }
  return Result<void>::Ok();
}

Result<void> MapServer::saveOptimizedMap(const std::string& output_save_dir) {
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
    if (pcl::io::savePCDFileBinaryCompressed(output_map_file, *optimized_map) != 0) {
      return Result<void>::Failure(Error::IoError(
          "failed to save map output: " + output_map_file.string()));
    }
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
