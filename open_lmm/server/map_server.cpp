
#include "map_server.hpp"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/utils/config.hpp>

// Pipeline nodes
#include <open_lmm/core/dynamic_remover/dynamic_remover_base.hpp>
#include <open_lmm/server/nodes/data_load_node.hpp>
#include <open_lmm/server/nodes/loop_detect_node.hpp>
#include <open_lmm/server/nodes/map_update_node.hpp>
#include <open_lmm/server/nodes/optimize_node.hpp>

namespace open_lmm {

MapServer::MapServer() { parseConfig(); }
MapServer::~MapServer() {}

void MapServer::parseConfig() {
  const fs::path root_data_dir = fs::path(GlobalConfig::get_root_data_dir());
  for (const std::string& sub_dir : GlobalConfig::get_sub_dir_list()) {
    data_dir_list_.push_back(fs::path(root_data_dir / sub_dir));
  }
  agent_num_ = data_dir_list_.size();

  output_save_dir_ = GlobalConfig::get_save_dir_path();
  fs::create_directories(output_save_dir_);

  config_map_server_ =
      Config(GlobalConfig::get_global_config_path("config_map_server"));
  enable_map_updater_ =
      config_map_server_->param<bool>("map_server", "enable_map_updater", true);
  anchor_agent_index_ =
      config_map_server_->param<int>("map_server", "anchor_agent_index", 0);
  save_voxel_size_ =
      config_map_server_->param<double>("map_server", "save_voxel_size", 0.2);

  config_data_loader_ =
      Config(GlobalConfig::get_global_config_path("config_data_loader"));
  config_loop_detector_ =
      Config(GlobalConfig::get_global_config_path("config_loop_detector"));
  config_dynamic_remover_ =
      Config(GlobalConfig::get_global_config_path("config_dynamic_remover"));

  Config config_backend_optimizer =
      Config(GlobalConfig::get_global_config_path("config_backend_optimizer"));
  backend_optimizer_ = std::shared_ptr<BackendOptimizerBase>(
      BackendOptimizerBase::createInstance(config_backend_optimizer));
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

void MapServer::process() {
  auto contexts = buildContexts();

  //! Phase 1: Alignment Pipeline (DataLoad → LoopDetect → Optimize)
  Pipeline align_pipeline;
  align_pipeline
    .AddNode(std::make_unique<DataLoadNode>(
        DataLoaderBase::createInstance(config_data_loader_.value())))
    .AddNode(std::make_unique<LoopDetectNode>(
        [cfg = config_loop_detector_.value()]() {
          return LoopDetectorBase::createInstance(cfg);
        }))
    .AddNode(std::make_unique<OptimizeNode>(backend_optimizer_));

  auto align_result = align_pipeline.Run(contexts, *shared_data_);
  if (!align_result) {
    spdlog::error("[MapServer] Alignment pipeline failed: {}",
                  align_result.GetError().Message());
    std::exit(1);
  }

  //! Phase 2: Map Update Pipeline (optional, 모든 에이전트 Optimize 완료 후)
  if (enable_map_updater_) {
    Pipeline update_pipeline;
    update_pipeline.AddNode(std::make_unique<MapUpdateNode>(
        DataLoaderBase::createInstance(config_data_loader_.value()),
        [cfg = config_dynamic_remover_.value()]() {
          return DynamicRemoverBase::createInstance(cfg);
        },
        output_save_dir_,
        save_voxel_size_));

    auto update_result = update_pipeline.Run(contexts, *shared_data_);
    if (!update_result) {
      spdlog::error("[MapServer] Map update pipeline failed: {}",
                    update_result.GetError().Message());
      std::exit(1);
    }
  }

  std::cout << "SAVING OPTIMIZED POSES & MAPS" << std::endl;
  saveOptimizedPoses(output_save_dir_);
  if (!enable_map_updater_) saveOptimizedMap(output_save_dir_);
  std::cout << "ALL PROCESSES DONE" << std::endl;
}

void MapServer::saveOptimizedPoses(const std::string& output_save_dir) {
  for (const auto& [agent_id, opt_data] : shared_data_->optimized_data) {
    fs::path output_save_dir_path(output_save_dir);
    fs::path output_pose_file =
        output_save_dir_path /
        ("optimized_poses_" + std::string{agent_id} + ".txt");
    std::ofstream file(output_pose_file);

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
    file.close();
  }
}

void MapServer::saveOptimizedMap(const std::string& output_save_dir) {
  for (const auto& [agent_id, opt_data] : shared_data_->optimized_data) {
    const auto& filtered_scans = shared_data_->raw_data.at(agent_id).filtered_scans;

    pcl::PointCloud<pcl::PointXYZI>::Ptr optimized_map(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& pose : opt_data.optimized_poses) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan(
          new pcl::PointCloud<pcl::PointXYZI>);
      int scan_idx = pose.first;
      Eigen::Matrix4d pose_matrix = pose.second.matrix();
      pcl::transformPointCloud(*filtered_scans.at(scan_idx),
                               *transformed_scan, pose_matrix);
      *optimized_map += *transformed_scan;
    }

    fs::path output_map_file =
        fs::path(output_save_dir_) /
        ("global_map_" + std::string{agent_id} + ".pcd");
    pcl::io::savePCDFileBinaryCompressed(output_map_file, *optimized_map);
  }
}

}  // namespace open_lmm
