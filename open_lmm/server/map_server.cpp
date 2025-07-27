
#include "map_server.hpp"

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Symbol.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include <open_lmm/common/pointcloud_utils.hpp>
#include <open_lmm/utils/config.hpp>

namespace open_lmm {

MapServer::MapServer() { parseConfig(); }

MapServer::~MapServer() {}

void MapServer::parseConfig() {
  const fs::path root_data_dir = fs::path(GlobalConfig::get_root_data_dir());
  for (const std::string& sub_dir : GlobalConfig::get_sub_dir_list()) {
    data_dir_list_.push_back(fs::path(root_data_dir / sub_dir));
  }

  agent_num_ = data_dir_list_.size();

  // TODO(gil) : here?
  output_save_dir_ = GlobalConfig::get_save_dir_path();
  fs::create_directories(output_save_dir_);

  config_map_server_ =
      Config(GlobalConfig::get_global_config_path("config_map_server"));
  enable_map_updater_ =
      config_map_server_->param<bool>("map_server", "enable_map_updater", true);
}

// TODO(gil) : refactor process
// TODO(gil): Refactor to remove centralized shared_data_ structure
void MapServer::process() {
  for (int i = 0; i < agent_num_; i++) {
    const char robot_id = 'A' + i;
    MapAlginer map_aligner(robot_id, shared_data_);
    map_aligner.process(data_dir_list_[i]);
  }

  if (enable_map_updater_) {
    for (int i = 0; i < agent_num_; i++) {
      const char robot_id = 'A' + i;
      MapUpdater map_updater(robot_id, shared_data_);
      auto static_map = map_updater.process(data_dir_list_[i]);
      fs::path output_save_dir_path(output_save_dir_);
      fs::path output_map_file =
          output_save_dir_path /
          ("global_map_" + std::string{robot_id} + ".pcd");
      auto ds_static_map =
          downsampleWithRangeFilter(static_map, 0.2, 0, 0, false);
      pcl::io::savePCDFileBinaryCompressed(output_map_file, *ds_static_map);
    }
  }

  std::cout << "SAVING OPTIMIZED POSES & MAPS" << std::endl;
  saveOptimizedPoses(output_save_dir_);

  if (!enable_map_updater_) {
    saveOptimizedMap(output_save_dir_);
  }

  std::cout << "ALL PROCESSES DONE" << std::endl;
}

void MapServer::saveOptimizedPoses(const std::string& output_save_dir) {
  for (const auto& optimized_poses : shared_data_->db_optimized_poses) {
    fs::path output_save_dir_path(output_save_dir);
    const char agent_id = optimized_poses.first;
    fs::path output_pose_file =
        output_save_dir_path /
        ("optimized_poses_" + std::string{agent_id} + ".txt");
    std::ofstream file(output_pose_file);

    for (const auto& pose : optimized_poses.second) {
      int scan_idx = pose.first;
      Eigen::Matrix4d pose_matrix = pose.second.matrix();
      Eigen::Vector3d translation = pose_matrix.block<3, 1>(0, 3);
      Eigen::Quaterniond quaternion(pose_matrix.block<3, 3>(0, 0));
      // TODO(gil) : add save options (pose format, extension, delimiter)
      file << scan_idx << "," << translation.x() << "," << translation.y()
           << "," << translation.z() << "," << quaternion.x() << ","
           << quaternion.y() << "," << quaternion.z() << "," << quaternion.w()
           << "\n";
    }
    file.close();
  }
}

void MapServer::saveOptimizedMap(const std::string& output_save_dir) {
  for (const auto& optimized_poses : shared_data_->db_optimized_poses) {
    fs::path output_save_dir_path(output_save_dir_);
    const char agent_id = optimized_poses.first;

    pcl::PointCloud<pcl::PointXYZI>::Ptr optimized_map(
        new pcl::PointCloud<pcl::PointXYZI>);
    for (const auto& pose : optimized_poses.second) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr transformed_scan(
          new pcl::PointCloud<pcl::PointXYZI>);
      int scan_idx = pose.first;
      Eigen::Matrix4d pose_matrix = pose.second.matrix();
      pcl::transformPointCloud(*shared_data_->db_scans[agent_id].at(scan_idx),
                               *transformed_scan, pose_matrix);
      *optimized_map += *transformed_scan;
    }

    fs::path output_map_file =
        output_save_dir_path / ("global_map_" + std::string{agent_id} + ".pcd");
    pcl::io::savePCDFileBinaryCompressed(output_map_file, *optimized_map);
  }
}

}  // namespace open_lmm