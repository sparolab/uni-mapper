#include "keyframe_recorder.hpp"

#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <cmath>
#include <cstdio>
#include <iomanip>
#include <sstream>
#include <stdexcept>
#include <system_error>
#include <utility>

namespace open_lmm::online {
namespace fs = std::filesystem;

namespace {

std::string ScanName(std::size_t index) {
  std::ostringstream name;
  name << std::setw(6) << std::setfill('0') << index << ".pcd";
  return name.str();
}

std::string KittiLine(const Eigen::Isometry3d& pose) {
  const Eigen::Matrix4d m = pose.matrix();
  std::ostringstream line;
  line << std::fixed << std::setprecision(6);
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 4; ++c) {
      if (r != 0 || c != 0) line << ' ';
      line << m(r, c);
    }
  }
  return line.str();
}

}  // namespace

KeyframeRecorder::KeyframeRecorder(fs::path directory, KeyframeParams params,
                                   std::string pose_file_name,
                                   std::string scan_dir_name)
    : directory_(std::move(directory)),
      params_(params),
      pose_file_name_(std::move(pose_file_name)),
      scan_dir_name_(std::move(scan_dir_name)) {
  if (params_.scans_per_keyframe == 0) params_.scans_per_keyframe = 1;
  fs::create_directories(directory_ / scan_dir_name_);
  const fs::path pose_path = directory_ / pose_file_name_;
  if (fs::exists(pose_path) && fs::file_size(pose_path) > 0) {
    throw std::runtime_error("refusing to append to existing recording: " +
                             pose_path.string());
  }
  pose_stream_.open(pose_path, std::ios::out | std::ios::trunc);
  if (!pose_stream_) {
    throw std::runtime_error("cannot open " + pose_path.string());
  }
}

bool KeyframeRecorder::IsKeyframe(const Eigen::Isometry3d& pose) const {
  if (!has_last_keyframe_) return true;
  const Eigen::Isometry3d delta = last_keyframe_.inverse() * pose;
  const double translation = delta.translation().norm();
  const double rotation_deg =
      Eigen::AngleAxisd(delta.rotation()).angle() * 180.0 / M_PI;
  return translation >= params_.min_translation_m ||
         rotation_deg >= params_.min_rotation_deg;
}

bool KeyframeRecorder::Add(const Eigen::Isometry3d& world_T_body,
                           const Cloud& body_cloud) {
  auto copy = std::make_shared<Cloud>(body_cloud);
  recent_.push_back(Frame{world_T_body, std::move(copy)});
  while (recent_.size() > params_.scans_per_keyframe) recent_.pop_front();
  if (!IsKeyframe(world_T_body)) return false;
  WriteKeyframe(world_T_body);
  return true;
}

void KeyframeRecorder::WriteKeyframe(const Eigen::Isometry3d& pose) {
  Cloud merged;
  const Eigen::Isometry3d body_T_world = pose.inverse();
  for (const auto& frame : recent_) {
    Cloud moved;
    const Eigen::Isometry3d kf_T_frame = body_T_world * frame.world_T_body;
    pcl::transformPointCloud(*frame.cloud, moved,
                             kf_T_frame.matrix().cast<float>());
    merged += moved;
  }
  Cloud filtered;
  if (params_.voxel_size_m > 0.0 && !merged.empty()) {
    pcl::VoxelGrid<pcl::PointXYZ> grid;
    const float leaf = static_cast<float>(params_.voxel_size_m);
    grid.setLeafSize(leaf, leaf, leaf);
    grid.setInputCloud(merged.makeShared());
    grid.filter(filtered);
  } else {
    filtered = std::move(merged);
  }
  filtered.width = static_cast<std::uint32_t>(filtered.size());
  filtered.height = 1;
  filtered.is_dense = true;

  std::size_t index = 0;
  {
    std::lock_guard lock(mutex_);
    index = keyframes_;
  }
  const fs::path scan = directory_ / scan_dir_name_ / ScanName(index);
  const fs::path temporary = fs::path(scan.string() + ".tmp");
  if (pcl::io::savePCDFileBinary(temporary.string(), filtered) != 0) {
    throw std::runtime_error("failed to write " + temporary.string());
  }
  fs::rename(temporary, scan);
  pose_stream_ << KittiLine(pose) << '\n';
  pose_stream_.flush();
  {
    std::lock_guard lock(mutex_);
    ++keyframes_;
  }
  has_last_keyframe_ = true;
  last_keyframe_ = pose;
}

std::size_t KeyframeRecorder::KeyframeCount() const {
  std::lock_guard lock(mutex_);
  return keyframes_;
}

std::size_t KeyframeRecorder::Snapshot(const fs::path& destination) const {
  const std::size_t count = KeyframeCount();
  fs::create_directories(destination / scan_dir_name_);
  for (std::size_t i = 0; i < count; ++i) {
    const fs::path source = directory_ / scan_dir_name_ / ScanName(i);
    const fs::path target = destination / scan_dir_name_ / ScanName(i);
    std::error_code error;
    fs::create_hard_link(source, target, error);
    if (error) fs::copy_file(source, target, fs::copy_options::overwrite_existing);
  }
  std::ifstream poses(directory_ / pose_file_name_);
  std::ofstream out(destination / pose_file_name_, std::ios::trunc);
  std::string line;
  for (std::size_t i = 0; i < count && std::getline(poses, line); ++i) {
    out << line << '\n';
  }
  return count;
}

}  // namespace open_lmm::online
