#pragma once

#include <Eigen/Geometry>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <cstddef>
#include <deque>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

namespace open_lmm::online {

struct KeyframeParams {
  double min_translation_m = 1.0;
  double min_rotation_deg = 10.0;
  // Number of consecutive body-frame scans (ending at the keyframe) merged
  // into one keyframe cloud, expressed in the keyframe body frame.
  std::size_t scans_per_keyframe = 5;
  double voxel_size_m = 0.1;
};

// Private leaf-adapter helper. Turns a stream of (pose, body-frame cloud)
// pairs from an odometry front end into an OpenLMM file-based agent
// directory: <dir>/Scans/NNNNNN.pcd plus <dir>/<pose_file> (KITTI 3x4 rows).
// Files are only ever appended; a keyframe becomes visible to snapshots once
// both its scan and its pose line are on disk.
class KeyframeRecorder {
 public:
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  KeyframeRecorder(std::filesystem::path directory, KeyframeParams params,
                   std::string pose_file_name = "optimized_poses.txt",
                   std::string scan_dir_name = "Scans");

  // Returns true when this measurement produced a new keyframe.
  bool Add(const Eigen::Isometry3d& world_T_body, const Cloud& body_cloud);

  [[nodiscard]] std::size_t KeyframeCount() const;
  // Pose of keyframe `index` as written to the pose file (local odom frame).
  [[nodiscard]] bool KeyframePose(std::size_t index,
                                  Eigen::Isometry3d& pose) const;
  // Filtered cloud of the most recent keyframe, in that keyframe's frame.
  [[nodiscard]] Cloud::ConstPtr LastKeyframeCloud() const;
  [[nodiscard]] const std::filesystem::path& Directory() const {
    return directory_;
  }

  // Materialises the first `count` keyframes into `destination` using hard
  // links for scans (falls back to copies) and a truncated pose file.
  // Returns the number of keyframes written.
  std::size_t Snapshot(const std::filesystem::path& destination) const;

 private:
  struct Frame {
    Eigen::Isometry3d world_T_body;
    Cloud::Ptr cloud;
  };

  [[nodiscard]] bool IsKeyframe(const Eigen::Isometry3d& pose) const;
  void WriteKeyframe(const Eigen::Isometry3d& pose);

  std::filesystem::path directory_;
  KeyframeParams params_;
  std::string pose_file_name_;
  std::string scan_dir_name_;
  std::deque<Frame> recent_;
  bool has_last_keyframe_ = false;
  Eigen::Isometry3d last_keyframe_ = Eigen::Isometry3d::Identity();
  std::ofstream pose_stream_;
  mutable std::mutex mutex_;
  std::size_t keyframes_ = 0;
  std::vector<Eigen::Isometry3d> keyframe_poses_;
  Cloud::ConstPtr last_cloud_;
};

}  // namespace open_lmm::online
