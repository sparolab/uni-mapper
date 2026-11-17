#pragma once

#include "keyframe_recorder.hpp"

#include <open_lmm/server/runtime_client.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/string.hpp>

#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_set>
#include <vector>

namespace open_lmm {

class RosVisualizationBridge;

// Online leaf adapter (phase 1: periodic re-merge).
//
// Per agent it subscribes to an odometry front end (FAST-LIO style
// `cloud_registered_body` + `Odometry`), records keyframes into an OpenLMM
// file-based dataset, and on a timer re-opens the single RuntimeClient on a
// hard-linked snapshot of everything recorded so far, then runs RunAll.
// Committed runtime state stays owned by the runtime; this node only owns the
// recordings and the schedule. RViz output reuses RosVisualizationBridge and
// is refreshed only after a merge job completes, so a failed or in-flight
// merge never clears the last good presentation.
class OpenLMMOnline : public rclcpp::Node {
 public:
  explicit OpenLMMOnline(const rclcpp::NodeOptions& options);
  ~OpenLMMOnline() override;

 private:
  using Odometry = nav_msgs::msg::Odometry;
  using Cloud2 = sensor_msgs::msg::PointCloud2;

  struct AgentStream {
    std::string name;
    std::unique_ptr<online::KeyframeRecorder> recorder;
    rclcpp::Subscription<Cloud2>::SharedPtr cloud_sub;
    rclcpp::Subscription<Odometry>::SharedPtr odom_sub;
    // FAST-LIO publishes the body cloud and the odometry with the same stamp;
    // pair them by stamp in whichever order they arrive.
    std::deque<Odometry::ConstSharedPtr> odometry;
    std::deque<Cloud2::ConstSharedPtr> clouds;
    std::mutex mutex;
    std::size_t merged_keyframes = 0;

    // Live presentation (independent of merges): odometry path and an
    // accumulated keyframe map in the agent's local odom frame, drawn in the
    // shared map frame through map_T_local. map_T_local is identity until a
    // merge aligns the agent, then follows the latest merge result.
    std::mutex live_mutex;
    std::vector<Eigen::Isometry3d> live_path;
    online::KeyframeRecorder::Cloud live_map;
    std::unordered_set<std::uint64_t> live_voxels;
    Eigen::Isometry3d map_T_local = Eigen::Isometry3d::Identity();
    bool aligned = false;
    rclcpp::Publisher<Cloud2>::SharedPtr live_points_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr live_path_pub;
    rclcpp::Publisher<Cloud2>::SharedPtr live_scan_pub;
  };

  void OnCloud(AgentStream& agent, Cloud2::ConstSharedPtr cloud);
  void OnOdometry(AgentStream& agent, Odometry::ConstSharedPtr odometry);
  void DrainMatches(AgentStream& agent);
  void OnMeasurement(AgentStream& agent, const Cloud2& cloud,
                     const Odometry& odometry);
  void OnTimer();
  void PublishLive();
  void UpdateAlignment(AgentStream& agent, std::size_t keyframes);
  void MergeLoop(std::stop_token stop);
  bool RunMerge();
  std::filesystem::path WriteSnapshotConfig(
      const std::filesystem::path& snapshot_root,
      const std::vector<std::string>& agents) const;
  void PublishStatus(const std::string& text);

  std::filesystem::path base_config_;
  std::filesystem::path work_dir_;
  std::filesystem::path output_dir_;
  std::size_t min_keyframes_per_agent_ = 20;
  std::size_t min_new_keyframes_ = 10;
  std::size_t min_agents_ = 2;
  double max_stamp_gap_s_ = 0.02;
  int keep_snapshots_ = 2;
  std::string live_frame_;
  double live_voxel_m_ = 0.3;
  rclcpp::TimerBase::SharedPtr live_timer_;

  std::vector<std::unique_ptr<AgentStream>> agents_;
  std::shared_ptr<RuntimeClient> runtime_;
  std::unique_ptr<RosVisualizationBridge> bridge_;
  ExecutionEventSubscription subscription_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

  std::mutex merge_mutex_;
  std::condition_variable_any merge_wake_;
  bool merge_requested_ = false;
  std::atomic<bool> merging_{false};
  std::uint64_t merge_index_ = 0;
  std::jthread merge_worker_;
};

}  // namespace open_lmm
