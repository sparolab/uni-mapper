#pragma once

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/runtime_api.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <open_lmm/server/runtime_client.hpp>

#include <builtin_interfaces/msg/time.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace open_lmm {

[[nodiscard]] Result<std::string> EncodeRosAgentKey(const AgentId& agent);
[[nodiscard]] Result<sensor_msgs::msg::PointCloud2> ToRosPointCloud(
    const VisualizationSnapshot& snapshot, const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp, std::size_t maximum_points);
[[nodiscard]] Result<nav_msgs::msg::Path> ToRosPath(
    const VisualizationSnapshot& snapshot, const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp);
[[nodiscard]] Result<visualization_msgs::msg::MarkerArray> ToRosLoopMarkers(
    const std::map<AgentId, VisualizationSnapshot>& snapshots,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp,
    double line_width_m);
[[nodiscard]] Result<visualization_msgs::msg::MarkerArray>
ToRosLoopDeleteMarkers(const std::string& frame_id,
                       const builtin_interfaces::msg::Time& stamp);
[[nodiscard]] bool AcceptRosVisualizationReplacement(
    uint64_t generation, uint64_t latest_requested_generation,
    uint64_t candidate_revision, uint64_t published_revision);

// Private leaf-adapter owner. It derives ROS presentation from the one
// RuntimeClient owned by OpenLMMROS and never owns committed runtime state.
class RosVisualizationBridge final {
 public:
  RosVisualizationBridge(rclcpp::Node& node,
                         std::shared_ptr<RuntimeClient> runtime);
  ~RosVisualizationBridge();

  RosVisualizationBridge(const RosVisualizationBridge&) = delete;
  RosVisualizationBridge& operator=(const RosVisualizationBridge&) = delete;

  void Start();
  void RequestRefresh(const ExecutionEvent& event);
  void Stop();

 private:
  struct AgentPublisher;

  void Run(std::stop_token stop);
  void RefreshAll(uint64_t generation);
  void RefreshAgent(const AgentId& agent, uint64_t generation);
  void PublishRemoval(const AgentId& agent);
  void PublishLoops(const builtin_interfaces::msg::Time& stamp);
  [[nodiscard]] bool IsLatest(const AgentId& agent,
                              uint64_t generation) const;
  [[nodiscard]] std::shared_ptr<AgentPublisher> PublisherFor(
      const AgentId& agent);
  void Warn(const std::string& message) const;

  rclcpp::Node& node_;
  std::shared_ptr<RuntimeClient> runtime_;
  bool enabled_ = true;
  std::string frame_id_;
  float preview_voxel_size_m_ = 0.4F;
  std::size_t maximum_points_ = 2'000'000;
  double loop_line_width_m_ = 0.03;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
      loop_publisher_;

  mutable std::mutex mutex_;
  std::condition_variable_any wake_;
  bool started_ = false;
  bool refresh_all_ = false;
  uint64_t next_generation_ = 0;
  uint64_t refresh_all_generation_ = 0;
  std::map<AgentId, uint64_t> pending_;
  std::map<AgentId, uint64_t> latest_requested_;
  std::map<AgentId, std::shared_ptr<AgentPublisher>> publishers_;
  std::map<AgentId, VisualizationSnapshot> published_snapshots_;
  std::jthread worker_;
};

}  // namespace open_lmm
