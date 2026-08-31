#include "ros_visualization_bridge.hpp"

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <visualization_msgs/msg/marker.hpp>

#include <Eigen/Geometry>

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstring>
#include <limits>
#include <set>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

namespace open_lmm {
namespace {

bool IsAsciiAlphanumeric(unsigned char value) {
  return (value >= 'a' && value <= 'z') ||
         (value >= 'A' && value <= 'Z') ||
         (value >= '0' && value <= '9');
}

void AppendFloat(std::vector<uint8_t>& bytes, std::size_t offset, float value) {
  std::memcpy(bytes.data() + offset, &value, sizeof(value));
}

geometry_msgs::msg::Point Point(double x, double y, double z) {
  geometry_msgs::msg::Point result;
  result.x = x;
  result.y = y;
  result.z = z;
  return result;
}

bool FiniteTransform(const Eigen::Isometry3f& transform) {
  return transform.matrix().allFinite();
}

bool RefreshEvent(EventType type) {
  switch (type) {
    case EventType::kArtifactCommitted:
    case EventType::kArtifactInvalidated:
    case EventType::kStageCompleted:
    case EventType::kJobCompleted: return true;
    default: return false;
  }
}

VisualizationSnapshot MetadataOnly(VisualizationSnapshot snapshot) {
  snapshot.points.clear();
  snapshot.points.shrink_to_fit();
  return snapshot;
}

}  // namespace

Result<std::string> EncodeRosAgentKey(const AgentId& agent) {
  if (!agent.IsValid()) {
    return Result<std::string>::Failure(
        Error::InvalidArgument("RViz topic requires a valid agent id"));
  }
  std::string result = "a_";
  result.reserve(2 + agent.Value().size() * 3);
  constexpr char kHex[] = "0123456789abcdef";
  for (const unsigned char value : agent.Value()) {
    if (IsAsciiAlphanumeric(value)) {
      result.push_back(static_cast<char>(value));
      continue;
    }
    if (value != '_' && value != '-' && value != '.') {
      return Result<std::string>::Failure(Error::InvalidArgument(
          "RViz topic cannot encode an invalid agent id byte"));
    }
    result.push_back('_');
    result.push_back(kHex[(value >> 4U) & 0x0fU]);
    result.push_back(kHex[value & 0x0fU]);
  }
  return Result<std::string>::Ok(std::move(result));
}

Result<sensor_msgs::msg::PointCloud2> ToRosPointCloud(
    const VisualizationSnapshot& snapshot, const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp, std::size_t maximum_points) {
  if (frame_id.empty()) {
    return Result<sensor_msgs::msg::PointCloud2>::Failure(
        Error::InvalidArgument("RViz frame id must not be empty"));
  }
  if (maximum_points == 0 || snapshot.points.size() > maximum_points ||
      snapshot.points.size() > std::numeric_limits<uint32_t>::max()) {
    return Result<sensor_msgs::msg::PointCloud2>::Failure(
        Error::InvalidArgument("RViz point payload exceeds the configured limit"));
  }

  sensor_msgs::msg::PointCloud2 message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  message.height = 1;
  message.width = static_cast<uint32_t>(snapshot.points.size());
  message.is_bigendian = std::endian::native == std::endian::big;
  message.is_dense = false;
  message.point_step = 16;
  message.row_step = message.width * message.point_step;
  message.fields.resize(4);
  constexpr const char* kNames[] = {"x", "y", "z", "intensity"};
  for (std::size_t index = 0; index < message.fields.size(); ++index) {
    message.fields[index].name = kNames[index];
    message.fields[index].offset = static_cast<uint32_t>(index * sizeof(float));
    message.fields[index].datatype = sensor_msgs::msg::PointField::FLOAT32;
    message.fields[index].count = 1;
  }
  message.data.resize(static_cast<std::size_t>(message.row_step));
  for (std::size_t index = 0; index < snapshot.points.size(); ++index) {
    const auto& point = snapshot.points[index];
    if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
        !std::isfinite(point.z) || !std::isfinite(point.intensity)) {
      return Result<sensor_msgs::msg::PointCloud2>::Failure(
          Error::InvalidArgument("RViz point payload contains a non-finite value"));
    }
    const std::size_t offset = index * message.point_step;
    AppendFloat(message.data, offset, point.x);
    AppendFloat(message.data, offset + 4, point.y);
    AppendFloat(message.data, offset + 8, point.z);
    AppendFloat(message.data, offset + 12, point.intensity);
  }
  return Result<sensor_msgs::msg::PointCloud2>::Ok(std::move(message));
}

Result<nav_msgs::msg::Path> ToRosPath(
    const VisualizationSnapshot& snapshot, const std::string& frame_id,
    const builtin_interfaces::msg::Time& stamp) {
  if (frame_id.empty()) {
    return Result<nav_msgs::msg::Path>::Failure(
        Error::InvalidArgument("RViz frame id must not be empty"));
  }
  nav_msgs::msg::Path message;
  message.header.frame_id = frame_id;
  message.header.stamp = stamp;
  message.poses.reserve(snapshot.poses.size());
  for (const auto& pose : snapshot.poses) {
    if (!FiniteTransform(pose.transform)) {
      return Result<nav_msgs::msg::Path>::Failure(
          Error::InvalidArgument("RViz path contains a non-finite transform"));
    }
    Eigen::Quaternionf orientation(pose.transform.linear());
    if (!orientation.coeffs().allFinite() ||
        orientation.norm() <= std::numeric_limits<float>::epsilon()) {
      return Result<nav_msgs::msg::Path>::Failure(
          Error::InvalidArgument("RViz path contains an invalid orientation"));
    }
    orientation.normalize();
    geometry_msgs::msg::PoseStamped output;
    output.header = message.header;
    const auto translation = pose.transform.translation();
    output.pose.position = Point(translation.x(), translation.y(), translation.z());
    output.pose.orientation.x = orientation.x();
    output.pose.orientation.y = orientation.y();
    output.pose.orientation.z = orientation.z();
    output.pose.orientation.w = orientation.w();
    message.poses.push_back(std::move(output));
  }
  return Result<nav_msgs::msg::Path>::Ok(std::move(message));
}

Result<visualization_msgs::msg::MarkerArray> ToRosLoopMarkers(
    const std::map<AgentId, VisualizationSnapshot>& snapshots,
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp,
    double line_width_m) {
  if (frame_id.empty() || !std::isfinite(line_width_m) || line_width_m <= 0.0) {
    return Result<visualization_msgs::msg::MarkerArray>::Failure(
        Error::InvalidArgument("RViz loop marker parameters are invalid"));
  }

  using PoseKey = std::pair<AgentId, std::size_t>;
  std::map<PoseKey, geometry_msgs::msg::Point> poses;
  for (const auto& [agent, snapshot] : snapshots) {
    (void)agent;
    for (const auto& pose : snapshot.poses) {
      if (pose.index < 0 || !FiniteTransform(pose.transform)) {
        return Result<visualization_msgs::msg::MarkerArray>::Failure(
            Error::InvalidArgument("RViz loop endpoint pose is invalid"));
      }
      const auto translation = pose.transform.translation();
      poses[{snapshot.agent, static_cast<std::size_t>(pose.index)}] =
          Point(translation.x(), translation.y(), translation.z());
    }
  }

  visualization_msgs::msg::Marker intra;
  intra.header.frame_id = frame_id;
  intra.header.stamp = stamp;
  intra.ns = "open_lmm/intra_loop";
  intra.id = 0;
  intra.type = visualization_msgs::msg::Marker::LINE_LIST;
  intra.action = visualization_msgs::msg::Marker::ADD;
  intra.pose.orientation.w = 1.0;
  intra.scale.x = line_width_m;
  intra.color.r = 0.10F;
  intra.color.g = 0.90F;
  intra.color.b = 0.25F;
  intra.color.a = 1.0F;

  visualization_msgs::msg::Marker inter = intra;
  inter.ns = "open_lmm/inter_loop";
  inter.color.r = 0.95F;
  inter.color.g = 0.20F;
  inter.color.b = 0.90F;

  using EdgeKey = std::tuple<uint8_t, AgentId, std::size_t, AgentId, std::size_t>;
  std::set<EdgeKey> emitted;
  for (const auto& [agent, snapshot] : snapshots) {
    (void)agent;
    for (const auto& edge : snapshot.edges) {
      if (edge.type == VisualizationEdgeType::kTrajectory) continue;
      const EdgeKey key{static_cast<uint8_t>(edge.type), edge.from_agent,
                        edge.from_index, edge.to_agent, edge.to_index};
      if (!emitted.insert(key).second) continue;
      const auto from = poses.find({edge.from_agent, edge.from_index});
      const auto to = poses.find({edge.to_agent, edge.to_index});
      if (from == poses.end() || to == poses.end()) continue;
      auto& marker = edge.type == VisualizationEdgeType::kIntraLoop ? intra : inter;
      marker.points.push_back(from->second);
      marker.points.push_back(to->second);
    }
  }

  visualization_msgs::msg::MarkerArray message;
  message.markers.push_back(std::move(intra));
  message.markers.push_back(std::move(inter));
  return Result<visualization_msgs::msg::MarkerArray>::Ok(std::move(message));
}

Result<visualization_msgs::msg::MarkerArray> ToRosLoopDeleteMarkers(
    const std::string& frame_id, const builtin_interfaces::msg::Time& stamp) {
  if (frame_id.empty()) {
    return Result<visualization_msgs::msg::MarkerArray>::Failure(
        Error::InvalidArgument("RViz frame id must not be empty"));
  }
  visualization_msgs::msg::MarkerArray message;
  for (const char* marker_namespace : {"open_lmm/intra_loop",
                                       "open_lmm/inter_loop"}) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = stamp;
    marker.ns = marker_namespace;
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    message.markers.push_back(std::move(marker));
  }
  return Result<visualization_msgs::msg::MarkerArray>::Ok(std::move(message));
}

bool AcceptRosVisualizationReplacement(uint64_t generation,
                                       uint64_t latest_requested_generation,
                                       uint64_t candidate_revision,
                                       uint64_t published_revision) {
  return generation == latest_requested_generation &&
         candidate_revision >= published_revision;
}

struct RosVisualizationBridge::AgentPublisher {
  AgentId agent;
  std::string key;
  uint64_t published_revision = 0;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr points;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path;
};

RosVisualizationBridge::RosVisualizationBridge(
    rclcpp::Node& node, std::shared_ptr<RuntimeClient> runtime)
    : node_(node), runtime_(std::move(runtime)) {
  enabled_ = node_.declare_parameter<bool>("rviz_visualization_enabled", true);
  frame_id_ = node_.declare_parameter<std::string>("rviz_frame_id", "map");
  const double voxel =
      node_.declare_parameter<double>("rviz_preview_voxel_size_m", 0.4);
  const int64_t maximum =
      node_.declare_parameter<int64_t>("rviz_max_point_count", 2'000'000);
  loop_line_width_m_ =
      node_.declare_parameter<double>("rviz_loop_line_width_m", 0.03);
  if (frame_id_.empty() || !std::isfinite(voxel) || voxel < 0.0 ||
      maximum < 1 || maximum > 2'000'000 ||
      !std::isfinite(loop_line_width_m_) || loop_line_width_m_ <= 0.0) {
    throw std::invalid_argument("invalid RViz visualization parameter");
  }
  preview_voxel_size_m_ = static_cast<float>(voxel);
  maximum_points_ = static_cast<std::size_t>(maximum);
  if (!enabled_) return;
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  loop_publisher_ = node_.create_publisher<visualization_msgs::msg::MarkerArray>(
      "~/visualization/loops", qos);
}

RosVisualizationBridge::~RosVisualizationBridge() { Stop(); }

void RosVisualizationBridge::Start() {
  if (!enabled_) return;
  std::lock_guard lock(mutex_);
  if (started_) return;
  started_ = true;
  refresh_all_ = true;
  refresh_all_generation_ = ++next_generation_;
  worker_ = std::jthread([this](std::stop_token stop) { Run(stop); });
  wake_.notify_all();
}

void RosVisualizationBridge::RequestRefresh(const ExecutionEvent& event) {
  if (!enabled_ || !RefreshEvent(event.type)) return;
  std::lock_guard lock(mutex_);
  if (!started_) return;
  const uint64_t generation = ++next_generation_;
  if (event.affected_agents.empty()) {
    refresh_all_ = true;
    refresh_all_generation_ = generation;
  } else {
    for (const auto& agent : event.affected_agents) {
      latest_requested_[agent] = generation;
      pending_[agent] = generation;
    }
  }
  wake_.notify_all();
}

void RosVisualizationBridge::Stop() {
  std::jthread worker;
  {
    std::lock_guard lock(mutex_);
    if (!started_) return;
    started_ = false;
    worker = std::move(worker_);
  }
  worker.request_stop();
  wake_.notify_all();
  if (worker.joinable()) worker.join();
  std::lock_guard lock(mutex_);
  pending_.clear();
  latest_requested_.clear();
  publishers_.clear();
  published_snapshots_.clear();
}

void RosVisualizationBridge::Run(std::stop_token stop) {
  while (!stop.stop_requested()) {
    bool all = false;
    uint64_t all_generation = 0;
    std::map<AgentId, uint64_t> pending;
    {
      std::unique_lock lock(mutex_);
      wake_.wait(lock, stop, [this] { return refresh_all_ || !pending_.empty(); });
      if (stop.stop_requested()) break;
      all = std::exchange(refresh_all_, false);
      all_generation = refresh_all_generation_;
      pending.swap(pending_);
    }
    if (all) RefreshAll(all_generation);
    for (const auto& [agent, generation] : pending) {
      if (stop.stop_requested()) break;
      RefreshAgent(agent, generation);
    }
    if (!pending.empty() && !stop.stop_requested()) {
      const builtin_interfaces::msg::Time stamp = node_.now();
      PublishLoops(stamp);
    }
  }
}

void RosVisualizationBridge::RefreshAll(uint64_t generation) {
  auto snapshot = runtime_->Snapshot();
  if (!snapshot) {
    Warn("RViz refresh failed: " + snapshot.GetError().Message());
    return;
  }
  std::set<AgentId> current(snapshot.Value().pipeline.agents.begin(),
                            snapshot.Value().pipeline.agents.end());
  std::vector<AgentId> removed;
  {
    std::lock_guard lock(mutex_);
    for (const auto& [agent, publisher] : publishers_) {
      (void)publisher;
      if (!current.contains(agent)) removed.push_back(agent);
    }
    for (const auto& agent : current) {
      auto& latest = latest_requested_[agent];
      latest = std::max(latest, generation);
    }
  }
  for (const auto& agent : removed) PublishRemoval(agent);
  for (const auto& agent : current) RefreshAgent(agent, generation);
  const builtin_interfaces::msg::Time stamp = node_.now();
  PublishLoops(stamp);
}

void RosVisualizationBridge::RefreshAgent(const AgentId& agent,
                                          uint64_t generation) {
  VisualizationQuery query;
  query.agent = agent;
  query.include_points = true;
  query.preview_voxel_size_m = preview_voxel_size_m_;
  query.maximum_points = maximum_points_;
  auto result = runtime_->Visualization(query);
  if (!result) {
    Warn("RViz visualization for " + agent.Value() + " failed: " +
         result.GetError().Message());
    return;
  }
  if (!IsLatest(agent, generation)) return;

  VisualizationSnapshot snapshot = std::move(result).Value();
  const builtin_interfaces::msg::Time stamp = node_.now();
  auto points = ToRosPointCloud(snapshot, frame_id_, stamp, maximum_points_);
  auto path = ToRosPath(snapshot, frame_id_, stamp);
  if (!points || !path) {
    const auto& error = points ? path.GetError() : points.GetError();
    Warn("RViz conversion for " + agent.Value() + " failed: " +
         error.Message());
    return;
  }
  if (!IsLatest(agent, generation)) return;

  auto publisher = PublisherFor(agent);
  if (!publisher) return;
  {
    std::lock_guard lock(mutex_);
    const auto latest = latest_requested_.find(agent);
    if (!started_ || latest == latest_requested_.end() ||
        !AcceptRosVisualizationReplacement(
            generation, latest->second, snapshot.revision,
            publisher->published_revision)) {
      return;
    }
    publisher->points->publish(std::move(points).Value());
    publisher->path->publish(std::move(path).Value());
    publisher->published_revision = snapshot.revision;
    published_snapshots_[agent] = MetadataOnly(std::move(snapshot));
  }
}

void RosVisualizationBridge::PublishRemoval(const AgentId& agent) {
  std::shared_ptr<AgentPublisher> publisher;
  {
    std::lock_guard lock(mutex_);
    const auto found = publishers_.find(agent);
    if (found == publishers_.end()) return;
    publisher = found->second;
  }
  const builtin_interfaces::msg::Time stamp = node_.now();
  sensor_msgs::msg::PointCloud2 points;
  points.header.frame_id = frame_id_;
  points.header.stamp = stamp;
  points.height = 1;
  points.is_dense = false;
  publisher->points->publish(points);
  nav_msgs::msg::Path path;
  path.header = points.header;
  publisher->path->publish(path);
  {
    std::lock_guard lock(mutex_);
    publishers_.erase(agent);
    published_snapshots_.erase(agent);
    latest_requested_.erase(agent);
    pending_.erase(agent);
  }
  auto deletes = ToRosLoopDeleteMarkers(frame_id_, stamp);
  if (deletes) loop_publisher_->publish(std::move(deletes).Value());
  PublishLoops(stamp);
}

void RosVisualizationBridge::PublishLoops(
    const builtin_interfaces::msg::Time& stamp) {
  std::map<AgentId, VisualizationSnapshot> snapshots;
  {
    std::lock_guard lock(mutex_);
    snapshots = published_snapshots_;
  }
  auto markers =
      ToRosLoopMarkers(snapshots, frame_id_, stamp, loop_line_width_m_);
  if (!markers) {
    Warn("RViz loop conversion failed: " + markers.GetError().Message());
    return;
  }
  loop_publisher_->publish(std::move(markers).Value());
}

bool RosVisualizationBridge::IsLatest(const AgentId& agent,
                                      uint64_t generation) const {
  std::lock_guard lock(mutex_);
  const auto found = latest_requested_.find(agent);
  return started_ && found != latest_requested_.end() &&
         found->second == generation;
}

std::shared_ptr<RosVisualizationBridge::AgentPublisher>
RosVisualizationBridge::PublisherFor(const AgentId& agent) {
  std::lock_guard lock(mutex_);
  const auto found = publishers_.find(agent);
  if (found != publishers_.end()) return found->second;
  auto key = EncodeRosAgentKey(agent);
  if (!key) {
    Warn(key.GetError().Message());
    return {};
  }
  auto result = std::make_shared<AgentPublisher>();
  result->agent = agent;
  result->key = std::move(key).Value();
  const auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local();
  const std::string prefix = "~/visualization/" + result->key;
  result->points = node_.create_publisher<sensor_msgs::msg::PointCloud2>(
      prefix + "/points", qos);
  result->path =
      node_.create_publisher<nav_msgs::msg::Path>(prefix + "/path", qos);
  publishers_[agent] = result;
  return result;
}

void RosVisualizationBridge::Warn(const std::string& message) const {
  RCLCPP_WARN(node_.get_logger(), "%s", message.c_str());
}

}  // namespace open_lmm
