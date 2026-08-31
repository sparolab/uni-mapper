#include "../../ros2/open_lmm_ros/runtime_adapter/ros_visualization_bridge.hpp"

#include <Eigen/Geometry>

#include <cmath>
#include <cstring>
#include <iostream>
#include <limits>
#include <map>
#include <stdexcept>
#include <string>

namespace {

void Check(bool condition, const std::string& message) {
  if (!condition) throw std::runtime_error(message);
}

open_lmm::AgentId Agent(const std::string& value) {
  auto parsed = open_lmm::AgentId::Parse(value);
  Check(parsed.IsOk(), "test agent id parses");
  return std::move(parsed).Value();
}

float ReadFloat(const sensor_msgs::msg::PointCloud2& cloud,
                std::size_t offset) {
  float value = 0.0F;
  std::memcpy(&value, cloud.data.data() + offset, sizeof(value));
  return value;
}

open_lmm::VisualizationPose Pose(int index, float x, float y, float z) {
  open_lmm::VisualizationPose result;
  result.index = index;
  result.transform = Eigen::Isometry3f::Identity();
  result.transform.translation() = Eigen::Vector3f(x, y, z);
  return result;
}

void TestAgentKeyEncoding() {
  auto dash = open_lmm::EncodeRosAgentKey(Agent("agent-1"));
  auto underscore = open_lmm::EncodeRosAgentKey(Agent("agent_1"));
  auto dot = open_lmm::EncodeRosAgentKey(Agent("agent.1"));
  Check(dash && dash.Value() == "a_agent_2d1", "dash is hex escaped");
  Check(underscore && underscore.Value() == "a_agent_5f1",
        "underscore is hex escaped");
  Check(dot && dot.Value() == "a_agent_2e1", "dot is hex escaped");
  Check(dash.Value() != underscore.Value() && dash.Value() != dot.Value(),
        "allowed punctuation cannot collide in ROS topic keys");
}

void TestPointCloudContract() {
  open_lmm::VisualizationSnapshot snapshot;
  snapshot.agent = Agent("agent1");
  snapshot.points = {{1.0F, 2.0F, 3.0F, 0.25F},
                     {-4.0F, 5.0F, 6.0F, 0.75F}};
  builtin_interfaces::msg::Time stamp;
  stamp.sec = 12;
  stamp.nanosec = 34;
  auto converted = open_lmm::ToRosPointCloud(snapshot, "map", stamp, 2);
  Check(converted.IsOk(), "bounded finite cloud converts");
  const auto& cloud = converted.Value();
  Check(cloud.header.frame_id == "map" && cloud.header.stamp.sec == 12,
        "cloud preserves batch frame and stamp");
  Check(cloud.height == 1 && cloud.width == 2 && cloud.point_step == 16 &&
            cloud.row_step == 32 && cloud.fields.size() == 4,
        "cloud layout is stable XYZI float32");
  Check(cloud.fields[0].name == "x" && cloud.fields[0].offset == 0 &&
            cloud.fields[3].name == "intensity" &&
            cloud.fields[3].offset == 12,
        "cloud field names and offsets are stable");
  Check(ReadFloat(cloud, 0) == 1.0F && ReadFloat(cloud, 12) == 0.25F &&
            ReadFloat(cloud, 16) == -4.0F && ReadFloat(cloud, 28) == 0.75F,
        "cloud values preserve snapshot order and intensity");
  Check(!open_lmm::ToRosPointCloud(snapshot, "map", stamp, 1),
        "oversized cloud fails instead of truncating");
  snapshot.points[0].x = std::numeric_limits<float>::quiet_NaN();
  Check(!open_lmm::ToRosPointCloud(snapshot, "map", stamp, 2),
        "non-finite cloud replacement is rejected");
}

void TestPathContract() {
  open_lmm::VisualizationSnapshot snapshot;
  snapshot.agent = Agent("agent1");
  snapshot.poses = {Pose(9, 9.0F, 1.0F, 0.0F),
                    Pose(2, 2.0F, 3.0F, 0.0F)};
  builtin_interfaces::msg::Time stamp;
  auto converted = open_lmm::ToRosPath(snapshot, "world", stamp);
  Check(converted && converted.Value().poses.size() == 2,
        "finite path converts");
  Check(converted.Value().poses[0].pose.position.x == 9.0 &&
            converted.Value().poses[1].pose.position.x == 2.0,
        "path retains snapshot vector order instead of sorting indices");
  Check(converted.Value().poses[0].header.frame_id == "world",
        "path poses share the requested frame");
  snapshot.poses[1].transform.translation().x() =
      std::numeric_limits<float>::infinity();
  Check(!open_lmm::ToRosPath(snapshot, "world", stamp),
        "non-finite path replacement is rejected");
}

void TestLoopMarkerContract() {
  const auto agent1 = Agent("agent1");
  const auto agent2 = Agent("agent2");
  open_lmm::VisualizationSnapshot first;
  first.agent = agent1;
  first.poses = {Pose(0, 0.0F, 0.0F, 0.0F), Pose(1, 1.0F, 0.0F, 0.0F)};
  first.edges.push_back(
      {agent1, 0, agent1, 1, open_lmm::VisualizationEdgeType::kIntraLoop});
  first.edges.push_back(
      {agent1, 1, agent2, 0, open_lmm::VisualizationEdgeType::kInterLoop});
  first.edges.push_back(
      {agent1, 0, agent1, 1, open_lmm::VisualizationEdgeType::kTrajectory});
  open_lmm::VisualizationSnapshot second;
  second.agent = agent2;
  second.poses = {Pose(0, 4.0F, 0.0F, 0.0F)};
  std::map<open_lmm::AgentId, open_lmm::VisualizationSnapshot> snapshots;
  snapshots.emplace(agent1, first);
  snapshots.emplace(agent2, second);
  builtin_interfaces::msg::Time stamp;
  auto converted =
      open_lmm::ToRosLoopMarkers(snapshots, "map", stamp, 0.05);
  Check(converted && converted.Value().markers.size() == 2,
        "loop conversion emits fixed intra/inter markers");
  const auto& intra = converted.Value().markers[0];
  const auto& inter = converted.Value().markers[1];
  Check(intra.ns == "open_lmm/intra_loop" && intra.points.size() == 2 &&
            inter.ns == "open_lmm/inter_loop" && inter.points.size() == 2,
        "loop types are separated and trajectory edges are omitted");
  Check(intra.scale.x == 0.05 && inter.points[1].x == 4.0,
        "marker width and cross-agent endpoint mapping are preserved");
  Check(!open_lmm::ToRosLoopMarkers(snapshots, "", stamp, 0.05) &&
            !open_lmm::ToRosLoopMarkers(snapshots, "map", stamp, 0.0),
        "invalid marker parameters fail closed");

  auto deletes = open_lmm::ToRosLoopDeleteMarkers("map", stamp);
  Check(deletes && deletes.Value().markers.size() == 2,
        "agent removal clears both fixed loop marker owners");
  Check(deletes.Value().markers[0].ns == "open_lmm/intra_loop" &&
            deletes.Value().markers[1].ns == "open_lmm/inter_loop" &&
            deletes.Value().markers[0].action ==
                visualization_msgs::msg::Marker::DELETE &&
            deletes.Value().markers[1].action ==
                visualization_msgs::msg::Marker::DELETE,
        "loop removal uses explicit RViz DELETE actions");
  Check(!open_lmm::ToRosLoopDeleteMarkers("", stamp),
        "loop removal rejects an invalid frame");
}

void TestReplacementAdmission() {
  Check(open_lmm::AcceptRosVisualizationReplacement(7, 7, 12, 12),
        "same generation and revision may replace the retained sample");
  Check(open_lmm::AcceptRosVisualizationReplacement(7, 7, 13, 12),
        "newer revision may replace the retained sample");
  Check(!open_lmm::AcceptRosVisualizationReplacement(6, 7, 13, 12),
        "superseded worker completion cannot publish");
  Check(!open_lmm::AcceptRosVisualizationReplacement(7, 7, 11, 12),
        "older runtime revision cannot replace newer presentation");
}

}  // namespace

int main() {
  try {
    TestAgentKeyEncoding();
    TestPointCloudContract();
    TestPathContract();
    TestLoopMarkerContract();
    TestReplacementAdmission();
    std::cout << "ROS visualization bridge contract tests passed\n";
    return 0;
  } catch (const std::exception& error) {
    std::cerr << "ROS visualization bridge contract failed: " << error.what()
              << '\n';
    return 1;
  }
}
