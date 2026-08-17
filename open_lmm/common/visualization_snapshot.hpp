#pragma once

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <vector>

namespace open_lmm {

enum class VisualizationEdgeType : uint8_t {
  kTrajectory,
  kIntraLoop,
  kInterLoop,
};

struct VisualizationPose {
  int index = 0;
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
};

struct VisualizationEdge {
  char from_agent = 0;
  std::size_t from_index = 0;
  char to_agent = 0;
  std::size_t to_index = 0;
  VisualizationEdgeType type = VisualizationEdgeType::kTrajectory;
};

struct VisualizationPoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
};

// Immutable-by-convention transfer object. Algorithm-owned PCL/Eigen containers
// are copied before this value crosses into the GUI thread.
struct VisualizationSnapshot {
  char agent = 0;
  uint64_t revision = 0;
  std::vector<VisualizationPose> poses;
  std::vector<VisualizationEdge> edges;
  std::vector<VisualizationPoint> points;
  Eigen::Vector3f min_bound = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_bound = Eigen::Vector3f::Zero();
  bool has_bounds = false;
  bool map_available = false;
};

}  // namespace open_lmm
