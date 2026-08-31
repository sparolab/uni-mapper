#pragma once

#include <open_lmm/common/agent_id.hpp>

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace open_lmm {

enum class VisualizationEdgeType : uint8_t {
  kTrajectory,
  kIntraLoop,
  kInterLoop,
};

enum class VisualizationPhase : uint8_t {
  kDataLoad,
  kLoopDetection,
  kOptimization,
  kMapUpdate,
  kSave,
};

enum class VisualizationPoseKind : uint8_t {
  kOdometry,
  kOptimized,
};

enum class VisualizationPointKind : uint8_t {
  kFilteredScanPreview,
  kOptimizationMapPreview,
  kFinalStaticMap,
};

struct VisualizationPose {
  int index = 0;
  Eigen::Isometry3f transform = Eigen::Isometry3f::Identity();
};

struct VisualizationEdge {
  AgentId from_agent;
  std::size_t from_index = 0;
  AgentId to_agent;
  std::size_t to_index = 0;
  VisualizationEdgeType type = VisualizationEdgeType::kTrajectory;
};

struct VisualizationPoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
  float intensity = 0.0F;
};

// Precomputed point payload produced alongside a DataLoad candidate. It is
// immutable once published so a failed candidate can be discarded without
// touching the committed visualization state.
struct VisualizationPointPreview {
  uint32_t voxel_millimeters = 0;
  std::vector<VisualizationPoint> points;
  Eigen::Vector3f min_bound = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_bound = Eigen::Vector3f::Zero();
  bool has_bounds = false;
  std::size_t source_point_count = 0;
};

using VisualizationPointPreviewHandle =
    std::shared_ptr<const VisualizationPointPreview>;

// Immutable-by-convention transfer object. Algorithm-owned PCL/Eigen containers
// are copied before this value crosses into the GUI thread.
struct VisualizationSnapshot {
  AgentId agent;
  uint64_t revision = 0;
  VisualizationPhase phase = VisualizationPhase::kDataLoad;
  VisualizationPoseKind pose_kind = VisualizationPoseKind::kOdometry;
  VisualizationPointKind point_kind =
      VisualizationPointKind::kFilteredScanPreview;
  std::vector<VisualizationPose> poses;
  std::vector<VisualizationEdge> edges;
  std::vector<VisualizationPoint> points;
  Eigen::Vector3f min_bound = Eigen::Vector3f::Zero();
  Eigen::Vector3f max_bound = Eigen::Vector3f::Zero();
  bool has_bounds = false;
  bool points_available = false;
  bool points_complete = false;
  bool map_available = false;
  std::size_t displayed_point_count = 0;
  std::size_t source_point_count = 0;
};

// Large point payloads are opt-in so routine GUI polling does not rebuild or
// copy a map. Resolution defaults are owned by the runtime; callers may still
// request an explicit positive override.
struct VisualizationQuery {
  AgentId agent;
  bool include_points = true;
  // Zero selects the canonical runtime visualization resolution. Positive
  // values are explicit caller overrides.
  float preview_voxel_size_m = 0.0F;
  // Retained for source and ABI compatibility. Point visualization is now
  // bounded by voxel resolution rather than an arbitrary point-count cutoff.
  std::size_t maximum_points = 1'000'000;
};

}  // namespace open_lmm
