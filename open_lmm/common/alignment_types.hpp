#pragma once

#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/rigid_transform.hpp>

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <optional>
#include <vector>

namespace open_lmm {

enum class AlignmentMethod : uint8_t {
  kPending,
  kKissMatcher,
  kDescriptor,
  kManual,
};

enum class AlignmentDecision : uint8_t {
  kAccept,
  kTryKissMatcher,
  kTryDescriptor,
  kManual,
  kCancel,
};

enum class AlignmentApproval : uint8_t {
  kAutomatic,
  kUser,
};

struct AlignmentMetrics {
  std::size_t correspondence_count = 0;
  std::size_t rotation_inliers = 0;
  std::size_t final_inliers = 0;
  std::size_t consensus_size = 0;
  std::optional<double> fitness;
  std::optional<double> overlap_ratio;
};

// target_T_source always maps a point expressed in the source agent map frame
// into the target/global map frame.
struct MapAlignmentProposal {
  uint64_t request_id = 0;
  AgentId target_agent;
  AgentId source_agent;
  AlignmentMethod method = AlignmentMethod::kKissMatcher;
  Eigen::Isometry3d target_T_source = Eigen::Isometry3d::Identity();
  AlignmentMetrics metrics;
};

struct AlignmentResponse {
  uint64_t request_id = 0;
  AlignmentDecision decision = AlignmentDecision::kCancel;
  std::optional<Eigen::Isometry3d> manual_target_T_source;
};

struct AlignmentVisualizationPoint {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

struct AlignmentLoopVisualization {
  AlignmentVisualizationPoint target;
  AlignmentVisualizationPoint source;
  bool inlier = false;
};

struct AlignmentVisualizationData {
  std::vector<AlignmentVisualizationPoint> target_trajectory;
  std::vector<AlignmentVisualizationPoint> source_trajectory;
  std::vector<AlignmentLoopVisualization> descriptor_loops;
};

struct AlignmentFeedbackSnapshot {
  MapAlignmentProposal proposal;
  std::vector<AlignmentVisualizationPoint> target_points;
  std::vector<AlignmentVisualizationPoint> source_points;
  AlignmentVisualizationData diagnostics;
};

struct StoredAlignment {
  MapAlignmentProposal proposal;
  AlignmentApproval approval = AlignmentApproval::kAutomatic;
  uint64_t accepted_at_unix_ms = 0;
};

}  // namespace open_lmm
