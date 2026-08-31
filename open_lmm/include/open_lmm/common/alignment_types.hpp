#pragma once

#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/rigid_transform.hpp>

#include <Eigen/Geometry>

#include <cstddef>
#include <cstdint>
#include <string>
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
  kExcludeAgent,
  kCancel,
};

enum class AlignmentApproval : uint8_t {
  kAutomatic,
  kUser,
};

enum class AlignmentAttemptState : uint8_t {
  kIdle,
  kRunning,
  kSucceeded,
  kFailedRecoverable,
};

enum class AlignmentAttemptFailure : uint8_t {
  kNoCandidate,
  kInsufficientInliers,
  kNoConsistentClique,
  kNoPoseNeighbor,
  kProposalQualityRejected,
};

struct LoopConstraintBuildDiagnostics {
  std::size_t sampled_source_frames = 0;
  std::size_t target_frames = 0;
  std::size_t within_radius = 0;
  double nearest_distance_m = 0.0;
  double threshold_m = 0.0;
  bool search_completed = false;
};

struct AlignmentAttemptStatus {
  AlignmentMethod method = AlignmentMethod::kPending;
  AlignmentAttemptState state = AlignmentAttemptState::kIdle;
  std::optional<AlignmentAttemptFailure> reason;
  std::string message;
  uint64_t attempt = 0;
  std::optional<LoopConstraintBuildDiagnostics> constraint_diagnostics;
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
  uint64_t session_revision = 0;
};

enum class AlignmentReviewState : uint8_t {
  kActive,
  kCancelled,
  kFailed,
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
  AlignmentAttemptStatus attempt_status;
  std::vector<AlignmentAttemptStatus> attempt_history;
  uint64_t session_revision = 0;
  AlignmentReviewState review_state = AlignmentReviewState::kActive;
  std::string terminal_message;
};

struct StoredAlignment {
  MapAlignmentProposal proposal;
  AlignmentApproval approval = AlignmentApproval::kAutomatic;
  uint64_t accepted_at_unix_ms = 0;
};

}  // namespace open_lmm
