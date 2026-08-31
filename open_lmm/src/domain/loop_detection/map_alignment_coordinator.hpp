#pragma once

#include <open_lmm/common/alignment_feedback.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace open_lmm {

enum class InteractiveAlignmentIntent : uint8_t {
  kChooseMethod,
  kManualOnly,
};

struct AlignmentProposalAttempt {
  std::optional<MapAlignmentProposal> proposal;
  AlignmentAttemptFailure failure = AlignmentAttemptFailure::kNoCandidate;
  std::string message;
};

struct AlignmentProposalValidation {
  bool accepted = true;
  AlignmentAttemptFailure failure =
      AlignmentAttemptFailure::kProposalQualityRejected;
  std::string message;
  std::optional<LoopConstraintBuildDiagnostics> constraint_diagnostics;
};

struct MapAlignmentCoordinatorInput {
  InteractiveAlignmentIntent intent =
      InteractiveAlignmentIntent::kChooseMethod;
  std::chrono::milliseconds feedback_timeout{};
  std::shared_ptr<AlignmentFeedbackBroker> feedback;
  std::shared_ptr<CancellationToken> cancellation;
  std::vector<AlignmentVisualizationPoint> target_points;
  std::vector<AlignmentVisualizationPoint> source_points;
  std::shared_ptr<AlignmentVisualizationData> visualization;
  std::function<Result<AlignmentProposalAttempt>()> kiss_proposer;
  std::function<Result<AlignmentProposalAttempt>()> descriptor_proposer;
  std::function<Result<AlignmentProposalValidation>(
      const MapAlignmentProposal&)> proposal_validator;
  AgentId target_agent;
  AgentId source_agent;
};

// Interactive transport driver only. Algorithm-specific computation is
// invoked lazily through proposer callbacks; final approval metadata and
// headless/stored selection belong to AlignmentDecisionPolicy.
class MapAlignmentCoordinator {
 public:
  Result<MapAlignmentProposal> Align(
      const MapAlignmentCoordinatorInput& input) const;

 private:
  static MapAlignmentProposal ManualProposal(
      const MapAlignmentCoordinatorInput& input,
      const Eigen::Isometry3d& initial_transform);
  static MapAlignmentProposal PendingProposal(
      const MapAlignmentCoordinatorInput& input);
  static Result<MapAlignmentProposal> ResolveResponse(
      const MapAlignmentProposal& proposal,
      const AlignmentResponse& response);
};

}  // namespace open_lmm
