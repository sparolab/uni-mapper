#pragma once

#include <open_lmm/common/alignment_feedback.hpp>

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <string>

namespace open_lmm {

struct MapAlignmentCoordinatorInput {
  std::string feedback_mode{"interactive"};
  std::chrono::milliseconds feedback_timeout{};
  std::shared_ptr<AlignmentFeedbackBroker> feedback;
  std::shared_ptr<CancellationToken> cancellation;
  std::vector<AlignmentVisualizationPoint> target_points;
  std::vector<AlignmentVisualizationPoint> source_points;
  std::function<std::optional<MapAlignmentProposal>()> kiss_proposer;
  std::function<std::optional<MapAlignmentProposal>()> descriptor_proposer;
  std::function<Result<void>(const MapAlignmentProposal&)> proposal_validator;
  char target_agent = 0;
  char source_agent = 0;
};

// Owns only fallback and approval policy. Algorithm-specific computation and
// loop generation remain outside this class and are invoked lazily through
// proposer callbacks.
class MapAlignmentCoordinator {
 public:
  Result<MapAlignmentProposal> Align(
      const MapAlignmentCoordinatorInput& input) const;

 private:
  Result<AlignmentResponse> Request(
      const MapAlignmentCoordinatorInput& input,
      MapAlignmentProposal proposal) const;
  static MapAlignmentProposal ManualProposal(
      const MapAlignmentCoordinatorInput& input,
      const Eigen::Isometry3d& initial_transform);
  static Result<MapAlignmentProposal> ResolveResponse(
      const MapAlignmentProposal& proposal,
      const AlignmentResponse& response);
  Result<MapAlignmentProposal> ValidateOrRetryManual(
      const MapAlignmentCoordinatorInput& input,
      Result<MapAlignmentProposal> result) const;
};

}  // namespace open_lmm
