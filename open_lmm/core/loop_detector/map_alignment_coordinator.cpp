#include "map_alignment_coordinator.hpp"

namespace open_lmm {

Result<AlignmentResponse> MapAlignmentCoordinator::Request(
    const MapAlignmentCoordinatorInput& input,
    MapAlignmentProposal proposal) const {
  if (!input.feedback || !input.feedback->IsEnabled()) {
    return Result<AlignmentResponse>::Failure(Error::InvalidArgument(
        "interactive map alignment requires an enabled feedback service"));
  }
  AlignmentFeedbackSnapshot snapshot;
  snapshot.proposal = std::move(proposal);
  snapshot.target_points = input.target_points;
  snapshot.source_points = input.source_points;
  return input.feedback->Request(std::move(snapshot), input.cancellation,
                                 input.feedback_timeout);
}

MapAlignmentProposal MapAlignmentCoordinator::ManualProposal(
    const MapAlignmentCoordinatorInput& input,
    const Eigen::Isometry3d& initial_transform) {
  MapAlignmentProposal proposal;
  proposal.target_agent = input.target_agent;
  proposal.source_agent = input.source_agent;
  proposal.method = AlignmentMethod::kManual;
  proposal.target_T_source = initial_transform;
  return proposal;
}

Result<MapAlignmentProposal> MapAlignmentCoordinator::ResolveResponse(
    const MapAlignmentProposal& proposal,
    const AlignmentResponse& response) {
  if (response.decision == AlignmentDecision::kAccept) {
    if (proposal.method == AlignmentMethod::kManual) {
      return Result<MapAlignmentProposal>::Failure(Error::InvalidArgument(
          "manual alignment must be applied with an explicit transform"));
    }
    return Result<MapAlignmentProposal>::Ok(proposal);
  }
  if (response.decision == AlignmentDecision::kManual) {
    if (!response.manual_target_T_source ||
        !IsFiniteRigidTransform(*response.manual_target_T_source)) {
      return Result<MapAlignmentProposal>::Failure(Error::InvalidArgument(
          "manual alignment requires a finite rigid transform"));
    }
    auto manual = proposal;
    manual.method = AlignmentMethod::kManual;
    manual.target_T_source = *response.manual_target_T_source;
    manual.metrics = {};
    return Result<MapAlignmentProposal>::Ok(std::move(manual));
  }
  return Result<MapAlignmentProposal>::Failure(
      Error::Cancelled("map alignment cancelled by user"));
}

Result<MapAlignmentProposal> MapAlignmentCoordinator::ValidateOrRetryManual(
    const MapAlignmentCoordinatorInput& input,
    Result<MapAlignmentProposal> result) const {
  while (result && input.proposal_validator) {
    auto validation = input.proposal_validator(result.Value());
    if (validation) return result;
    if (result.Value().method != AlignmentMethod::kManual) {
      return Result<MapAlignmentProposal>::Failure(validation.GetError());
    }

    auto retry = ManualProposal(input, result.Value().target_T_source);
    auto response = Request(input, retry);
    if (!response) {
      return Result<MapAlignmentProposal>::Failure(response.GetError());
    }
    result = ResolveResponse(retry, response.Value());
  }
  return result;
}

Result<MapAlignmentProposal> MapAlignmentCoordinator::Align(
    const MapAlignmentCoordinatorInput& input) const {
  std::optional<MapAlignmentProposal> kiss;
  if (input.kiss_proposer) kiss = input.kiss_proposer();

  if (input.feedback_mode == "always_manual") {
    auto response = Request(input, ManualProposal(
        input, kiss ? kiss->target_T_source : Eigen::Isometry3d::Identity()));
    if (!response) {
      return Result<MapAlignmentProposal>::Failure(response.GetError());
    }
    return ValidateOrRetryManual(input, ResolveResponse(
        ManualProposal(input, kiss ? kiss->target_T_source
                                   : Eigen::Isometry3d::Identity()),
        response.Value()));
  }

  if (kiss) {
    auto response = Request(input, *kiss);
    if (!response) {
      return Result<MapAlignmentProposal>::Failure(response.GetError());
    }
    if (response.Value().decision != AlignmentDecision::kTryDescriptor) {
      return ValidateOrRetryManual(
          input, ResolveResponse(*kiss, response.Value()));
    }
  }

  std::optional<MapAlignmentProposal> descriptor;
  if (input.descriptor_proposer) descriptor = input.descriptor_proposer();
  MapAlignmentProposal fallback = descriptor.value_or(ManualProposal(
      input, kiss ? kiss->target_T_source : Eigen::Isometry3d::Identity()));
  auto response = Request(input, fallback);
  if (!response) {
    return Result<MapAlignmentProposal>::Failure(response.GetError());
  }
  if (response.Value().decision == AlignmentDecision::kTryDescriptor) {
    return Result<MapAlignmentProposal>::Failure(Error::InvalidArgument(
        "descriptor fallback was already attempted"));
  }
  return ValidateOrRetryManual(
      input, ResolveResponse(fallback, response.Value()));
}

}  // namespace open_lmm
