#include "map_alignment_coordinator.hpp"

#include <exception>
#include <string>

namespace open_lmm {
namespace {

Result<std::optional<MapAlignmentProposal>> InvokeProposer(
    const std::function<Result<std::optional<MapAlignmentProposal>>()>& proposer,
    std::string_view name) {
  if (!proposer) {
    return Result<std::optional<MapAlignmentProposal>>::Ok(std::nullopt);
  }
  try {
    return proposer();
  } catch (const std::exception& error) {
    return Result<std::optional<MapAlignmentProposal>>::Failure(
        Error::RegistrationFailed(std::string(name) +
                                  " proposer callback exception: " +
                                  error.what()));
  } catch (...) {
    return Result<std::optional<MapAlignmentProposal>>::Failure(
        Error::RegistrationFailed(std::string(name) +
                                  " proposer callback exception"));
  }
}

}  // namespace

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
  if (input.visualization) snapshot.diagnostics = *input.visualization;
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

MapAlignmentProposal MapAlignmentCoordinator::PendingProposal(
    const MapAlignmentCoordinatorInput& input) {
  MapAlignmentProposal proposal;
  proposal.target_agent = input.target_agent;
  proposal.source_agent = input.source_agent;
  proposal.method = AlignmentMethod::kPending;
  return proposal;
}

Result<MapAlignmentProposal> MapAlignmentCoordinator::ResolveResponse(
    const MapAlignmentProposal& proposal,
    const AlignmentResponse& response) {
  if (response.decision == AlignmentDecision::kAccept) {
    if (proposal.method == AlignmentMethod::kPending) {
      return Result<MapAlignmentProposal>::Failure(Error::InvalidArgument(
          "an alignment method must be attempted before acceptance"));
    }
    if (proposal.method == AlignmentMethod::kManual) {
      return Result<MapAlignmentProposal>::Failure(Error::InvalidArgument(
          "manual alignment must be applied with an explicit transform"));
    }
    return Result<MapAlignmentProposal>::Ok(proposal);
  }
  if (response.decision == AlignmentDecision::kManual) {
    if (!response.manual_target_T_source) {
      return Result<MapAlignmentProposal>::Failure(Error::InvalidArgument(
          "manual alignment requires a finite rigid transform"));
    }
    auto valid = ValidateRigidTransform(*response.manual_target_T_source,
                                        "manual alignment");
    if (!valid) {
      return Result<MapAlignmentProposal>::Failure(valid.GetError());
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
  if (input.intent == InteractiveAlignmentIntent::kManualOnly) {
    auto manual = ManualProposal(input, Eigen::Isometry3d::Identity());
    auto response = Request(input, manual);
    if (!response) {
      return Result<MapAlignmentProposal>::Failure(response.GetError());
    }
    return ValidateOrRetryManual(input,
                                 ResolveResponse(manual, response.Value()));
  }

  MapAlignmentProposal proposal = PendingProposal(input);
  while (true) {
    auto response = Request(input, proposal);
    if (!response) {
      return Result<MapAlignmentProposal>::Failure(response.GetError());
    }
    if (response.Value().decision == AlignmentDecision::kTryKissMatcher) {
      auto kiss = InvokeProposer(input.kiss_proposer, "KISS Matcher");
      if (!kiss) {
        return Result<MapAlignmentProposal>::Failure(kiss.GetError());
      }
      if (!kiss.Value()) {
        return Result<MapAlignmentProposal>::Failure(
            Error::RegistrationFailed(
                "KISS Matcher did not produce an alignment"));
      }
      proposal = *kiss.Value();
      continue;
    }
    if (response.Value().decision == AlignmentDecision::kTryDescriptor) {
      auto descriptor = InvokeProposer(input.descriptor_proposer, "Descriptor");
      if (!descriptor) {
        return Result<MapAlignmentProposal>::Failure(descriptor.GetError());
      }
      if (!descriptor.Value()) {
        return Result<MapAlignmentProposal>::Failure(
            Error::RegistrationFailed(
                "Descriptor did not produce an alignment"));
      }
      proposal = *descriptor.Value();
      continue;
    }
    return ValidateOrRetryManual(
        input, ResolveResponse(proposal, response.Value()));
  }
}

}  // namespace open_lmm
