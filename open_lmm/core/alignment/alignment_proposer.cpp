#include "alignment_proposer.hpp"

#include <exception>
#include <string>
#include <utility>

namespace open_lmm {
namespace {

Result<void> ValidateProposalRequest(
    const AlignmentProposalRequest& request) {
  if (!request.target_agent.IsValid() || !request.source_agent.IsValid()) {
    return Result<void>::Failure(
        Error::InvalidArgument("alignment proposal agent ID is invalid"));
  }
  if (request.target_agent == request.source_agent) {
    return Result<void>::Failure(Error::InvalidArgument(
        "alignment proposal must join different agents"));
  }
  return Result<void>::Ok();
}

Result<void> ValidateProposal(const MapAlignmentProposal& proposal,
                              const AlignmentProposalRequest& request) {
  if (proposal.target_agent != request.target_agent ||
      proposal.source_agent != request.source_agent) {
    return Result<void>::Failure(Error::InvalidArgument(
        "alignment proposer changed the requested agent identities"));
  }
  if (proposal.method != AlignmentMethod::kKissMatcher &&
      proposal.method != AlignmentMethod::kDescriptor) {
    return Result<void>::Failure(Error::InvalidArgument(
        "alignment proposer returned a policy-owned method"));
  }
  return ValidateRigidTransform(proposal.target_T_source,
                                "alignment proposer output");
}

}  // namespace

AlignmentProposer::AlignmentProposer(ProposalAlgorithm algorithm)
    : algorithm_(std::move(algorithm)) {}

Result<std::optional<MapAlignmentProposal>> AlignmentProposer::Propose(
    const AlgorithmExecutionContext& context,
    const AlignmentProposalRequest& request) const {
  AlgorithmExecutionTimer timer(context);
  auto cancelled =
      CheckAlgorithmCancellation(context, "before alignment proposal");
  if (!cancelled) {
    return Result<std::optional<MapAlignmentProposal>>::Failure(
        cancelled.GetError());
  }
  auto valid_request = ValidateProposalRequest(request);
  if (!valid_request) {
    return Result<std::optional<MapAlignmentProposal>>::Failure(
        WithAlgorithmContext(valid_request.GetError(), context));
  }
  if (!algorithm_) {
    return Result<std::optional<MapAlignmentProposal>>::Failure(
        WithAlgorithmContext(
            Error::InvalidArgument("alignment proposal algorithm is empty"),
            context));
  }
  try {
    auto proposed = algorithm_(request);
    if (!proposed) {
      return Result<std::optional<MapAlignmentProposal>>::Failure(
          WithAlgorithmContext(proposed.GetError(), context));
    }
    if (proposed.Value()) {
      auto valid = ValidateProposal(*proposed.Value(), request);
      if (!valid) {
        return Result<std::optional<MapAlignmentProposal>>::Failure(
            WithAlgorithmContext(valid.GetError(), context));
      }
    }
    cancelled =
        CheckAlgorithmCancellation(context, "after alignment proposal");
    if (!cancelled) {
      return Result<std::optional<MapAlignmentProposal>>::Failure(
          cancelled.GetError());
    }
    return proposed;
  } catch (const std::exception& error) {
    return Result<std::optional<MapAlignmentProposal>>::Failure(
        WithAlgorithmContext(
            Error::RegistrationFailed(
                std::string("alignment proposer exception: ") + error.what())
                .MarkFatalRuntime(),
            context));
  } catch (...) {
    return Result<std::optional<MapAlignmentProposal>>::Failure(
        WithAlgorithmContext(
            Error::RegistrationFailed("unknown alignment proposer exception")
                .MarkFatalRuntime(),
            context));
  }
}

}  // namespace open_lmm
