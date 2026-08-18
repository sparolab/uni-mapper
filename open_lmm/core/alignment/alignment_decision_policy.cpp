#include "alignment_decision_policy.hpp"

#include <string_view>

namespace open_lmm {
namespace {

Result<void> ValidateProposal(const MapAlignmentProposal& proposal,
                              const AgentId& source_agent,
                              std::optional<AlignmentMethod> expected,
                              std::string_view origin) {
  if (proposal.source_agent != source_agent) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(origin) + " alignment belongs to a different source agent"));
  }
  if (proposal.method == AlignmentMethod::kPending ||
      (expected && proposal.method != *expected)) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(origin) + " alignment method is invalid"));
  }
  if (!IsFiniteRigidTransform(proposal.target_T_source)) {
    return Result<void>::Failure(Error::InvalidArgument(
        std::string(origin) + " alignment transform is not finite and rigid"));
  }
  return Result<void>::Ok();
}

Result<AlignmentPolicyOutcome> Accept(
    MapAlignmentProposal proposal, AlignmentApproval approval,
    uint64_t accepted_at_unix_ms) {
  return Result<AlignmentPolicyOutcome>::Ok(
      {AlignmentPolicyAction::kAccept,
       StoredAlignment{std::move(proposal), approval, accepted_at_unix_ms},
       false});
}

}  // namespace

Result<AlignmentPolicyOutcome> AlignmentDecisionPolicy::Decide(
    const AlignmentPolicyInput& input) const {
  if (input.source_is_anchor) {
    MapAlignmentProposal anchor;
    anchor.target_agent = input.source_agent;
    anchor.source_agent = input.source_agent;
    anchor.method = AlignmentMethod::kKissMatcher;
    anchor.target_T_source = Eigen::Isometry3d::Identity();
    return Accept(std::move(anchor), AlignmentApproval::kAutomatic,
                  input.accepted_at_unix_ms);
  }

  if (input.stored_alignment) {
    auto valid = ValidateProposal(input.stored_alignment->proposal,
                                  input.source_agent, std::nullopt, "stored");
    if (!valid) {
      return Result<AlignmentPolicyOutcome>::Failure(valid.GetError());
    }
    AlignmentPolicyOutcome outcome;
    outcome.action = AlignmentPolicyAction::kAccept;
    outcome.accepted = input.stored_alignment;
    return Result<AlignmentPolicyOutcome>::Ok(std::move(outcome));
  }

  if (input.feedback_mode != AlignmentFeedbackMode::kAutomatic) {
    if (!input.interactive_service_available) {
      return Result<AlignmentPolicyOutcome>::Failure(Error::InvalidArgument(
          "interactive alignment requires an available feedback service"));
    }
    if (input.interactive.state == InteractiveAlignmentState::kPending) {
      return Result<AlignmentPolicyOutcome>::Ok(
          {AlignmentPolicyAction::kRequestInteractive, std::nullopt,
           input.feedback_mode == AlignmentFeedbackMode::kAlwaysManual});
    }
    if (input.interactive.state == InteractiveAlignmentState::kCancelled) {
      return Result<AlignmentPolicyOutcome>::Failure(
          Error::Cancelled("interactive alignment was cancelled"));
    }
    if (!input.interactive.proposal) {
      return Result<AlignmentPolicyOutcome>::Failure(Error::InvalidArgument(
          "interactive acceptance requires an alignment proposal"));
    }
    const auto& proposal = *input.interactive.proposal;
    auto valid = ValidateProposal(proposal, input.source_agent, std::nullopt,
                                  "interactive");
    if (!valid) {
      return Result<AlignmentPolicyOutcome>::Failure(valid.GetError());
    }
    if (input.feedback_mode == AlignmentFeedbackMode::kAlwaysManual &&
        proposal.method != AlignmentMethod::kManual) {
      return Result<AlignmentPolicyOutcome>::Failure(Error::InvalidArgument(
          "always-manual alignment requires a manual proposal"));
    }
    return Accept(proposal, AlignmentApproval::kUser,
                  input.accepted_at_unix_ms);
  }

  if (input.headless_policy == HeadlessAlignmentPolicy::kFail) {
    return Result<AlignmentPolicyOutcome>::Failure(Error::RegistrationFailed(
        "headless alignment is disabled by policy"));
  }

  const auto accept_automatic = [&](const MapAlignmentProposal& proposal,
                                    AlignmentMethod expected) {
    auto valid = ValidateProposal(proposal, input.source_agent, expected,
                                  "automatic");
    if (!valid) {
      return Result<AlignmentPolicyOutcome>::Failure(valid.GetError());
    }
    return Accept(proposal, AlignmentApproval::kAutomatic,
                  input.accepted_at_unix_ms);
  };

  if (input.headless_policy == HeadlessAlignmentPolicy::kKissOnly) {
    if (!input.kiss_proposal) {
      return Result<AlignmentPolicyOutcome>::Failure(
          Error::RegistrationFailed(
              "KISS Matcher did not produce an alignment"));
    }
    return accept_automatic(*input.kiss_proposal,
                            AlignmentMethod::kKissMatcher);
  }

  if (input.headless_policy ==
      HeadlessAlignmentPolicy::kKissThenDescriptor) {
    if (input.kiss_proposal) {
      return accept_automatic(*input.kiss_proposal,
                              AlignmentMethod::kKissMatcher);
    }
    if (input.descriptor_proposal) {
      return accept_automatic(*input.descriptor_proposal,
                              AlignmentMethod::kDescriptor);
    }
    return Result<AlignmentPolicyOutcome>::Failure(
        Error::RegistrationFailed(
            "neither KISS Matcher nor Descriptor produced an alignment"));
  }

  return Result<AlignmentPolicyOutcome>::Failure(
      Error::InvalidArgument("unknown headless alignment policy"));
}

}  // namespace open_lmm
