#include "map_alignment_coordinator.hpp"

#include <algorithm>
#include <exception>
#include <string>

namespace open_lmm {
namespace {

constexpr std::size_t kMaximumAttemptHistory = 16;

Result<AlignmentProposalAttempt> InvokeProposer(
    const std::function<Result<AlignmentProposalAttempt>()>& proposer,
    std::string_view name) {
  if (!proposer) {
    return Result<AlignmentProposalAttempt>::Failure(
        Error::InvalidArgument(std::string(name) +
                               " proposer callback is unavailable"));
  }
  try {
    return proposer();
  } catch (const std::exception& error) {
    return Result<AlignmentProposalAttempt>::Failure(
        Error::RegistrationFailed(std::string(name) +
                                  " proposer callback exception: " +
                                  error.what())
            .MarkFatalRuntime());
  } catch (...) {
    return Result<AlignmentProposalAttempt>::Failure(
        Error::RegistrationFailed(std::string(name) +
                                  " proposer callback exception")
            .MarkFatalRuntime());
  }
}

class ReviewSessionGuard {
 public:
  ReviewSessionGuard(std::shared_ptr<AlignmentFeedbackBroker> broker,
                     AlignmentReviewSessionId session_id)
      : broker_(std::move(broker)), session_id_(session_id) {}
  ~ReviewSessionGuard() {
    if (broker_) (void)broker_->End(session_id_, terminal_error_);
  }
  void PreserveTerminal(const Error& error) { terminal_error_ = error; }

 private:
  std::shared_ptr<AlignmentFeedbackBroker> broker_;
  AlignmentReviewSessionId session_id_;
  std::optional<Error> terminal_error_;
};

void AppendBounded(std::vector<AlignmentAttemptStatus>& history,
                   const AlignmentAttemptStatus& status) {
  history.push_back(status);
  if (history.size() > kMaximumAttemptHistory) {
    history.erase(history.begin(),
                  history.begin() +
                      static_cast<std::ptrdiff_t>(history.size() -
                                                  kMaximumAttemptHistory));
  }
}

}  // namespace

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

Result<MapAlignmentProposal> MapAlignmentCoordinator::Align(
    const MapAlignmentCoordinatorInput& input) const {
  if (!input.feedback || !input.feedback->IsEnabled()) {
    return Result<MapAlignmentProposal>::Failure(Error::InvalidArgument(
        "interactive map alignment requires an enabled feedback service"));
  }

  MapAlignmentProposal proposal =
      input.intent == InteractiveAlignmentIntent::kManualOnly
          ? ManualProposal(input, Eigen::Isometry3d::Identity())
          : PendingProposal(input);
  AlignmentAttemptStatus status;
  status.method = proposal.method;
  std::vector<AlignmentAttemptStatus> history;
  uint64_t next_attempt = 1;
  const auto report_waiting = [&]() noexcept {
    if (!input.progress) return;
    try {
      input.progress({input.source_agent, "loop_detect",
                      AlgorithmProgressPhase::kWaitAlignmentReview, 0,
                      std::nullopt});
    } catch (...) {
      // Progress is observational and must not change review semantics.
    }
  };

  const auto snapshot = [&]() {
    AlignmentFeedbackSnapshot value;
    value.proposal = proposal;
    value.target_points = input.target_points;
    value.source_points = input.source_points;
    if (input.visualization) value.diagnostics = *input.visualization;
    value.attempt_status = status;
    value.attempt_history = history;
    return value;
  };

  auto begun = input.feedback->Begin(snapshot());
  if (!begun) {
    return Result<MapAlignmentProposal>::Failure(begun.GetError());
  }
  const auto session_id = begun.Value();
  ReviewSessionGuard session(input.feedback, session_id);

  const auto terminal_failure = [&](const Error& error) {
    session.PreserveTerminal(error);
    return Result<MapAlignmentProposal>::Failure(error);
  };

  const auto publish = [&]() -> Result<void> {
    return input.feedback->Update(session_id, snapshot());
  };
  const auto wait = [&]() -> Result<AlignmentResponse> {
    report_waiting();
    return input.feedback->WaitDecision(session_id, input.cancellation,
                                        input.feedback_timeout);
  };
  const auto fail_attempt = [&](AlignmentMethod method,
                                AlignmentAttemptFailure reason,
                                std::string message,
                                std::optional<LoopConstraintBuildDiagnostics>
                                    diagnostics = std::nullopt)
      -> Result<void> {
    status = {method, AlignmentAttemptState::kFailedRecoverable, reason,
              std::move(message), next_attempt++, std::move(diagnostics)};
    AppendBounded(history, status);
    return publish();
  };
  const auto run_proposer = [&](AlignmentMethod method,
                                const auto& proposer) -> Result<void> {
    status = {method, AlignmentAttemptState::kRunning, std::nullopt,
              method == AlignmentMethod::kKissMatcher
                  ? "Running KISS Matcher"
                  : "Running descriptor alignment",
              next_attempt, std::nullopt};
    auto updated = publish();
    if (!updated) return updated;
    auto attempted = InvokeProposer(
        proposer, method == AlignmentMethod::kKissMatcher ? "KISS Matcher"
                                                          : "Descriptor");
    if (!attempted) {
      return Result<void>::Failure(attempted.GetError());
    }
    if (!attempted.Value().proposal) {
      const std::string message = attempted.Value().message.empty()
          ? (method == AlignmentMethod::kKissMatcher
                 ? "KISS Matcher did not produce an alignment"
                 : "Descriptor did not produce an alignment")
          : attempted.Value().message;
      return fail_attempt(method, attempted.Value().failure, message);
    }
    proposal = *attempted.Value().proposal;
    status = {method, AlignmentAttemptState::kSucceeded, std::nullopt,
              attempted.Value().message.empty()
                  ? "Alignment proposal is ready for review"
                  : attempted.Value().message,
              next_attempt++, std::nullopt};
    AppendBounded(history, status);
    return publish();
  };

  while (true) {
    auto response = wait();
    if (!response) {
      return terminal_failure(response.GetError());
    }
    if (response.Value().decision == AlignmentDecision::kTryKissMatcher) {
      auto attempted =
          run_proposer(AlignmentMethod::kKissMatcher, input.kiss_proposer);
      if (!attempted) {
        return terminal_failure(attempted.GetError());
      }
      continue;
    }
    if (response.Value().decision == AlignmentDecision::kTryDescriptor) {
      auto attempted =
          run_proposer(AlignmentMethod::kDescriptor,
                       input.descriptor_proposer);
      if (!attempted) {
        return terminal_failure(attempted.GetError());
      }
      continue;
    }
    if (response.Value().decision == AlignmentDecision::kExcludeAgent) {
      if (input.source_agent == input.target_agent ||
          status.state != AlignmentAttemptState::kFailedRecoverable) {
        return terminal_failure(Error::InvalidArgument(
            "only a failed follower alignment may be excluded"));
      }
      return Result<MapAlignmentProposal>::Failure(Error::AgentExcluded(
          input.source_agent.Value() + " was excluded by user"));
    }

    auto resolved = ResolveResponse(proposal, response.Value());
    if (!resolved) {
      return terminal_failure(resolved.GetError());
    }
    if (!input.proposal_validator) return resolved;

    auto validation = input.proposal_validator(resolved.Value());
    if (!validation) {
      return terminal_failure(validation.GetError());
    }
    if (validation.Value().accepted) return resolved;

    proposal = resolved.Value();
    const std::string message = validation.Value().message.empty()
        ? "Alignment proposal did not produce valid loop constraints"
        : validation.Value().message;
    auto failed = fail_attempt(
        proposal.method, validation.Value().failure, message,
        validation.Value().constraint_diagnostics);
    if (!failed) {
      return terminal_failure(failed.GetError());
    }
  }
}

}  // namespace open_lmm
