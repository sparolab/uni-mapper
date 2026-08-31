#include <domain/loop_detection/map_alignment_coordinator.hpp>
#include <domain/loop_detection/descriptor_alignment_proposer.hpp>
#include <domain/loop_detection/map_alignment_refiner.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <vector>

using namespace open_lmm;

namespace {
AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

MapAlignmentProposal Proposal(AlignmentMethod method, double x) {
  MapAlignmentProposal proposal;
  proposal.target_agent = Id("A");
  proposal.source_agent = Id("B");
  proposal.method = method;
  proposal.target_T_source.translation().x() = x;
  return proposal;
}

Result<AlignmentProposalAttempt> Proposed(
    std::optional<MapAlignmentProposal> proposal) {
  return Result<AlignmentProposalAttempt>::Ok(
      {.proposal = std::move(proposal)});
}

Result<AlignmentProposalAttempt> Proposed(
    MapAlignmentProposal proposal) {
  return Proposed(std::optional<MapAlignmentProposal>(std::move(proposal)));
}

AlignmentFeedbackSnapshot WaitForSnapshot(
    const std::shared_ptr<AlignmentFeedbackBroker>& broker) {
  for (int i = 0; i < 200; ++i) {
    if (auto snapshot = broker->Snapshot()) return *snapshot;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cerr << "FAIL: feedback snapshot timeout\n";
  std::exit(1);
}

AlignmentFeedbackSnapshot WaitForAttempt(
    const std::shared_ptr<AlignmentFeedbackBroker>& broker,
    uint64_t after_revision, AlignmentAttemptState state) {
  for (int i = 0; i < 500; ++i) {
    if (auto snapshot = broker->Snapshot();
        snapshot && snapshot->session_revision > after_revision &&
        snapshot->attempt_status.state == state) {
      return *snapshot;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  std::cerr << "FAIL: feedback attempt timeout\n";
  std::exit(1);
}

AlignmentResponse Response(
    const AlignmentFeedbackSnapshot& snapshot, AlignmentDecision decision,
    std::optional<Eigen::Isometry3d> manual = std::nullopt) {
  return {snapshot.proposal.request_id, decision, std::move(manual),
          snapshot.session_revision};
}

MapAlignmentCoordinatorInput Input(
    const std::shared_ptr<AlignmentFeedbackBroker>& broker) {
  MapAlignmentCoordinatorInput input;
  input.feedback = broker;
  input.cancellation = std::make_shared<CancellationToken>();
  input.target_agent = Id("A");
  input.source_agent = Id("B");
  return input;
}

void TestKissAcceptDoesNotComputeDescriptor() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  std::atomic<int> kiss_calls{0};
  std::atomic<int> descriptor_calls{0};
  std::vector<AlgorithmProgress> progress;
  input.progress = [&](const AlgorithmProgress& update) {
    progress.push_back(update);
  };
  input.kiss_proposer = [&] {
    ++kiss_calls;
    return Proposed(Proposal(AlignmentMethod::kKissMatcher, 1));
  };
  input.descriptor_proposer = [&] {
    ++descriptor_calls;
    return Proposed(Proposal(AlignmentMethod::kDescriptor, 2));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  Check(snapshot.proposal.method == AlignmentMethod::kPending &&
            kiss_calls == 0 && descriptor_calls == 0,
        "alignment methods remain lazy before user selection");
  const auto pending_request = snapshot.proposal.request_id;
  const auto pending_revision = snapshot.session_revision;
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryKissMatcher))),
        "request KISS response");
  do {
    snapshot = WaitForSnapshot(broker);
    std::this_thread::yield();
  } while (snapshot.session_revision == pending_revision ||
           snapshot.proposal.method != AlignmentMethod::kKissMatcher);
  Check(snapshot.proposal.request_id == pending_request,
        "KISS proposal remains in the same review session");
  Check(snapshot.proposal.method == AlignmentMethod::kKissMatcher &&
            kiss_calls == 1,
        "KISS runs only after explicit request");
  Check(static_cast<bool>(
            broker->Respond(Response(snapshot, AlignmentDecision::kAccept))),
        "accept KISS response");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kKissMatcher,
        "KISS result accepted");
  Check(descriptor_calls == 0, "Descriptor remains lazy after KISS accept");
  const auto wait_updates = std::count_if(
      progress.begin(), progress.end(), [](const AlgorithmProgress& update) {
        return update.phase ==
               AlgorithmProgressPhase::kWaitAlignmentReview;
      });
  Check(wait_updates >= 2 &&
            std::all_of(progress.begin(), progress.end(),
                        [](const AlgorithmProgress& update) {
                          return update.phase !=
                                     AlgorithmProgressPhase::kWaitAlignmentReview ||
                                 !update.total;
                        }),
        "initial and post-proposal review waits must remain indeterminate");
}

void TestRecoverableFailureAllowsExplicitFollowerExclusion() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] {
    return Result<AlignmentProposalAttempt>::Ok(
        {.proposal = std::nullopt,
         .failure = AlignmentAttemptFailure::kInsufficientInliers,
         .message = "not enough inliers"});
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  Check(!broker->Respond(Response(snapshot,
                                 AlignmentDecision::kExcludeAgent)),
        "agent exclusion is rejected before a recoverable failure");
  Check(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryKissMatcher))
            .IsOk(),
        "failed follower fixture starts KISS proposal");
  snapshot = WaitForAttempt(broker, snapshot.session_revision,
                            AlignmentAttemptState::kFailedRecoverable);
  Check(broker->Respond(
            Response(snapshot, AlignmentDecision::kExcludeAgent))
            .IsOk(),
        "recoverable follower failure accepts explicit exclusion");
  worker.join();
  Check(!result &&
            result.GetError().code == Error::Code::kAgentExcluded &&
            !broker->Snapshot(),
        "explicit exclusion ends review without turning the stage into cancellation");
}

void TestDescriptorFallback() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] { return Proposed(Proposal(AlignmentMethod::kKissMatcher, 1)); };
  input.descriptor_proposer = [] {
    return Proposed(Proposal(AlignmentMethod::kDescriptor, 2));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  Check(snapshot.proposal.method == AlignmentMethod::kPending,
        "Descriptor flow starts pending");
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryDescriptor))),
        "request Descriptor response");
  do {
    snapshot = WaitForSnapshot(broker);
    std::this_thread::yield();
  } while (snapshot.proposal.method != AlignmentMethod::kDescriptor);
  Check(static_cast<bool>(
            broker->Respond(Response(snapshot, AlignmentDecision::kAccept))),
        "accept Descriptor response");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kDescriptor,
        "Descriptor fallback accepted");
  Check(result.Value().target_T_source.translation().x() == 2,
        "Descriptor transform preserved");
}

void TestKissFailureRetriesDescriptorInSameSession() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] {
    return Result<AlignmentProposalAttempt>::Ok(
        {.proposal = std::nullopt,
         .failure = AlignmentAttemptFailure::kInsufficientInliers,
         .message = "only 2 final inliers; minimum is 5"});
  };
  input.descriptor_proposer = [] {
    return Proposed(Proposal(AlignmentMethod::kDescriptor, 8));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });

  auto snapshot = WaitForSnapshot(broker);
  const auto session_id = snapshot.proposal.request_id;
  const auto initial_revision = snapshot.session_revision;
  Check(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryKissMatcher)).IsOk(),
        "request KISS failure fixture");
  snapshot = WaitForAttempt(broker, initial_revision,
                            AlignmentAttemptState::kFailedRecoverable);
  Check(snapshot.proposal.request_id == session_id &&
            snapshot.attempt_status.reason ==
                AlignmentAttemptFailure::kInsufficientInliers &&
            snapshot.attempt_history.size() == 1,
        "KISS failure remains visible in one review session");

  const auto failed_revision = snapshot.session_revision;
  Check(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryDescriptor)).IsOk(),
        "retry Descriptor after KISS failure");
  snapshot = WaitForAttempt(broker, failed_revision,
                            AlignmentAttemptState::kSucceeded);
  Check(snapshot.proposal.request_id == session_id &&
            snapshot.proposal.method == AlignmentMethod::kDescriptor &&
            snapshot.attempt_history.size() == 2,
        "Descriptor succeeds without replacing the review session");
  Check(broker->Respond(Response(snapshot, AlignmentDecision::kAccept)).IsOk(),
        "accept Descriptor after recoverable KISS failure");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kDescriptor,
        "recoverable retry returns Descriptor result");
}

void TestDescriptorFailureRetriesManualInSameSession() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.descriptor_proposer = [] {
    return Result<AlignmentProposalAttempt>::Ok(
        {.proposal = std::nullopt,
         .failure = AlignmentAttemptFailure::kNoConsistentClique,
         .message = "descriptor candidates had no consistent clique"});
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });

  auto snapshot = WaitForSnapshot(broker);
  const auto revision = snapshot.session_revision;
  Check(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryDescriptor)).IsOk(),
        "request Descriptor failure fixture");
  snapshot = WaitForAttempt(broker, revision,
                            AlignmentAttemptState::kFailedRecoverable);
  Check(snapshot.attempt_status.reason ==
            AlignmentAttemptFailure::kNoConsistentClique,
        "Descriptor failure reason remains typed");

  Eigen::Isometry3d manual = Eigen::Isometry3d::Identity();
  manual.translation().y() = 6;
  Check(broker->Respond(Response(snapshot, AlignmentDecision::kManual, manual))
            .IsOk(),
        "apply Manual after Descriptor failure");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kManual &&
            result.Value().target_T_source.translation().y() == 6,
        "Manual retry completes the existing review session");
}

void TestAttemptHistoryIsBounded() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] {
    return Result<AlignmentProposalAttempt>::Ok(
        {.proposal = std::nullopt,
         .failure = AlignmentAttemptFailure::kNoCandidate,
         .message = "no KISS candidate"});
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::InvalidArgument("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  for (uint64_t attempt = 1; attempt <= 20; ++attempt) {
    const auto revision = snapshot.session_revision;
    Check(broker->Respond(
              Response(snapshot, AlignmentDecision::kTryKissMatcher)).IsOk(),
          "submit bounded history attempt");
    snapshot = WaitForAttempt(broker, revision,
                              AlignmentAttemptState::kFailedRecoverable);
  }
  Check(snapshot.attempt_history.size() == 16 &&
            snapshot.attempt_history.front().attempt == 5 &&
            snapshot.attempt_history.back().attempt == 20,
        "attempt history keeps the most recent sixteen entries");
  Check(broker->Respond(Response(snapshot, AlignmentDecision::kCancel)).IsOk(),
        "cancel bounded history fixture");
  worker.join();
  Check(!result && result.GetError().code == Error::Code::kCancelled,
        "bounded history fixture terminates by cancellation");
  const auto terminal = broker->Snapshot();
  Check(terminal &&
            terminal->review_state == AlignmentReviewState::kCancelled &&
            terminal->attempt_history.size() == 16 &&
            terminal->terminal_message.find("cancelled") != std::string::npos,
        "cancellation preserves bounded history in a terminal review");
  Check(!broker->Respond(Response(*terminal, AlignmentDecision::kAccept)),
        "terminal cancelled review rejects responses");
}

void TestFullFallbackToManual() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] { return Proposed(Proposal(AlignmentMethod::kKissMatcher, 1)); };
  input.descriptor_proposer = [] {
    return Proposed(Proposal(AlignmentMethod::kDescriptor, 2));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  const auto pending_request = snapshot.proposal.request_id;
  const auto pending_revision = snapshot.session_revision;
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryKissMatcher))),
        "full fallback requests KISS");
  do {
    snapshot = WaitForSnapshot(broker);
    std::this_thread::yield();
  } while (snapshot.session_revision == pending_revision ||
           snapshot.proposal.method != AlignmentMethod::kKissMatcher);
  Check(snapshot.proposal.request_id == pending_request,
        "fallback keeps one review session");
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryDescriptor))),
        "full fallback rejects KISS");
  const auto kiss_revision = snapshot.session_revision;
  for (int i = 0; i < 200; ++i) {
    snapshot = WaitForSnapshot(broker);
    if (snapshot.session_revision != kiss_revision &&
        snapshot.proposal.method == AlignmentMethod::kDescriptor) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  Check(snapshot.proposal.method == AlignmentMethod::kDescriptor,
        "full fallback presents Descriptor proposal");
  Eigen::Isometry3d manual = Eigen::Isometry3d::Identity();
  manual.translation().z() = 7;
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kManual, manual))),
        "full fallback applies manual transform");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kManual,
        "full fallback returns Manual proposal");
  Check(result.Value().target_T_source.translation().z() == 7,
        "full fallback preserves Manual transform");
}

void TestAlwaysManualAndRigidTransformValidation() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.intent = InteractiveAlignmentIntent::kManualOnly;
  std::atomic<int> kiss_calls{0};
  input.kiss_proposer = [&] {
    ++kiss_calls;
    return Proposed(Proposal(AlignmentMethod::kKissMatcher, 3));
  };
  input.descriptor_proposer = []() -> Result<AlignmentProposalAttempt> {
    std::abort();
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  const auto snapshot = WaitForSnapshot(broker);
  Check(snapshot.proposal.method == AlignmentMethod::kManual,
        "always_manual opens manual proposal");
  Check(snapshot.proposal.target_T_source.isApprox(Eigen::Isometry3d::Identity()) &&
            kiss_calls == 0,
        "always manual starts at identity without running KISS");
  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.linear()(0, 0) = 2;
  Check(!static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kManual, invalid))),
        "non-rigid manual transform rejected");
  Eigen::Isometry3d manual = Eigen::Isometry3d::Identity();
  manual.translation().y() = 4;
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kManual, manual))),
        "valid manual transform accepted");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kManual,
        "manual result returned");
  Check(result.Value().target_T_source.translation().y() == 4,
        "manual transform preserved");
}

void TestTimeout() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.feedback_timeout = std::chrono::milliseconds(5);
  input.kiss_proposer = [] { return Proposed(Proposal(AlignmentMethod::kKissMatcher, 1)); };
  const auto result = MapAlignmentCoordinator().Align(input);
  Check(!result && result.GetError().code == Error::Code::kInvalidArgument,
        "feedback timeout reported");
  const auto terminal = broker->Snapshot();
  Check(terminal && terminal->review_state == AlignmentReviewState::kFailed &&
            terminal->terminal_message.find("timed out") != std::string::npos,
        "timed-out request remains visible as a terminal review");
  Check(!broker->Respond(Response(*terminal, AlignmentDecision::kAccept)),
        "terminal timeout review rejects responses");

  AlignmentFeedbackSnapshot replacement;
  replacement.proposal = Proposal(AlignmentMethod::kPending, 0);
  const auto begun = broker->Begin(replacement);
  Check(begun && broker->Snapshot() &&
            broker->Snapshot()->review_state == AlignmentReviewState::kActive &&
            broker->Snapshot()->proposal.request_id !=
                terminal->proposal.request_id,
        "new review replaces the previous terminal review");
  Check(broker->End(begun.Value()).IsOk(), "end replacement review");
}

void TestProposerFailurePreservesStructuredError() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] {
    return Result<AlignmentProposalAttempt>::Failure(
        Error::PluginLoadFailed("KISS plugin failed")
            .WithExecution("alignment", "kiss", Id("B")));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::InvalidArgument("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  const auto snapshot = WaitForSnapshot(broker);
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryKissMatcher))),
        "request failing KISS proposal");
  worker.join();
  Check(!result && result.GetError().code == Error::Code::kPluginLoadFailed,
        "proposer error code is preserved");
  Check(result.GetError().context.stage == "alignment" &&
            result.GetError().context.node == "kiss" &&
            result.GetError().context.agent == Id("B"),
        "proposer structured context is preserved");
  const auto terminal = broker->Snapshot();
  Check(terminal && terminal->review_state == AlignmentReviewState::kFailed &&
            terminal->attempt_status.state ==
                AlignmentAttemptState::kRunning &&
            terminal->terminal_message.find("KISS plugin failed") !=
                std::string::npos,
        "structured proposer failure preserves the last review as terminal");
  Check(!broker->Respond(Response(*terminal, AlignmentDecision::kAccept)),
        "terminal proposer failure rejects responses");
}

void TestRunningAttemptRejectsQueuedResponse() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  std::atomic<bool> proposer_entered{false};
  std::atomic<bool> release_proposer{false};
  input.kiss_proposer = [&] {
    proposer_entered = true;
    while (!release_proposer.load()) std::this_thread::yield();
    return Proposed(Proposal(AlignmentMethod::kKissMatcher, 3));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::InvalidArgument("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  const auto initial_revision = snapshot.session_revision;
  Check(broker->Respond(
            Response(snapshot, AlignmentDecision::kTryKissMatcher)).IsOk(),
        "start blocking KISS attempt");
  snapshot = WaitForAttempt(broker, initial_revision,
                            AlignmentAttemptState::kRunning);
  Check(proposer_entered.load(), "blocking proposer entered");
  Check(!broker->Respond(Response(snapshot, AlignmentDecision::kAccept)),
        "running attempt rejects a queued acceptance");
  release_proposer = true;
  snapshot = WaitForAttempt(broker, snapshot.session_revision,
                            AlignmentAttemptState::kSucceeded);
  Check(broker->Respond(Response(snapshot, AlignmentDecision::kAccept)).IsOk(),
        "completed attempt accepts its current revision");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kKissMatcher,
        "running response rejection does not disturb the attempt");
}

void TestManualValidationRetries() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] { return Proposed(Proposal(AlignmentMethod::kKissMatcher, 1)); };
  input.descriptor_proposer = [] { return Proposed(std::nullopt); };
  std::atomic<int> validation_calls{0};
  input.proposal_validator = [&](const MapAlignmentProposal&) {
    if (++validation_calls == 1) {
      LoopConstraintBuildDiagnostics diagnostics;
      diagnostics.sampled_source_frames = 4;
      diagnostics.target_frames = 10;
      diagnostics.within_radius = 0;
      diagnostics.nearest_distance_m = 3.4;
      diagnostics.threshold_m = 2.0;
      diagnostics.search_completed = true;
      return Result<AlignmentProposalValidation>::Ok(
          {.accepted = false,
           .failure = AlignmentAttemptFailure::kNoPoseNeighbor,
           .message = "no valid loops",
           .constraint_diagnostics = diagnostics});
    }
    return Result<AlignmentProposalValidation>::Ok({.accepted = true});
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  const auto first_request = snapshot.proposal.request_id;
  const auto first_revision = snapshot.session_revision;
  Eigen::Isometry3d first = Eigen::Isometry3d::Identity();
  first.translation().x() = 10;
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kManual, first))),
        "first manual transform accepted by mailbox");
  for (int i = 0; i < 200; ++i) {
    snapshot = WaitForSnapshot(broker);
    if (snapshot.session_revision != first_revision &&
        snapshot.attempt_status.state ==
            AlignmentAttemptState::kFailedRecoverable) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  Check(snapshot.proposal.request_id == first_request,
        "failed manual validation stays in the review session");
  Check(snapshot.proposal.method == AlignmentMethod::kManual,
        "retry remains in manual mode");
  Check(snapshot.proposal.target_T_source.translation().x() == 10,
        "retry starts from rejected manual transform");
  Check(snapshot.attempt_status.reason ==
            AlignmentAttemptFailure::kNoPoseNeighbor &&
            snapshot.attempt_status.constraint_diagnostics &&
            snapshot.attempt_status.constraint_diagnostics->nearest_distance_m ==
                3.4,
        "manual retry publishes typed nearest-neighbor diagnostics");
  Eigen::Isometry3d second = Eigen::Isometry3d::Identity();
  second.translation().x() = 20;
  Check(static_cast<bool>(broker->Respond(
            Response(snapshot, AlignmentDecision::kManual, second))),
        "second manual transform accepted");
  worker.join();
  Check(result && result.Value().target_T_source.translation().x() == 20,
        "validated retry result returned");
  Check(validation_calls == 2, "manual validator called for both attempts");
}

void TestDescriptorConsensusRejectsOutlier() {
  PoseVec odometry(4, Eigen::Isometry3d::Identity());
  AgentOptimizedData optimized_agent;
  optimized_agent.agent_id = Id("A");
  LoopPairVec loops;
  const double translations[] = {0.8, 1.0, 1.2, 100.0};
  for (int i = 0; i < 4; ++i) {
    optimized_agent.optimized_poses.emplace_back(
        i, Eigen::Isometry3d::Identity());
    LoopPair loop;
    loop.to = {Id("A"), static_cast<std::size_t>(i)};
    loop.from = {Id("B"), static_cast<std::size_t>(i)};
    loop.init_rel_pose = Eigen::Isometry3d::Identity();
    loop.init_rel_pose.translation().x() = translations[i];
    loops.push_back(loop);
  }
  AgentOptimizedDataMap optimized;
  optimized[Id("A")] =
      std::make_shared<const AgentOptimizedData>(std::move(optimized_agent));
  DescriptorAlignmentDiagnostics diagnostics;
  const auto proposal = DescriptorAlignmentProposer(10.0, 20.0 * M_PI / 180.0).Propose(
      Id("A"), Id("B"), odometry, optimized, loops, &diagnostics);
  Check(proposal.has_value(), "Descriptor consensus proposal exists");
  Check(proposal->metrics.consensus_size == 3,
        "Descriptor outlier excluded from consensus");
  Check(std::abs(proposal->target_T_source.translation().x() - 1.0) < 1e-9,
        "Descriptor consensus transform averages inliers");
  Check(diagnostics.inlier_loop_indices == std::vector<std::size_t>({0, 1, 2}),
        "Descriptor diagnostics preserve original inlier loop indices");

  DescriptorAlignmentOptions exact_options;
  exact_options.pcm_translation_threshold = 10.0;
  exact_options.pcm_rotation_threshold_rad = 20.0 * M_PI / 180.0;
  exact_options.solver = "exact";
  exact_options.threads = 2;
  exact_options.max_candidates = 0;
  const auto exact_proposal = DescriptorAlignmentProposer(exact_options).Propose(
      Id("A"), Id("B"), odometry, optimized, loops);
  Check(exact_proposal.has_value(), "Exact Descriptor solver remains available");
  Check(exact_proposal->metrics.consensus_size == 3,
        "Exact Descriptor solver rejects the same outlier");
  Check(exact_proposal->target_T_source.matrix().isApprox(
            proposal->target_T_source.matrix(), 1e-12),
        "Heuristic and exact Descriptor poses agree on the reference set");
}

void TestMapRefinementAndQualityMetrics() {
  std::vector<Eigen::Vector3f> target;
  std::vector<Eigen::Vector3f> source;
  const Eigen::Vector3f translation(1.0F, 2.0F, 0.5F);
  for (int i = 0; i < 40; ++i) {
    const Eigen::Vector3f point(
        i * 0.7F, std::sin(i * 0.37F) * 3.0F,
        std::cos(i * 0.23F) * 1.5F);
    target.push_back(point);
    source.push_back(point - translation);
  }
  Eigen::Isometry3d initial = Eigen::Isometry3d::Identity();
  initial.translation() = Eigen::Vector3d(0.8, 1.8, 0.4);
  const auto refined = RefineMapAlignment(target, source, initial, true);
  Check(refined.refined, "map refinement ran");
  Check((refined.target_T_source.translation() - translation.cast<double>())
            .norm() < 1.0e-3,
        "map refinement recovers translation");
  Check(refined.correspondence_count == source.size(),
        "quality metric counts map correspondences");
  Check(refined.overlap_ratio && *refined.overlap_ratio > 0.99,
        "quality metric reports overlap");
  Check(refined.fitness && *refined.fitness < 1.0e-6,
        "quality metric reports nearest-neighbor fitness");
}

}  // namespace

int main() {
  TestKissAcceptDoesNotComputeDescriptor();
  TestRecoverableFailureAllowsExplicitFollowerExclusion();
  TestDescriptorFallback();
  TestKissFailureRetriesDescriptorInSameSession();
  TestDescriptorFailureRetriesManualInSameSession();
  TestAttemptHistoryIsBounded();
  TestFullFallbackToManual();
  TestAlwaysManualAndRigidTransformValidation();
  TestTimeout();
  TestProposerFailurePreservesStructuredError();
  TestRunningAttemptRejectsQueuedResponse();
  TestManualValidationRetries();
  TestDescriptorConsensusRejectsOutlier();
  TestMapRefinementAndQualityMetrics();
  std::cout << "map alignment coordinator tests passed\n";
  return 0;
}
