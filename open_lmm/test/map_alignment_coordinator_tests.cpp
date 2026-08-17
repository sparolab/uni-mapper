#include <open_lmm/core/loop_detector/map_alignment_coordinator.hpp>
#include <open_lmm/core/loop_detector/descriptor_alignment_proposer.hpp>
#include <open_lmm/core/loop_detector/map_alignment_refiner.hpp>

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>

using namespace open_lmm;

namespace {
void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

MapAlignmentProposal Proposal(AlignmentMethod method, double x) {
  MapAlignmentProposal proposal;
  proposal.target_agent = 'A';
  proposal.source_agent = 'B';
  proposal.method = method;
  proposal.target_T_source.translation().x() = x;
  return proposal;
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

MapAlignmentCoordinatorInput Input(
    const std::shared_ptr<AlignmentFeedbackBroker>& broker) {
  MapAlignmentCoordinatorInput input;
  input.feedback = broker;
  input.cancellation = std::make_shared<CancellationToken>();
  input.target_agent = 'A';
  input.source_agent = 'B';
  return input;
}

void TestKissAcceptDoesNotComputeDescriptor() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  std::atomic<int> descriptor_calls{0};
  input.kiss_proposer = [] { return Proposal(AlignmentMethod::kKissMatcher, 1); };
  input.descriptor_proposer = [&] {
    ++descriptor_calls;
    return std::optional(Proposal(AlignmentMethod::kDescriptor, 2));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  const auto snapshot = WaitForSnapshot(broker);
  Check(static_cast<bool>(broker->Respond({snapshot.proposal.request_id,
                         AlignmentDecision::kAccept, std::nullopt})),
        "accept KISS response");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kKissMatcher,
        "KISS result accepted");
  Check(descriptor_calls == 0, "Descriptor remains lazy after KISS accept");
}

void TestDescriptorFallback() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] { return Proposal(AlignmentMethod::kKissMatcher, 1); };
  input.descriptor_proposer = [] {
    return std::optional(Proposal(AlignmentMethod::kDescriptor, 2));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  Check(static_cast<bool>(broker->Respond({snapshot.proposal.request_id,
                         AlignmentDecision::kTryDescriptor, std::nullopt})),
        "request Descriptor response");
  do {
    snapshot = WaitForSnapshot(broker);
    std::this_thread::yield();
  } while (snapshot.proposal.method != AlignmentMethod::kDescriptor);
  Check(static_cast<bool>(broker->Respond({snapshot.proposal.request_id,
                         AlignmentDecision::kAccept, std::nullopt})),
        "accept Descriptor response");
  worker.join();
  Check(result && result.Value().method == AlignmentMethod::kDescriptor,
        "Descriptor fallback accepted");
  Check(result.Value().target_T_source.translation().x() == 2,
        "Descriptor transform preserved");
}

void TestFullFallbackToManual() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] { return Proposal(AlignmentMethod::kKissMatcher, 1); };
  input.descriptor_proposer = [] {
    return std::optional(Proposal(AlignmentMethod::kDescriptor, 2));
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  Check(static_cast<bool>(broker->Respond(
            {snapshot.proposal.request_id,
             AlignmentDecision::kTryDescriptor, std::nullopt})),
        "full fallback rejects KISS");
  const auto kiss_request = snapshot.proposal.request_id;
  for (int i = 0; i < 200; ++i) {
    snapshot = WaitForSnapshot(broker);
    if (snapshot.proposal.request_id != kiss_request) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  Check(snapshot.proposal.method == AlignmentMethod::kDescriptor,
        "full fallback presents Descriptor proposal");
  Eigen::Isometry3d manual = Eigen::Isometry3d::Identity();
  manual.translation().z() = 7;
  Check(static_cast<bool>(broker->Respond(
            {snapshot.proposal.request_id, AlignmentDecision::kManual,
             manual})),
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
  input.feedback_mode = "always_manual";
  input.kiss_proposer = [] { return Proposal(AlignmentMethod::kKissMatcher, 3); };
  input.descriptor_proposer = []() -> std::optional<MapAlignmentProposal> {
    std::abort();
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  const auto snapshot = WaitForSnapshot(broker);
  Check(snapshot.proposal.method == AlignmentMethod::kManual,
        "always_manual opens manual proposal");
  Check(snapshot.proposal.target_T_source.translation().x() == 3,
        "manual starts from KISS transform");
  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.linear()(0, 0) = 2;
  Check(!static_cast<bool>(broker->Respond({snapshot.proposal.request_id,
                          AlignmentDecision::kManual, invalid})),
        "non-rigid manual transform rejected");
  Eigen::Isometry3d manual = Eigen::Isometry3d::Identity();
  manual.translation().y() = 4;
  Check(static_cast<bool>(broker->Respond({snapshot.proposal.request_id,
                         AlignmentDecision::kManual, manual})),
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
  input.kiss_proposer = [] { return Proposal(AlignmentMethod::kKissMatcher, 1); };
  const auto result = MapAlignmentCoordinator().Align(input);
  Check(!result && result.GetError().code == Error::Code::kInvalidArgument,
        "feedback timeout reported");
  Check(!broker->Snapshot(), "timed-out request cleared");
}

void TestManualValidationRetries() {
  auto broker = std::make_shared<AlignmentFeedbackBroker>();
  broker->SetEnabled(true);
  auto input = Input(broker);
  input.kiss_proposer = [] { return Proposal(AlignmentMethod::kKissMatcher, 1); };
  input.descriptor_proposer = [] { return std::optional<MapAlignmentProposal>{}; };
  std::atomic<int> validation_calls{0};
  input.proposal_validator = [&](const MapAlignmentProposal&) {
    if (++validation_calls == 1) {
      return Result<void>::Failure(
          Error::RegistrationFailed("no valid loops"));
    }
    return Result<void>::Ok();
  };
  Result<MapAlignmentProposal> result =
      Result<MapAlignmentProposal>::Failure(Error::Cancelled("not run"));
  std::thread worker([&] { result = MapAlignmentCoordinator().Align(input); });
  auto snapshot = WaitForSnapshot(broker);
  const auto first_request = snapshot.proposal.request_id;
  Eigen::Isometry3d first = Eigen::Isometry3d::Identity();
  first.translation().x() = 10;
  Check(static_cast<bool>(broker->Respond(
            {first_request, AlignmentDecision::kManual, first})),
        "first manual transform accepted by mailbox");
  for (int i = 0; i < 200; ++i) {
    snapshot = WaitForSnapshot(broker);
    if (snapshot.proposal.request_id != first_request) break;
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  Check(snapshot.proposal.request_id != first_request,
        "failed manual validation creates a new request");
  Check(snapshot.proposal.method == AlignmentMethod::kManual,
        "retry remains in manual mode");
  Check(snapshot.proposal.target_T_source.translation().x() == 10,
        "retry starts from rejected manual transform");
  Eigen::Isometry3d second = Eigen::Isometry3d::Identity();
  second.translation().x() = 20;
  Check(static_cast<bool>(broker->Respond(
            {snapshot.proposal.request_id, AlignmentDecision::kManual, second})),
        "second manual transform accepted");
  worker.join();
  Check(result && result.Value().target_T_source.translation().x() == 20,
        "validated retry result returned");
  Check(validation_calls == 2, "manual validator called for both attempts");
}

void TestDescriptorConsensusRejectsOutlier() {
  PoseVec odometry(4, Eigen::Isometry3d::Identity());
  std::map<char, AgentOptimizedData> optimized;
  optimized['A'].agent_id = 'A';
  LoopPairVec loops;
  const double translations[] = {0.8, 1.0, 1.2, 100.0};
  for (int i = 0; i < 4; ++i) {
    optimized['A'].optimized_poses.emplace_back(
        i, Eigen::Isometry3d::Identity());
    LoopPair loop;
    loop.to = {'A', static_cast<std::size_t>(i)};
    loop.from = {'B', static_cast<std::size_t>(i)};
    loop.init_rel_pose = Eigen::Isometry3d::Identity();
    loop.init_rel_pose.translation().x() = translations[i];
    loops.push_back(loop);
  }
  const auto proposal = DescriptorAlignmentProposer(10.0, 20.0 * M_PI / 180.0).Propose(
      'A', 'B', odometry, optimized, loops, {}, {});
  Check(proposal.has_value(), "Descriptor consensus proposal exists");
  Check(proposal->metrics.consensus_size == 3,
        "Descriptor outlier excluded from consensus");
  Check(std::abs(proposal->target_T_source.translation().x() - 1.0) < 1e-9,
        "Descriptor consensus transform averages inliers");
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
  TestDescriptorFallback();
  TestFullFallbackToManual();
  TestAlwaysManualAndRigidTransformValidation();
  TestTimeout();
  TestManualValidationRetries();
  TestDescriptorConsensusRejectsOutlier();
  TestMapRefinementAndQualityMetrics();
  std::cout << "map alignment coordinator tests passed\n";
  return 0;
}
