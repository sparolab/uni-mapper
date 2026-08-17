#include <open_lmm/server/pipeline_controller.hpp>
#include <open_lmm/core/loop_detector/map_alignment_coordinator.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>

using namespace open_lmm;

namespace {
void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

class FakeRunner final : public StageRunner {
 public:
  void SetCancellationToken(std::shared_ptr<CancellationToken> token) override {
    cancellation = std::move(token);
  }
  void SetAlignmentFeedbackBroker(
      std::shared_ptr<AlignmentFeedbackBroker> value) override {
    alignment_feedback = std::move(value);
  }
  Result<void> RunNode(NodeId, std::optional<char>) override {
    node_entered.store(true);
    if (block_node_until_cancel) {
      while (!cancellation || !cancellation->IsCancellationRequested()) {
        std::this_thread::yield();
      }
      return Result<void>::Failure(Error::Cancelled("fake node safe point"));
    }    return Result<void>::Ok();
  }
  Result<void> RunStage(StageId stage) override {
    {
      std::lock_guard lock(mutex);
      calls.push_back(stage);
    }
    if (stage == StageId::kAlignment && request_alignment_feedback) {
      AlignmentFeedbackSnapshot snapshot;
      snapshot.proposal.target_agent = 'A';
      snapshot.proposal.source_agent = 'B';
      snapshot.proposal.method = AlignmentMethod::kKissMatcher;
      auto response = alignment_feedback->Request(std::move(snapshot), cancellation);
      if (!response) return Result<void>::Failure(response.GetError());
      if (response.Value().decision == AlignmentDecision::kCancel) {
        return Result<void>::Failure(Error::Cancelled("feedback cancelled"));
      }
    }
    if (stage == StageId::kAlignment && coordinate_alignment) {
      MapAlignmentCoordinatorInput input;
      input.feedback = alignment_feedback;
      input.cancellation = cancellation;
      input.feedback_timeout = alignment_timeout;
      input.target_agent = 'A';
      input.source_agent = 'B';
      input.kiss_proposer = [] {
        MapAlignmentProposal proposal;
        proposal.target_agent = 'A';
        proposal.source_agent = 'B';
        proposal.method = AlignmentMethod::kKissMatcher;
        proposal.target_T_source.translation().x() = 1;
        return std::optional(proposal);
      };
      input.descriptor_proposer = [] {
        MapAlignmentProposal proposal;
        proposal.target_agent = 'A';
        proposal.source_agent = 'B';
        proposal.method = AlignmentMethod::kDescriptor;
        proposal.target_T_source.translation().x() = 2;
        return std::optional(proposal);
      };
      auto coordinated = MapAlignmentCoordinator().Align(input);
      if (!coordinated) {
        return Result<void>::Failure(coordinated.GetError());
      }
      coordinated_alignment = coordinated.Value();
    }
    if (fail_stage && *fail_stage == stage) {
      return Result<void>::Failure(Error::InvalidArgument("induced failure"));
    }
    return Result<void>::Ok();
  }
  Result<void> RunOptimizeThrough(char target) override {
    replay_target = target;
    return replay_fails
        ? Result<void>::Failure(Error::OptimizationFailed("induced replay"))
        : Result<void>::Ok();
  }
  Result<void> Reconfigure(ConfigDomain domain) override {
    reconfigured_domain = domain;
    return fail_reconfigure
        ? Result<void>::Failure(
              Error::InvalidArgument("fake reconfigure failure"))
        : Result<void>::Ok();
  }
  std::vector<char> AgentIds() const override { return agent_ids; }

  mutable std::mutex mutex;
  std::vector<StageId> calls;
  std::optional<StageId> fail_stage;
  std::optional<char> replay_target;
  bool replay_fails = false;
  bool fail_reconfigure = false;
  std::optional<ConfigDomain> reconfigured_domain;
  bool block_node_until_cancel = false;
  std::atomic<bool> node_entered{false};
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback;
  bool request_alignment_feedback = false;
  bool coordinate_alignment = false;
  std::chrono::milliseconds alignment_timeout{};
  std::optional<MapAlignmentProposal> coordinated_alignment;
  std::vector<char> agent_ids{'A', 'B'};
};

ArtifactState StateOf(const PipelineSnapshot& snapshot, ArtifactType type,
                      std::optional<char> agent = std::nullopt) {
  for (const auto& artifact : snapshot.artifacts) {
    if (artifact.key.type == type && artifact.key.agent == agent) {
      return artifact.state;
    }
  }
  return ArtifactState::kMissing;
}

void TestRunAllAndArtifacts() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  auto submitted = controller.SubmitRunAll();
  Check(submitted.IsOk(), "RunAll submission");
  Check(controller.Wait(submitted.Value()).IsOk(), "RunAll completion");
  auto snapshot = controller.Snapshot();
  Check(snapshot.job && snapshot.job->state == JobState::kSucceeded,
        "successful job snapshot");
  Check(runner->calls.size() == 4, "all four stages called");
  Check(StateOf(snapshot, ArtifactType::kRawData, 'A') == ArtifactState::kReady,
        "raw artifact ready");
  Check(StateOf(snapshot, ArtifactType::kOptimizerState) == ArtifactState::kReady,
        "optimizer artifact ready");
  Check(StateOf(snapshot, ArtifactType::kMapAlignment, 'B') ==
            ArtifactState::kReady,
        "map alignment artifact ready");
  Check(StateOf(snapshot, ArtifactType::kPoseFile, 'B') == ArtifactState::kReady,
        "pose artifact ready");
}

void TestRerunInvalidatesDownstream() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  auto all = controller.SubmitRunAll();
  Check(all && controller.Wait(all.Value()), "initial RunAll");
  auto load = controller.SubmitStage(StageId::kDataLoad);
  Check(load && controller.Wait(load.Value()), "DataLoad rerun");
  auto snapshot = controller.Snapshot();
  Check(StateOf(snapshot, ArtifactType::kRawData, 'A') == ArtifactState::kReady,
        "rerun output ready");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, 'A') ==
            ArtifactState::kStale,
        "alignment output stale after load rerun");
  Check(StateOf(snapshot, ArtifactType::kPoseFile, 'A') == ArtifactState::kStale,
        "saved pose stale after load rerun");
}

void TestFailureStopsPipeline() {
  auto runner = std::make_shared<FakeRunner>();
  runner->fail_stage = StageId::kAlignment;
  PipelineController controller(runner);
  auto job = controller.SubmitRunAll();
  Check(job.IsOk(), "failure job submission");
  Check(!controller.Wait(job.Value()), "failure propagated");
  Check(runner->calls.size() == 2, "later stages skipped");
  auto snapshot = controller.Snapshot();
  Check(snapshot.job && snapshot.job->state == JobState::kFailed,
        "failed snapshot");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, 'A') ==
            ArtifactState::kFailed,
        "failed artifact marked");
}

void TestBoundaryCancellation() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  uint64_t id = 0;
  auto subscription =
      controller.SubscribeEvents([&](const ExecutionEvent& event) {
        if (event.type == EventType::kStageCompleted &&
            event.stage == StageId::kDataLoad) {
          auto result = controller.Cancel(event.job_id);
          Check(result.IsOk(), "cancel request accepted");
        }
      });
  auto job = controller.SubmitRunAll();
  Check(job.IsOk(), "cancel job submission");
  id = job.Value();
  Check(!controller.Wait(id), "cancel returned as failure result");
  auto snapshot = controller.Snapshot();
  Check(snapshot.job && snapshot.job->state == JobState::kCancelled,
        "cancelled snapshot");
  Check(runner->calls.size() == 1, "cancel applied before next stage");
}

void TestOptimizerReplay() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  auto job = controller.SubmitOptimizeThrough('A');
  Check(job && controller.Wait(job.Value()), "optimizer replay completion");
  Check(runner->replay_target == 'A', "optimizer target forwarded");
  auto snapshot = controller.Snapshot();
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, 'A') ==
            ArtifactState::kReady,
        "replayed agent pose ready");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, 'B') ==
            ArtifactState::kStale,
        "later agent pose stale");
}

void TestNodeCommandsAndMetadata() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  const auto descriptors = controller.NodeDescriptors();
  Check(descriptors.size() == 5, "all node descriptors exposed");
  Check(descriptors[2].ordered, "optimizer metadata is ordered");
  Check(!controller.SubmitNode(NodeId::kLoopDetect, 'A'),
        "missing RawData rejects node before job creation");

  std::vector<uint64_t> sequences;
  auto subscription =
      controller.SubscribeEvents([&](const ExecutionEvent& event) {
        sequences.push_back(event.sequence);
      });
  auto load = controller.SubmitNode(NodeId::kDataLoad, 'A');
  Check(load && controller.Wait(load.Value()), "agent DataLoad node");
  auto loop = controller.SubmitNode(NodeId::kLoopDetect, 'A');
  Check(loop && controller.Wait(loop.Value()), "agent LoopDetect node");
  auto optimize = controller.SubmitNode(NodeId::kOptimize, 'A');
  Check(optimize && controller.Wait(optimize.Value()), "agent Optimize node");
  for (std::size_t i = 1; i < sequences.size(); ++i) {
    Check(sequences[i] > sequences[i - 1], "event sequence monotonic");
  }
  const auto snapshot = controller.Snapshot();
  Check(!snapshot.recent_events.empty(), "event history available in snapshot");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, 'A') ==
            ArtifactState::kReady,
        "node output committed");
}

void TestConfigApplyInvalidation() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  auto load = controller.SubmitNode(NodeId::kDataLoad, 'A');
  Check(load && controller.Wait(load.Value()), "load before config apply");
  Check(controller.ApplyConfig(ConfigDomain::kDataLoader, 2).IsOk(),
        "increasing config revision applied");
  Check(runner->reconfigured_domain == ConfigDomain::kDataLoader,
        "config is applied to the active runner");
  const auto snapshot = controller.Snapshot();
  Check(snapshot.config_revision == 2, "snapshot config revision");
  Check(StateOf(snapshot, ArtifactType::kRawData, 'A') == ArtifactState::kStale,
        "loader config invalidates raw data");
  Check(!controller.SubmitNode(NodeId::kLoopDetect, 'A'),
        "stale artifact rejected as node input");
  Check(!controller.ApplyConfig(ConfigDomain::kOptimizer, 2),
        "non-increasing config revision rejected");

  runner->fail_reconfigure = true;
  Check(!controller.ApplyConfig(ConfigDomain::kOptimizer, 3),
        "runner reconfigure failure is propagated");
  Check(controller.Snapshot().config_revision == 2,
        "failed runner reconfigure does not commit revision");
}


void TestNodeCancellationRollsBackArtifacts() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  auto initial = controller.SubmitNode(NodeId::kDataLoad, 'A');
  Check(initial && controller.Wait(initial.Value()), "initial artifact commit");
  const auto before = controller.Snapshot();
  uint64_t raw_revision = 0;
  for (const auto& artifact : before.artifacts) {
    if (artifact.key.type == ArtifactType::kRawData &&
        artifact.key.agent == std::optional<char>('A')) {
      raw_revision = artifact.revision;
    }
  }

  runner->node_entered = false;
  runner->block_node_until_cancel = true;
  auto rerun = controller.SubmitNode(NodeId::kDataLoad, 'A');
  Check(rerun.IsOk(), "cancellable node submitted");
  while (!runner->node_entered.load()) std::this_thread::yield();
  Check(controller.Cancel(rerun.Value()).IsOk(), "in-node cancel accepted");
  Check(!controller.Wait(rerun.Value()), "cancelled node result");
  const auto after = controller.Snapshot();
  Check(after.job && after.job->state == JobState::kCancelled,
        "node job transitions through cancellation");
  bool preserved = false;
  for (const auto& artifact : after.artifacts) {
    if (artifact.key.type == ArtifactType::kRawData &&
        artifact.key.agent == std::optional<char>('A')) {
      preserved = artifact.state == ArtifactState::kReady &&
                  artifact.revision == raw_revision;
    }
  }
  Check(preserved, "cancel restores committed artifact revision");
}

void TestSessionRunnerReplacement() {
  auto first = std::make_shared<FakeRunner>();
  PipelineController controller(first);
  auto replacement = std::make_shared<FakeRunner>();
  replacement->agent_ids = {'C'};
  const auto before_revision = controller.Snapshot().config_revision;
  Check(controller.ReplaceRunner(replacement).IsOk(),
        "idle controller accepts a new session runner");
  const auto snapshot = controller.Snapshot();
  Check(snapshot.config_revision == before_revision + 1,
        "new session advances config revision");
  Check(snapshot.agents == std::vector<char>{'C'},
        "new session resets registered agents");
  Check(StateOf(snapshot, ArtifactType::kAgentInput, 'C') ==
            ArtifactState::kReady,
        "new session resets artifacts");
}

void TestAlignmentFeedbackAcceptAndStaleResponse() {
  auto runner = std::make_shared<FakeRunner>();
  runner->request_alignment_feedback = true;
  PipelineController controller(runner);
  controller.SetAlignmentFeedbackEnabled(true);
  auto job = controller.SubmitStage(StageId::kAlignment);
  Check(job.IsOk(), "interactive alignment submitted");
  std::optional<AlignmentFeedbackSnapshot> request;
  while (!(request = controller.GetAlignmentFeedbackSnapshot())) {
    std::this_thread::yield();
  }
  Check(controller.Snapshot().job->state ==
            JobState::kWaitingForAlignmentFeedback,
        "job waits for alignment feedback");
  AlignmentResponse response{request->proposal.request_id,
                             AlignmentDecision::kAccept, std::nullopt};
  Check(controller.RespondToAlignment(job.Value(), response).IsOk(),
        "alignment response accepted");
  Check(controller.Wait(job.Value()).IsOk(), "alignment resumes after feedback");
  Check(!controller.RespondToAlignment(job.Value(), response),
        "stale alignment response rejected");
}

void TestAlignmentFeedbackCancellation() {
  auto runner = std::make_shared<FakeRunner>();
  runner->request_alignment_feedback = true;
  PipelineController controller(runner);
  controller.SetAlignmentFeedbackEnabled(true);
  auto job = controller.SubmitStage(StageId::kAlignment);
  Check(job.IsOk(), "cancellable alignment submitted");
  while (!controller.GetAlignmentFeedbackSnapshot()) std::this_thread::yield();
  Check(controller.Cancel(job.Value()).IsOk(),
        "feedback wait accepts cancellation");
  Check(!controller.Wait(job.Value()), "cancelled feedback job fails wait");
  Check(controller.Snapshot().job->state == JobState::kCancelled,
        "feedback cancellation reaches cancelled state");
}

void TestControllerCoordinatorFullFallbackIntegration() {
  auto runner = std::make_shared<FakeRunner>();
  runner->coordinate_alignment = true;
  PipelineController controller(runner);
  controller.SetAlignmentFeedbackEnabled(true);
  auto job = controller.SubmitStage(StageId::kAlignment);
  Check(job.IsOk(), "coordinator integration submitted");

  std::optional<AlignmentFeedbackSnapshot> request;
  while (!(request = controller.GetAlignmentFeedbackSnapshot())) {
    std::this_thread::yield();
  }
  const uint64_t kiss_request = request->proposal.request_id;
  Check(controller.RespondToAlignment(
            job.Value(), {kiss_request, AlignmentDecision::kTryDescriptor,
                          std::nullopt}).IsOk(),
        "integration rejects KISS");
  do {
    request = controller.GetAlignmentFeedbackSnapshot();
    std::this_thread::yield();
  } while (!request || request->proposal.request_id == kiss_request);
  Check(request->proposal.method == AlignmentMethod::kDescriptor,
        "integration reaches Descriptor");

  Eigen::Isometry3d invalid = Eigen::Isometry3d::Identity();
  invalid.linear()(0, 0) = 2;
  Check(!controller.RespondToAlignment(
            job.Value(), {request->proposal.request_id,
                          AlignmentDecision::kManual, invalid}),
        "integration rejects invalid Manual transform");
  Check(controller.Snapshot().job->state ==
            JobState::kWaitingForAlignmentFeedback,
        "invalid response keeps integration waiting");

  Eigen::Isometry3d manual = Eigen::Isometry3d::Identity();
  manual.translation().z() = 9;
  Check(controller.RespondToAlignment(
            job.Value(), {request->proposal.request_id,
                          AlignmentDecision::kManual, manual}).IsOk(),
        "integration accepts valid Manual transform");
  Check(controller.Wait(job.Value()).IsOk(), "integration job completes");
  Check(runner->coordinated_alignment &&
            runner->coordinated_alignment->method == AlignmentMethod::kManual &&
            runner->coordinated_alignment->target_T_source.translation().z() == 9,
        "integration preserves Manual result");
}

void TestControllerCoordinatorTimeoutIntegration() {
  auto runner = std::make_shared<FakeRunner>();
  runner->coordinate_alignment = true;
  runner->alignment_timeout = std::chrono::milliseconds(5);
  PipelineController controller(runner);
  controller.SetAlignmentFeedbackEnabled(true);
  auto job = controller.SubmitStage(StageId::kAlignment);
  Check(job.IsOk(), "timeout integration submitted");
  Check(!controller.Wait(job.Value()), "timeout integration fails job");
  Check(controller.Snapshot().job->state == JobState::kFailed,
        "timeout integration reaches failed state");
  Check(!controller.GetAlignmentFeedbackSnapshot(),
        "timeout integration clears feedback request");
}

}  // namespace

int main() {
  TestRunAllAndArtifacts();
  TestRerunInvalidatesDownstream();
  TestFailureStopsPipeline();
  TestBoundaryCancellation();
  TestOptimizerReplay();
  TestNodeCommandsAndMetadata();
  TestConfigApplyInvalidation();
  TestNodeCancellationRollsBackArtifacts();
  TestSessionRunnerReplacement();
  TestAlignmentFeedbackAcceptAndStaleResponse();
  TestAlignmentFeedbackCancellation();
  TestControllerCoordinatorFullFallbackIntegration();
  TestControllerCoordinatorTimeoutIntegration();
  std::cout << "pipeline controller tests passed\n";
  return 0;
}
