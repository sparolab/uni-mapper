#include <open_lmm/server/pipeline_controller.hpp>

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
    std::lock_guard lock(mutex);
    calls.push_back(stage);
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
  std::vector<char> AgentIds() const override { return {'A', 'B'}; }

  mutable std::mutex mutex;
  std::vector<StageId> calls;
  std::optional<StageId> fail_stage;
  std::optional<char> replay_target;
  bool replay_fails = false;
  bool block_node_until_cancel = false;
  std::atomic<bool> node_entered{false};
  std::shared_ptr<CancellationToken> cancellation;
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
  controller.SetEventCallback([&](const ExecutionEvent& event) {
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
  controller.SetEventCallback([&](const ExecutionEvent& event) {
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
  const auto snapshot = controller.Snapshot();
  Check(snapshot.config_revision == 2, "snapshot config revision");
  Check(StateOf(snapshot, ArtifactType::kRawData, 'A') == ArtifactState::kStale,
        "loader config invalidates raw data");
  Check(!controller.SubmitNode(NodeId::kLoopDetect, 'A'),
        "stale artifact rejected as node input");
  Check(!controller.ApplyConfig(ConfigDomain::kOptimizer, 2),
        "non-increasing config revision rejected");
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
  std::cout << "pipeline controller tests passed\n";
  return 0;
}
