#include <open_lmm/server/pipeline_controller.hpp>
#include <open_lmm/core/loop_detector/map_alignment_coordinator.hpp>
#include "test_runtime_port.hpp"

#include <algorithm>
#include <cstdlib>
#include <future>
#include <iostream>
#include <memory>
#include <mutex>
#include <vector>
#include <thread>

using namespace open_lmm;

namespace {
AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

void Check(bool condition, const char* message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

class FakeRunner final : public test::RuntimePortFixture {
 public:
  enum class ReceiptFault { kNone, kBase, kCommitted, kNoMutation };
  explicit FakeRunner(
      std::vector<AgentId> agents = {Id("A"), Id("B")})
      : RuntimePortFixture(std::move(agents)) {}

  CommittedSessionSnapshot Snapshot() const override {
    if (on_agent_ids) on_agent_ids();
    return RuntimePortFixture::Snapshot();
  }

  Result<void> ExecuteFixture(const ExecutionCommand& command,
                              const ExecutionContext& context) override {
    cancellation = context.cancellation;
    alignment_feedback = context.alignment_feedback;
    switch (command.kind) {
      case ExecutionCommandKind::kStage:
        return RunStage(*command.stage);
      case ExecutionCommandKind::kNode:
        return RunNode(*command.node, command.agent);
      case ExecutionCommandKind::kOptimizeThrough:
        return RunOptimizeThrough(*command.agent);
      case ExecutionCommandKind::kReconfigure:
        return Reconfigure(*command.config_domain, command.config_revision);
    }
    return Result<void>::Failure(Error::InvalidArgument("unknown fixture command"));
  }
  Result<void> RunNode(NodeId, std::optional<AgentId>) {
    node_entered.store(true);
    if (non_cooperative_node) {
      while (!release_non_cooperative.load(std::memory_order_acquire)) {
        std::this_thread::yield();
      }
      node_exited.store(true, std::memory_order_release);
      return Result<void>::Ok();
    }
    if (block_node_until_cancel) {
      while (!cancellation || !cancellation->IsCancellationRequested()) {
        std::this_thread::yield();
      }
      return Result<void>::Failure(Error::Cancelled("fake node safe point"));
    }    return Result<void>::Ok();
  }
  CancellationCapability CancellationMetadata() const override {
    if (!non_cooperative_node) return {};
    return {.cooperative = false,
            .mode = CancellationMode::kNonCooperative,
            .non_interruptible_operations = {"fake blocking plugin"},
            .requires_process_isolation = true};
  }
  Result<void> RunStage(StageId stage) {
    {
      std::lock_guard lock(mutex);
      calls.push_back(stage);
    }
    if (stage == StageId::kAlignment && request_alignment_feedback) {
      std::unique_lock<std::mutex> runner_lock(mutex, std::defer_lock);
      if (hold_mutex_during_feedback) {
        runner_lock.lock();
        feedback_mutex_held.store(true);
        while (!allow_feedback_notification.load()) std::this_thread::yield();
      }
      AlignmentFeedbackSnapshot snapshot;
      snapshot.proposal.target_agent = Id("A");
      snapshot.proposal.source_agent = Id("B");
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
      input.target_agent = Id("A");
      input.source_agent = Id("B");
      input.kiss_proposer = [] {
        MapAlignmentProposal proposal;
        proposal.target_agent = Id("A");
        proposal.source_agent = Id("B");
        proposal.method = AlignmentMethod::kKissMatcher;
        proposal.target_T_source.translation().x() = 1;
        return Result<std::optional<MapAlignmentProposal>>::Ok(
            std::optional<MapAlignmentProposal>(std::move(proposal)));
      };
      input.descriptor_proposer = [] {
        MapAlignmentProposal proposal;
        proposal.target_agent = Id("A");
        proposal.source_agent = Id("B");
        proposal.method = AlignmentMethod::kDescriptor;
        proposal.target_T_source.translation().x() = 2;
        return Result<std::optional<MapAlignmentProposal>>::Ok(
            std::optional<MapAlignmentProposal>(std::move(proposal)));
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
  Result<void> RunOptimizeThrough(const AgentId& target) {
    replay_target = target;
    return replay_fails
        ? Result<void>::Failure(Error::OptimizationFailed("induced replay"))
        : Result<void>::Ok();
  }
  Result<void> Reconfigure(ConfigDomain domain, uint64_t) {
    reconfigured_domain = domain;
    return fail_reconfigure
        ? Result<void>::Failure(
              Error::InvalidArgument("fake reconfigure failure"))
        : Result<void>::Ok();
  }
  ExecutionReceipt AdjustReceipt(ExecutionReceipt receipt) const override {
    switch (receipt_fault) {
      case ReceiptFault::kNone: break;
      case ReceiptFault::kBase: ++receipt.base_revision; break;
      case ReceiptFault::kCommitted: ++receipt.committed_revision; break;
      case ReceiptFault::kNoMutation:
        receipt.committed_revision = receipt.base_revision;
        break;
    }
    return receipt;
  }
  mutable std::mutex mutex;
  std::vector<StageId> calls;
  std::optional<StageId> fail_stage;
  std::optional<AgentId> replay_target;
  bool replay_fails = false;
  bool fail_reconfigure = false;
  ReceiptFault receipt_fault = ReceiptFault::kNone;
  std::optional<ConfigDomain> reconfigured_domain;
  bool block_node_until_cancel = false;
  std::atomic<bool> node_entered{false};
  bool non_cooperative_node = false;
  std::atomic<bool> release_non_cooperative{false};
  std::atomic<bool> node_exited{false};
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback;
  bool request_alignment_feedback = false;
  bool coordinate_alignment = false;
  std::chrono::milliseconds alignment_timeout{};
  std::optional<MapAlignmentProposal> coordinated_alignment;
  bool hold_mutex_during_feedback = false;
  std::atomic<bool> feedback_mutex_held{false};
  std::atomic<bool> allow_feedback_notification{false};
  std::function<void()> on_agent_ids;
};

class ManagedSessionRunner final : public test::RuntimePortFixture {
 public:
  ManagedSessionRunner() : RuntimePortFixture({Id("A")}, 5) {}

  Result<void> ExecuteFixture(const ExecutionCommand&,
                              const ExecutionContext&) override {
    if (fail_next.exchange(false)) {
      return Result<void>::Failure(Error::InvalidArgument("induced failure"));
    }
    return Result<void>::Ok();
  }

  std::atomic<bool> fail_next{false};
};

ArtifactState StateOf(const PipelineSnapshot& snapshot, ArtifactType type,
                      std::optional<AgentId> agent = std::nullopt) {
  for (const auto& artifact : snapshot.artifacts) {
    if (artifact.key.type == type && artifact.key.agent == agent) {
      return artifact.state;
    }
  }
  return ArtifactState::kMissing;
}

bool SameArtifacts(const std::vector<ArtifactMetadata>& lhs,
                   const std::vector<ArtifactMetadata>& rhs) {
  if (lhs.size() != rhs.size()) return false;
  for (std::size_t i = 0; i < lhs.size(); ++i) {
    const auto& a = lhs[i];
    const auto& b = rhs[i];
    if (a.key != b.key || a.state != b.state || a.revision != b.revision ||
        a.producer != b.producer || a.detail != b.detail ||
        a.external_path != b.external_path ||
        a.fingerprint != b.fingerprint) {
      return false;
    }
  }
  return true;
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
  Check(StateOf(snapshot, ArtifactType::kRawData, Id("A")) == ArtifactState::kReady,
        "raw artifact ready");
  Check(StateOf(snapshot, ArtifactType::kOptimizerState) == ArtifactState::kReady,
        "optimizer artifact ready");
  Check(StateOf(snapshot, ArtifactType::kMapAlignment, Id("B")) ==
            ArtifactState::kReady,
        "map alignment artifact ready");
  Check(StateOf(snapshot, ArtifactType::kPoseFile, Id("B")) == ArtifactState::kReady,
        "pose artifact ready");
}

void TestMalformedExecutionReceiptsCannotPublishSuccess() {
  for (const auto fault : {FakeRunner::ReceiptFault::kBase,
                           FakeRunner::ReceiptFault::kCommitted,
                           FakeRunner::ReceiptFault::kNoMutation}) {
    auto runner = std::make_shared<FakeRunner>();
    runner->receipt_fault = fault;
    PipelineController controller(runner);
    const auto submitted = controller.SubmitStage(StageId::kDataLoad);
    Check(submitted && !controller.Wait(submitted.Value()),
          "malformed successful receipt fails the job");
    const auto snapshot = controller.Snapshot();
    Check(snapshot.job && snapshot.job->state == JobState::kFailed,
          "receipt mismatch transitions to failed");
    Check(std::none_of(snapshot.recent_events.begin(),
                       snapshot.recent_events.end(),
                       [](const ExecutionEvent& event) {
                         return event.type == EventType::kStageCompleted;
                       }),
          "receipt mismatch cannot publish stage success");
    Check(StateOf(snapshot, ArtifactType::kRawData, Id("A")) ==
              ArtifactState::kReady,
          "controller resynchronizes the authoritative post-commit snapshot");
    runner->receipt_fault = FakeRunner::ReceiptFault::kNone;
    const auto rejected = controller.SubmitStage(StageId::kDataLoad);
    Check(!rejected &&
              rejected.GetError().severity == Error::Severity::kFatalSession,
          "receipt protocol failure rejects later commands as fatal");
  }
}

void TestMalformedReconfigureReceiptResynchronizesAndPoisonsSession() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  const auto loaded = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(loaded && controller.Wait(loaded.Value()),
        "prepare committed raw artifact before malformed reconfigure");

  runner->receipt_fault = FakeRunner::ReceiptFault::kCommitted;
  const auto applied = controller.ApplyConfig(ConfigDomain::kDataLoader, 2);
  Check(!applied &&
            applied.GetError().severity == Error::Severity::kFatalSession,
        "malformed post-commit reconfigure receipt is a fatal protocol error");
  const auto snapshot = controller.Snapshot();
  Check(snapshot.config_revision == 2 &&
            StateOf(snapshot, ArtifactType::kRawData, Id("A")) ==
                ArtifactState::kStale,
        "failed ApplyConfig result still publishes authoritative committed state");
  const auto committed = runner->Snapshot();
  Check(committed.revision == 3 && committed.config_revision == 2,
        "fixture confirms reconfigure committed before receipt corruption");

  runner->receipt_fault = FakeRunner::ReceiptFault::kNone;
  const auto retry = controller.ApplyConfig(ConfigDomain::kOptimizer, 3);
  Check(!retry && retry.GetError().severity == Error::Severity::kFatalSession &&
            runner->Snapshot().revision == committed.revision,
        "poisoned controller performs no further reconfigure command");

  auto replacement = std::make_shared<FakeRunner>();
  Check(controller.ReplacePorts(replacement, replacement).IsOk(),
        "fresh session replacement clears the protocol failure");
  const auto recovered = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(recovered && controller.Wait(recovered.Value()),
        "replacement session accepts commands after protocol failure");
}

void TestConcurrentSubmissionsAreSerialized() {
  auto runner = std::make_shared<FakeRunner>();
  runner->block_node_until_cancel = true;
  PipelineController controller(runner);
  std::atomic<int> ready{0};
  std::atomic<bool> go{false};
  bool accepted[2] = {false, false};
  uint64_t job_ids[2] = {0, 0};
  auto submit = [&](int index) {
    ++ready;
    while (!go.load()) std::this_thread::yield();
    auto result = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
    accepted[index] = result.IsOk();
    if (result) job_ids[index] = result.Value();
  };
  std::thread first(submit, 0);
  std::thread second(submit, 1);
  while (ready.load() != 2) std::this_thread::yield();
  go = true;
  first.join();
  second.join();
  Check(accepted[0] != accepted[1],
        "only one concurrent pipeline submission is accepted");
  const uint64_t accepted_id = accepted[0] ? job_ids[0] : job_ids[1];
  while (!runner->node_entered.load()) std::this_thread::yield();
  Check(controller.Cancel(accepted_id).IsOk(),
        "accepted concurrent submission remains cancellable");
  Check(!controller.Wait(accepted_id),
        "cancelled concurrent submission completes");
}

void TestRerunInvalidatesDownstream() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  auto all = controller.SubmitRunAll();
  Check(all && controller.Wait(all.Value()), "initial RunAll");
  auto load = controller.SubmitStage(StageId::kDataLoad);
  Check(load && controller.Wait(load.Value()), "DataLoad rerun");
  auto snapshot = controller.Snapshot();
  Check(StateOf(snapshot, ArtifactType::kRawData, Id("A")) == ArtifactState::kReady,
        "rerun output ready");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, Id("A")) ==
            ArtifactState::kStale,
        "alignment output stale after load rerun");
  Check(StateOf(snapshot, ArtifactType::kPoseFile, Id("A")) == ArtifactState::kStale,
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
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, Id("A")) !=
            ArtifactState::kFailed,
        "failed command leaves committed artifact metadata unchanged");
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
  auto job = controller.SubmitOptimizeThrough(Id("A"));
  Check(job && controller.Wait(job.Value()), "optimizer replay completion");
  Check(runner->replay_target == Id("A"), "optimizer target forwarded");
  auto snapshot = controller.Snapshot();
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, Id("A")) ==
            ArtifactState::kReady,
        "replayed agent pose ready");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, Id("B")) ==
            ArtifactState::kStale,
        "later agent pose stale");
}

void TestNodeCommandsAndMetadata() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  const auto descriptors = controller.NodeDescriptors();
  Check(descriptors.size() == 6, "all node descriptors exposed");
  Check(descriptors[2].ordered, "optimizer metadata is ordered");
  Check(descriptors[4].scope == ExecutionScope::kSession &&
            descriptors[5].scope == ExecutionScope::kSession,
        "session node scope is exposed to API consumers");
  auto missing_input = controller.SubmitNode(NodeId::kLoopDetect, Id("A"));
  Check(missing_input && !controller.Wait(missing_input.Value()),
        "command port rejects node with missing RawData");

  std::vector<uint64_t> sequences;
  auto subscription =
      controller.SubscribeEvents([&](const ExecutionEvent& event) {
        sequences.push_back(event.sequence);
      });
  auto load = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(load && controller.Wait(load.Value()), "agent DataLoad node");
  auto loop = controller.SubmitNode(NodeId::kLoopDetect, Id("A"));
  Check(loop && controller.Wait(loop.Value()), "agent LoopDetect node");
  auto optimize = controller.SubmitNode(NodeId::kOptimize, Id("A"));
  Check(optimize && controller.Wait(optimize.Value()), "agent Optimize node");
  auto save = controller.SubmitNode(NodeId::kPoseSave);
  Check(save && controller.Wait(save.Value()),
        "session PoseSave node does not require an agent target");
  Check(controller.WaitForEventCallbacks().IsOk(),
        "node event callbacks drain before inspecting subscriber state");
  for (std::size_t i = 1; i < sequences.size(); ++i) {
    Check(sequences[i] > sequences[i - 1], "event sequence monotonic");
  }
  const auto snapshot = controller.Snapshot();
  Check(!snapshot.recent_events.empty(), "event history available in snapshot");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, Id("A")) ==
            ArtifactState::kReady,
        "node output committed");
  const auto pose_commit = std::find_if(
      snapshot.recent_events.rbegin(), snapshot.recent_events.rend(),
      [](const ExecutionEvent& event) {
        return event.type == EventType::kArtifactCommitted &&
               event.node == NodeId::kPoseSave;
      });
  Check(pose_commit != snapshot.recent_events.rend() &&
            !pose_commit->agent &&
            pose_commit->affected_agents == std::vector<AgentId>{Id("A")},
        "PoseSave commit event reports its actual session file-set");
}

void TestConfigApplyInvalidation() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  auto load = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(load && controller.Wait(load.Value()), "load before config apply");
  Check(controller.ApplyConfig(ConfigDomain::kDataLoader, 2).IsOk(),
        "increasing config revision applied");
  Check(runner->reconfigured_domain == ConfigDomain::kDataLoader,
        "config is applied to the active runner");
  const auto snapshot = controller.Snapshot();
  Check(snapshot.config_revision == 2, "snapshot config revision");
  Check(StateOf(snapshot, ArtifactType::kRawData, Id("A")) == ArtifactState::kStale,
        "loader config invalidates raw data");
  auto stale_input = controller.SubmitNode(NodeId::kLoopDetect, Id("A"));
  Check(stale_input && !controller.Wait(stale_input.Value()),
        "command port rejects stale node input");
  Check(!controller.ApplyConfig(ConfigDomain::kOptimizer, 2),
        "non-increasing config revision rejected");

  runner->fail_reconfigure = true;
  const auto before_failure = runner->Snapshot();
  Check(!controller.ApplyConfig(ConfigDomain::kOptimizer, 3),
        "runner reconfigure failure is propagated");
  Check(controller.Snapshot().config_revision == 2,
        "failed runner reconfigure does not commit revision");
  const auto after_failure = runner->Snapshot();
  Check(after_failure.revision == before_failure.revision &&
            after_failure.config_revision == before_failure.config_revision &&
            SameArtifacts(after_failure.artifacts, before_failure.artifacts),
        "pre-commit reconfigure failure preserves revision and artifacts");
}


void TestNodeCancellationRollsBackArtifacts() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  auto initial = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(initial && controller.Wait(initial.Value()), "initial artifact commit");
  const auto before = controller.Snapshot();
  uint64_t raw_revision = 0;
  for (const auto& artifact : before.artifacts) {
    if (artifact.key.type == ArtifactType::kRawData &&
        artifact.key.agent == std::optional<AgentId>(Id("A"))) {
      raw_revision = artifact.revision;
    }
  }

  runner->node_entered = false;
  runner->block_node_until_cancel = true;
  auto rerun = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(rerun.IsOk(), "cancellable node submitted");
  while (!runner->node_entered.load()) std::this_thread::yield();
  Check(controller.Cancel(rerun.Value()).IsOk(), "in-node cancel accepted");
  Check(!controller.Wait(rerun.Value()), "cancelled node result");
  const auto after = controller.Snapshot();
  Check(after.job && after.job->state == JobState::kCancelled,
        "node job transitions through cancellation");
  Check(after.job->cancellation.capability.cooperative &&
            after.job->cancellation.cancel_requested_at_unix_ns &&
            after.job->cancellation.cancel_observed_at_unix_ns &&
            after.job->cancellation.cancel_completed_at_unix_ns,
        "cooperative cancellation exposes all timestamps");
  bool preserved = false;
  for (const auto& artifact : after.artifacts) {
    if (artifact.key.type == ArtifactType::kRawData &&
        artifact.key.agent == std::optional<AgentId>(Id("A"))) {
      preserved = artifact.state == ArtifactState::kReady &&
                  artifact.revision == raw_revision;
    }
  }
  Check(preserved, "cancel restores committed artifact revision");
  bool emitted_authoritative_change = false;
  for (const auto& event : after.recent_events) {
    if (event.job_id == rerun.Value() &&
        (event.type == EventType::kArtifactInvalidated ||
         event.type == EventType::kArtifactCommitted)) {
      emitted_authoritative_change = true;
    }
  }
  Check(!emitted_authoritative_change,
        "cancelled node emits no authoritative artifact event");
}

void TestCancellationTelemetryAndJoinOrdering() {
  auto runner = std::make_shared<FakeRunner>();
  runner->non_cooperative_node = true;
  runner->release_non_cooperative = true;
  PipelineController controller(runner);

  auto initial = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(initial && controller.Wait(initial.Value()),
        "initial non-cooperative fixture commit");
  const auto committed = controller.Snapshot().artifacts;

  runner->node_entered = false;
  runner->node_exited = false;
  runner->release_non_cooperative = false;
  std::atomic<int> terminal_events{0};
  std::atomic<bool> terminal_after_worker_exit{false};
  auto subscription = controller.SubscribeEvents(
      [&](const ExecutionEvent& event) {
        if (event.type != EventType::kJobCancelled) return;
        ++terminal_events;
        terminal_after_worker_exit = runner->node_exited.load();
        Check(event.cancellation &&
                  event.cancellation->cancel_completed_at_unix_ns,
              "terminal event carries completed cancellation telemetry");
      });

  auto job = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(job.IsOk(), "non-cooperative fixture submitted");
  while (!runner->node_entered.load()) std::this_thread::yield();
  const auto acknowledgement_start = std::chrono::steady_clock::now();
  Check(controller.Cancel(job.Value()).IsOk(),
        "non-cooperative cancellation request accepted");
  const auto acknowledgement_latency = std::chrono::steady_clock::now() -
                                       acknowledgement_start;
  Check(acknowledgement_latency < std::chrono::milliseconds(100),
        "cancel acknowledgement exceeds 100 ms");

  const auto pending = controller.Snapshot();
  Check(pending.job && pending.job->state == JobState::kCancelling &&
            pending.job->cancellation.Pending() &&
            pending.job->cancellation.cancel_requested_at_unix_ns &&
            !pending.job->cancellation.cancel_observed_at_unix_ns &&
            !pending.job->cancellation.capability.cooperative &&
            pending.job->cancellation.capability.requires_process_isolation,
        "non-cooperative pending context is observable");
  Check(terminal_events.load() == 0,
        "terminal event emitted while worker remains blocked");

  runner->release_non_cooperative = true;
  Check(!controller.Wait(job.Value()),
        "post-operation safe point reports cancellation");
  const auto completed = controller.Snapshot();
  const auto& telemetry = completed.job->cancellation;
  Check(telemetry.cancel_requested_at_unix_ns &&
            telemetry.cancel_observed_at_unix_ns &&
            telemetry.cancel_completed_at_unix_ns &&
            *telemetry.cancel_requested_at_unix_ns <=
                *telemetry.cancel_observed_at_unix_ns &&
            *telemetry.cancel_observed_at_unix_ns <=
                *telemetry.cancel_completed_at_unix_ns,
        "cancellation timestamps are complete and ordered");
  Check(controller.WaitForEventCallbacks().IsOk(),
        "cancellation callbacks drain before subscriber inspection");
  Check(terminal_events.load() == 1 && terminal_after_worker_exit.load(),
        "terminal cancellation is emitted once after worker join");
  bool artifacts_preserved = completed.artifacts.size() == committed.size();
  for (std::size_t index = 0;
       artifacts_preserved && index < committed.size(); ++index) {
    const auto& before = committed[index];
    const auto& after = completed.artifacts[index];
    artifacts_preserved = before.key == after.key &&
                          before.state == after.state &&
                          before.revision == after.revision &&
                          before.external_path == after.external_path;
  }
  Check(artifacts_preserved,
        "cancelled non-cooperative job preserves committed artifacts");
}

void TestManagedSessionMetadataIsAuthoritative() {
  auto runner = std::make_shared<ManagedSessionRunner>();
  PipelineController controller(runner);
  Check(controller.Snapshot().config_revision == 5,
        "controller imports committed session config revision");
  auto first = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(first && controller.Wait(first.Value()),
        "managed session commits node payload and metadata");
  const auto committed = controller.Snapshot();
  uint64_t raw_revision = 0;
  for (const auto& artifact : committed.artifacts) {
    if (artifact.key == ArtifactKey{ArtifactType::kRawData, Id("A")}) {
      raw_revision = artifact.revision;
    }
  }
  Check(raw_revision != 0 &&
            StateOf(committed, ArtifactType::kRawData, Id("A")) ==
                ArtifactState::kReady,
        "controller exposes runner committed artifact");

  runner->fail_next = true;
  auto failed = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(failed && !controller.Wait(failed.Value()),
        "managed session failure is propagated");
  const auto after = controller.Snapshot();
  bool unchanged = false;
  for (const auto& artifact : after.artifacts) {
    if (artifact.key == ArtifactKey{ArtifactType::kRawData, Id("A")}) {
      unchanged = artifact.state == ArtifactState::kReady &&
                  artifact.revision == raw_revision;
    }
  }
  Check(unchanged,
        "controller does not synthesize metadata after transaction failure");
  const auto failed_event = std::find_if(
      after.recent_events.rbegin(), after.recent_events.rend(),
      [](const ExecutionEvent& event) {
        return event.type == EventType::kNodeFailed;
      });
  Check(failed_event != after.recent_events.rend() && failed_event->error &&
            failed_event->error->severity == Error::Severity::kRecoverable &&
            failed_event->error->context.session_revision ==
                std::optional<uint64_t>(2) &&
            failed_event->error->context.stage == "data_load" &&
            failed_event->error->context.node == "data_load" &&
            failed_event->error->context.agent == std::optional<AgentId>(Id("A")),
        "recoverable node error carries session/stage/node/agent context");
}

void TestSessionRunnerReplacement() {
  auto first = std::make_shared<FakeRunner>();
  PipelineController controller(first);
  auto replacement = std::make_shared<FakeRunner>(
      std::vector<AgentId>{Id("C")});
  const auto before_revision = controller.Snapshot().config_revision;
  Check(controller.ReplacePorts(replacement, replacement).IsOk(),
        "idle controller accepts a new session runner");
  const auto snapshot = controller.Snapshot();
  Check(snapshot.config_revision == replacement->Snapshot().config_revision &&
            snapshot.config_revision == before_revision,
        "new session exposes the replacement port revision verbatim");
  Check(snapshot.agents == std::vector<AgentId>{Id("C")},
        "new session resets registered agents");
  Check(StateOf(snapshot, ArtifactType::kAgentInput, Id("C")) ==
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

void TestAlignmentFeedbackCanRespondFromRequestCallback() {
  auto runner = std::make_shared<FakeRunner>();
  runner->request_alignment_feedback = true;
  PipelineController controller(runner);
  controller.SetAlignmentFeedbackEnabled(true);
  std::mutex sequences_mutex;
  std::vector<uint64_t> sequences;
  std::atomic<bool> synchronous_response_ok{false};
  auto subscription = controller.SubscribeEvents(
      [&](const ExecutionEvent& event) {
        {
          std::lock_guard lock(sequences_mutex);
          sequences.push_back(event.sequence);
        }
        if (event.type != EventType::kAlignmentFeedbackRequested) return;
        const auto request = controller.GetAlignmentFeedbackSnapshot();
        synchronous_response_ok = request &&
            controller.RespondToAlignment(
                event.job_id,
                {request->proposal.request_id, AlignmentDecision::kAccept,
                 std::nullopt}).IsOk();
      });

  const auto job = controller.SubmitStage(StageId::kAlignment);
  Check(job && controller.Wait(job.Value()),
        "synchronous alignment callback did not complete the job");
  Check(synchronous_response_ok.load(),
        "request callback could not inspect and answer published feedback");
  {
    std::lock_guard lock(sequences_mutex);
    Check(std::adjacent_find(
              sequences.begin(), sequences.end(),
              [](uint64_t lhs, uint64_t rhs) { return lhs >= rhs; }) ==
              sequences.end(),
          "reentrant event callback reversed subscriber sequence");
  }
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

void TestSnapshotDoesNotInvokeRunnerWhileControllerLocked() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  runner->on_agent_ids = [&controller] { (void)controller.Snapshot(); };

  auto snapshot = std::async(std::launch::async,
                             [&controller] { return controller.Snapshot(); });
  Check(snapshot.wait_for(std::chrono::milliseconds(500)) ==
            std::future_status::ready,
        "Snapshot must not call the query port while holding controller state");
  Check(snapshot.get().agents == std::vector<AgentId>({Id("A"), Id("B")}),
        "Snapshot uses cached immutable agent IDs");
}

void TestSnapshotDuringFeedbackNotificationDoesNotDeadlock() {
  auto runner = std::make_shared<FakeRunner>();
  runner->request_alignment_feedback = true;
  runner->hold_mutex_during_feedback = true;
  PipelineController controller(runner);
  controller.SetAlignmentFeedbackEnabled(true);
  auto job = controller.SubmitStage(StageId::kAlignment);
  Check(job.IsOk(), "lock inversion regression job submitted");

  const auto deadline = std::chrono::steady_clock::now() +
                        std::chrono::milliseconds(500);
  while (!runner->feedback_mutex_held.load() &&
         std::chrono::steady_clock::now() < deadline) {
    std::this_thread::yield();
  }
  Check(runner->feedback_mutex_held.load(),
        "runner entered feedback lock inversion window");

  auto snapshot = std::async(std::launch::async,
                             [&controller] { return controller.Snapshot(); });
  Check(snapshot.wait_for(std::chrono::milliseconds(500)) ==
            std::future_status::ready,
        "Snapshot remains bounded while runner state is locked");
  Check(snapshot.get().job.has_value(),
        "concurrent Snapshot returns controller state");

  runner->allow_feedback_notification.store(true);
  std::optional<AlignmentFeedbackSnapshot> request;
  const auto feedback_deadline = std::chrono::steady_clock::now() +
                                 std::chrono::milliseconds(500);
  while (!(request = controller.GetAlignmentFeedbackSnapshot()) &&
         std::chrono::steady_clock::now() < feedback_deadline) {
    std::this_thread::yield();
  }
  Check(request.has_value(), "feedback notification completes after Snapshot");
  Check(controller.RespondToAlignment(
            job.Value(), {request->proposal.request_id,
                          AlignmentDecision::kAccept, std::nullopt}).IsOk(),
        "feedback response accepted after concurrent Snapshot");
  Check(controller.Wait(job.Value()).IsOk(),
        "feedback job completes without lock inversion");
}

void TestEventCallbackCanUnsubscribeReenterAndThrow() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  std::optional<ExecutionEventSubscription> subscription;
  std::atomic<int> callback_count{0};
  subscription.emplace(controller.SubscribeEvents(
      [&](const ExecutionEvent&) {
        ++callback_count;
        (void)controller.Snapshot();
        subscription->Reset();
        throw std::runtime_error("observer failure");
      }));
  auto job = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(job && controller.Wait(job.Value()),
        "subscriber exception does not fail pipeline job");
  Check(callback_count.load() == 1,
        "self-unsubscribe prevents later callbacks without deadlock");
}

void TestTerminalCommitPrecedesWaitAndReentrantCallback() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  std::promise<Result<void>> callback_wait_promise;
  auto callback_wait = callback_wait_promise.get_future();
  std::atomic<int> terminal_callbacks{0};
  std::atomic<bool> terminal_visible_in_snapshot{false};
  auto subscription = controller.SubscribeEvents(
      [&](const ExecutionEvent& event) {
        if (event.type != EventType::kJobCompleted &&
            event.type != EventType::kJobCancelled) {
          return;
        }
        if (terminal_callbacks.fetch_add(1) != 0) return;
        const auto snapshot = controller.Snapshot();
        terminal_visible_in_snapshot = std::count_if(
            snapshot.recent_events.begin(), snapshot.recent_events.end(),
            [&](const ExecutionEvent& candidate) {
              return candidate.job_id == event.job_id &&
                     (candidate.type == EventType::kJobCompleted ||
                      candidate.type == EventType::kJobCancelled);
            }) == 1;
        callback_wait_promise.set_value(controller.Wait(event.job_id));
      });

  const auto job = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(job && controller.Wait(job.Value()),
        "terminal journal commit did not release Wait");
  Check(callback_wait.wait_for(std::chrono::milliseconds(500)) ==
            std::future_status::ready &&
            callback_wait.get().IsOk(),
        "terminal callback could not re-enter Wait");
  Check(terminal_callbacks.load() == 1 &&
            terminal_visible_in_snapshot.load(),
        "terminal callback did not observe exactly one committed event");

  const auto snapshot = controller.Snapshot();
  const auto terminal_events = std::count_if(
      snapshot.recent_events.begin(), snapshot.recent_events.end(),
      [&](const ExecutionEvent& event) {
        return event.job_id == job.Value() &&
               (event.type == EventType::kJobCompleted ||
                event.type == EventType::kJobCancelled);
      });
  Check(terminal_events == 1,
        "successful job journal contains duplicate terminal events");
}

void TestTerminalCallbackRejectsWorkerLifecycleCommands() {
  auto runner = std::make_shared<FakeRunner>();
  PipelineController controller(runner);
  std::promise<bool> callback_result_promise;
  auto callback_result = callback_result_promise.get_future();
  std::atomic<bool> handled{false};
  auto subscription = controller.SubscribeEvents(
      [&](const ExecutionEvent& event) {
        if (event.type != EventType::kJobCompleted || handled.exchange(true)) {
          return;
        }
        const auto submitted = controller.SubmitStage(StageId::kDataLoad);
        auto replacement = std::make_shared<FakeRunner>();
        const auto replaced = controller.ReplacePorts(replacement, replacement);
        callback_result_promise.set_value(
            !submitted && !replaced &&
            submitted.GetError().Message().find("event callback") !=
                std::string::npos &&
            replaced.GetError().Message().find("event callback") !=
                std::string::npos);
      });

  const auto job = controller.SubmitNode(NodeId::kDataLoad, Id("A"));
  Check(job && controller.Wait(job.Value()),
        "terminal callback rejection fixture did not complete");
  Check(callback_result.wait_for(std::chrono::milliseconds(500)) ==
            std::future_status::ready &&
            callback_result.get(),
        "worker lifecycle command was not safely rejected in callback");
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
  TestMalformedExecutionReceiptsCannotPublishSuccess();
  TestMalformedReconfigureReceiptResynchronizesAndPoisonsSession();
  TestConcurrentSubmissionsAreSerialized();
  TestRerunInvalidatesDownstream();
  TestFailureStopsPipeline();
  TestBoundaryCancellation();
  TestOptimizerReplay();
  TestNodeCommandsAndMetadata();
  TestConfigApplyInvalidation();
  TestNodeCancellationRollsBackArtifacts();
  TestCancellationTelemetryAndJoinOrdering();
  TestManagedSessionMetadataIsAuthoritative();
  TestSessionRunnerReplacement();
  TestAlignmentFeedbackAcceptAndStaleResponse();
  TestAlignmentFeedbackCanRespondFromRequestCallback();
  TestAlignmentFeedbackCancellation();
  TestSnapshotDoesNotInvokeRunnerWhileControllerLocked();
  TestSnapshotDuringFeedbackNotificationDoesNotDeadlock();
  TestEventCallbackCanUnsubscribeReenterAndThrow();
  TestTerminalCommitPrecedesWaitAndReentrantCallback();
  TestTerminalCallbackRejectsWorkerLifecycleCommands();
  TestControllerCoordinatorFullFallbackIntegration();
  TestControllerCoordinatorTimeoutIntegration();
  std::cout << "pipeline controller tests passed\n";
  return 0;
}
