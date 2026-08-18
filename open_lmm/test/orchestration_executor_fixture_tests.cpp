#include <open_lmm/server/execution/alignment_executor.hpp>
#include <open_lmm/server/execution/alignment_artifact_store.hpp>
#include <open_lmm/server/execution/optimize_executor.hpp>

#include <cstdlib>
#include <iostream>
#include <set>
#include <string>

namespace open_lmm {
namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

class FakeOptimizer final : public BackendOptimizerBase {
 public:
  Result<BackendOptimizerOutput> Process(
      const AlgorithmExecutionContext&,
      const BackendOptimizerInput&) override {
    return Result<BackendOptimizerOutput>::Ok({});
  }
  void Reset() override {}
  bool HasProcessedAgent(const AgentId&) const override { return false; }
  std::size_t ProcessedAgentCount() const override { return 0; }
};

class EmptyIndex final : public DescriptorIndex {
 public:
  std::size_t getSize() const override { return 0; }
  std::unique_ptr<DescriptorIndex> Clone() const override {
    return std::make_unique<EmptyIndex>();
  }
  void clear() override {}
  void merge(const DescriptorIndex&) override {}
  void insert(AgentId, std::size_t,
              const std::shared_ptr<IDescriptorKdtree>&) override {}
  std::optional<std::tuple<AgentId, std::size_t, Eigen::Isometry3d>> query(
      const std::shared_ptr<IDescriptorKdtree>&) const override {
    return std::nullopt;
  }
  std::vector<std::tuple<AgentId, std::size_t, Eigen::Isometry3d>> queryK(
      const std::shared_ptr<IDescriptorKdtree>&,
      std::size_t) const override {
    return {};
  }
};

std::shared_ptr<const RuntimeState> State() {
  const std::vector<AgentId> agents{Id("A"), Id("B"), Id("C")};
  auto config = std::make_shared<RuntimeConfig>();
  config->loop_detector = std::make_shared<const LoopDetectorConfig>();
  config->optimizer = std::make_shared<const OptimizerConfig>();
  auto database = std::make_shared<SharedDatabase>();
  auto payload = std::make_shared<RuntimePayload>();
  for (std::size_t index = 0; index < agents.size(); ++index) {
    auto raw = std::make_shared<AgentRawData>();
    raw->agent_id = agents[index];
    database->raw_data[agents[index]] = raw;
    AgentPipelineCtx context;
    context.agent = {.id = agents[index],
                     .role = index == 0 ? AgentRole::kAnchor
                                        : AgentRole::kFollower,
                     .order = static_cast<int>(index)};
    context.raw_data = raw;
    auto loop = std::make_shared<LoopDetectorOutput>();
    loop->agent_descriptors = std::make_shared<EmptyIndex>();
    context.loop_output = std::move(loop);
    payload->contexts.push_back(std::move(context));
    payload->resident_memory_reservations[agents[index]] =
        std::make_shared<MemoryReservation>();
  }
  payload->database = database;
  payload->optimizer = std::make_shared<FakeOptimizer>();
  auto state = std::make_shared<RuntimeState>();
  state->revision = 41;
  state->config = std::move(config);
  state->ordered_agents = agents;
  state->payload = std::move(payload);
  return state;
}

auto OptimizerFactory() {
  return [](const OptimizerConfig&) {
    return Result<std::shared_ptr<BackendOptimizerBase>>::Ok(
        std::make_shared<FakeOptimizer>());
  };
}

void TestAlignmentStageUsesCommandContextWithoutCommit() {
  auto state = State();
  std::vector<std::string> calls;
  auto cancellation = std::make_shared<CancellationToken>();
  auto feedback = std::make_shared<AlignmentFeedbackBroker>();
  AlignmentExecutor executor(
      OptimizerFactory(),
      [&](AgentPipelineCtx& context, SharedDatabase& database) {
        Check(context.cancellation == cancellation,
              "command cancellation reaches loop execution");
        Check(database.alignment_feedback == feedback,
              "command feedback reaches alignment database");
        calls.push_back("L" + context.agent.id.Value());
        auto output = std::make_shared<LoopDetectorOutput>();
        output->agent_descriptors = std::make_shared<EmptyIndex>();
        context.loop_output = std::move(output);
        return Result<void>::Ok();
      },
      [&](AgentPipelineCtx& context, SharedDatabase&,
          const std::shared_ptr<BackendOptimizerBase>&) {
        calls.push_back("O" + context.agent.id.Value());
        return Result<void>::Ok();
      });
  auto result = executor.ExecuteStage(
      state, {.cancellation = cancellation,
              .alignment_feedback = feedback,
              .base_revision = state->revision});
  Check(result && result.Value().base_revision == 41,
        "candidate preserves base revision without committing it");
  Check(calls == std::vector<std::string>({"LA", "OA", "LB", "OB", "LC", "OC"}),
        "alignment preserves ordered per-agent node execution");
  Check(result.Value().payload->resident_memory_reservations ==
            state->payload->resident_memory_reservations,
        "alignment candidate retains resident reservation owners");
}

void TestLoopReplayStopsAfterTargetLoop() {
  auto state = State();
  std::vector<std::string> calls;
  AlignmentExecutor executor(
      OptimizerFactory(),
      [&](AgentPipelineCtx& context, SharedDatabase&) {
        calls.push_back("L" + context.agent.id.Value());
        context.loop_output = std::make_shared<LoopDetectorOutput>();
        return Result<void>::Ok();
      },
      [&](AgentPipelineCtx& context, SharedDatabase&,
          const std::shared_ptr<BackendOptimizerBase>&) {
        calls.push_back("O" + context.agent.id.Value());
        return Result<void>::Ok();
      });
  auto replay = executor.ReplayLoopDetectThrough(
      state, Id("B"), {.base_revision = state->revision});
  Check(replay && calls == std::vector<std::string>({"LA", "OA", "LB"}),
        "loop replay optimizes only agents before its target");
  Check(replay.Value().execution_agents ==
            std::vector<AgentId>({Id("A"), Id("B")}),
        "loop replay candidate reports its ordered prefix");
}

void TestOptimizeReplayClearsSuffixAndHonorsCancellation() {
  auto state = State();
  std::vector<AgentId> optimized;
  OptimizeExecutor executor(
      OptimizerFactory(),
      [&](AgentPipelineCtx& context, SharedDatabase&,
          const std::shared_ptr<BackendOptimizerBase>&) {
        optimized.push_back(context.agent.id);
        return Result<void>::Ok();
      });
  auto replay = executor.ReplayThrough(
      state, Id("B"), {.base_revision = state->revision});
  Check(replay && optimized == std::vector<AgentId>({Id("A"), Id("B")}),
        "optimizer replay executes its ordered prefix");
  Check(replay.Value().payload->contexts[0].loop_output &&
            replay.Value().payload->contexts[1].loop_output &&
            !replay.Value().payload->contexts[2].loop_output,
        "optimizer replay invalidates only the suffix loop payload");

  auto cancellation = std::make_shared<CancellationToken>();
  cancellation->Request();
  auto cancelled = executor.ReplayThrough(
      state, Id("A"), {.cancellation = cancellation,
                       .base_revision = state->revision});
  Check(!cancelled &&
            cancelled.GetError().code == Error::Code::kCancelled,
        "optimizer candidate is not published after cancellation");
}

void TestArtifactStoreRehydratesOnlyFromCommittedAuthority() {
  auto mutable_state = std::make_shared<RuntimeState>(*State());
  auto config = std::make_shared<RuntimeConfig>(*mutable_state->config);
  auto metadata = std::make_shared<AlignmentArtifactMetadata>();
  metadata->cache_path = "/tmp/alignment-cache.json";
  metadata->input_fingerprints[Id("A")] = "input-a";
  metadata->runtime_fingerprint = "session";
  config->fingerprint = "config";
  config->alignment_artifacts = metadata;
  mutable_state->config = config;
  auto database = std::make_shared<SharedDatabase>(
      *mutable_state->payload->database);
  StoredAlignment approval;
  approval.proposal.source_agent = Id("B");
  database->stored_alignments[Id("B")] = approval;
  auto payload = std::make_shared<RuntimePayload>(*mutable_state->payload);
  payload->database = database;
  mutable_state->payload = payload;

  auto store = AlignmentArtifactStore::FromCommitted(*mutable_state);
  Check(store && store.Value().Identity().config_fingerprint == "config" &&
            store.Value().Identity().runtime_fingerprint == "session" &&
            store.Value().Cached().contains(Id("B")),
        "artifact store is reconstructed from committed metadata and payload");
}

}  // namespace
}  // namespace open_lmm

int main() {
  open_lmm::TestAlignmentStageUsesCommandContextWithoutCommit();
  open_lmm::TestLoopReplayStopsAfterTargetLoop();
  open_lmm::TestOptimizeReplayClearsSuffixAndHonorsCancellation();
  open_lmm::TestArtifactStoreRehydratesOnlyFromCommittedAuthority();
  std::cout << "orchestration executor fixture tests passed\n";
  return 0;
}
