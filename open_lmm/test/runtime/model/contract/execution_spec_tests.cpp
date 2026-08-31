#include <open_lmm/common/agent_data.hpp>
#include <runtime/state/artifact_repository.hpp>
#include <runtime/model/execution_spec.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>
#include <string_view>

namespace open_lmm {
namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

AgentId Id(const char* value) { return AgentId::Parse(value).Value(); }

class TestDescriptorIndex final : public DescriptorIndex {
 public:
  explicit TestDescriptorIndex(std::size_t size) : size_(size) {}

  std::size_t getSize() const override { return size_; }
  std::unique_ptr<DescriptorIndex> Clone() const override {
    return std::make_unique<TestDescriptorIndex>(*this);
  }
  void clear() override { size_ = 0; }
  void merge(const DescriptorIndex& other) override {
    const auto* typed = dynamic_cast<const TestDescriptorIndex*>(&other);
    Check(typed != nullptr, "descriptor store merges compatible index types");
    size_ += typed->size_;
  }
  void insert(AgentId, std::size_t,
              const std::shared_ptr<IDescriptorKdtree>&) override {
    ++size_;
  }
  std::optional<std::tuple<AgentId, std::size_t, Eigen::Isometry3d>> query(
      const std::shared_ptr<IDescriptorKdtree>&) const override {
    return std::nullopt;
  }
  std::vector<std::tuple<AgentId, std::size_t, Eigen::Isometry3d>> queryK(
      const std::shared_ptr<IDescriptorKdtree>&,
      std::size_t) const override {
    return {};
  }

 private:
  std::size_t size_ = 0;
};

DescriptorIndexHandle MakeDatabase(AgentId agent, std::size_t count,
                                   double offset) {
  (void)agent;
  (void)offset;
  return std::make_shared<TestDescriptorIndex>(count);
}

ArtifactState StateOf(const std::vector<ArtifactMetadata>& artifacts,
                      ArtifactType type,
                      std::optional<AgentId> agent = std::nullopt) {
  for (const auto& artifact : artifacts) {
    if (artifact.key == ArtifactKey{type, agent}) return artifact.state;
  }
  return ArtifactState::kMissing;
}

void TestExecutionSpecIsSingleOrderedSource() {
  Check(ExecutionSpecs().size() == 6, "six node specs registered");
  Check(PipelineNodes() ==
            std::vector<NodeId>({NodeId::kDataLoad, NodeId::kLoopDetect,
                                 NodeId::kOptimize, NodeId::kMapUpdate,
                                 NodeId::kPoseSave,
                                 NodeId::kFallbackMapSave}),
        "pipeline node order derives from execution spec");
  Check(PipelineStages() ==
            std::vector<StageId>({StageId::kDataLoad, StageId::kAlignment,
                                  StageId::kMapUpdate, StageId::kSave}),
        "pipeline stage order derives from execution spec");
  Check(StageNodes(StageId::kAlignment) ==
            std::vector<NodeId>({NodeId::kLoopDetect, NodeId::kOptimize}),
        "alignment assembly derives from execution spec");
  const auto& loop = ExecutionSpecFor(NodeId::kLoopDetect);
  Check(loop.ordered &&
            std::find(loop.invalidates.begin(), loop.invalidates.end(),
                      ArtifactType::kOptimizedPoses) != loop.invalidates.end(),
        "ordered/invalidation policy lives in execution spec");
  const auto& optimize = ExecutionSpecFor(NodeId::kOptimize);
  Check(std::find(optimize.invalidates.begin(), optimize.invalidates.end(),
                  ArtifactType::kLoopCandidates) != optimize.invalidates.end(),
        "optimizer replay invalidates follower loop outputs");
  const auto optimizer_config_artifacts =
      AffectedArtifacts(ConfigDomain::kOptimizer);
  Check(std::find(optimizer_config_artifacts.begin(),
                  optimizer_config_artifacts.end(),
                  ArtifactType::kLoopCandidates) !=
                optimizer_config_artifacts.end() &&
            std::find(optimizer_config_artifacts.begin(),
                      optimizer_config_artifacts.end(),
                      ArtifactType::kDescriptorState) ==
                optimizer_config_artifacts.end() &&
            std::find(optimizer_config_artifacts.begin(),
                      optimizer_config_artifacts.end(),
                      ArtifactType::kMapAlignment) ==
                optimizer_config_artifacts.end(),
        "optimizer config invalidates loop candidates without staling "
        "descriptor or map-alignment state");
  Check(ProgressTotal(NodeId::kLoopDetect, {Id("A"), Id("B"), Id("C")},
                      Id("B")) == 2,
        "ordered progress total uses replay prefix");
  const auto& pose_save = ExecutionSpecFor(NodeId::kPoseSave);
  Check(pose_save.scope == ExecutionScope::kRuntime,
        "PoseSave is a session-scoped file-set operation");
  const auto& fallback_map = ExecutionSpecFor(NodeId::kFallbackMapSave);
  Check(fallback_map.scope == ExecutionScope::kRuntime &&
            fallback_map.required_artifacts ==
                std::vector<ArtifactType>({ArtifactType::kRawData,
                                           ArtifactType::kOptimizedPoses}) &&
            fallback_map.produces ==
                std::vector<ArtifactType>({ArtifactType::kGlobalMap,
                                           ArtifactType::kPcdFile}),
        "fallback map file-set has an explicit session execution spec");
  Check(ArtifactExecutionSpecs().size() == 12 &&
            ArtifactOwnership(ArtifactType::kDescriptorState) ==
                ExecutionScope::kRuntime &&
            ArtifactOwnership(ArtifactType::kPoseFile) ==
                ExecutionScope::kPerAgent,
        "artifact ownership is declared by the registry");
}

void TestArtifactRevisionDiffDefinesAffectedAgents() {
  ArtifactRepository artifacts;
  const std::vector<AgentId> agents{Id("A"), Id("B"), Id("C")};
  artifacts.Reset(agents);
  CommittedRuntimeSnapshot before;
  before.ordered_agents = agents;
  before.artifacts = artifacts.Snapshot();

  auto after = before;
  for (auto& artifact : after.artifacts) {
    if (artifact.key == ArtifactKey{ArtifactType::kRawData, Id("B")}) {
      ++artifact.revision;
    }
  }
  Check(ArtifactRevisionAffectedAgents(before, after) ==
            std::vector<AgentId>{Id("B")},
        "per-agent artifact revision diff reports its exact owner");

  after = before;
  for (auto& artifact : after.artifacts) {
    if (artifact.key ==
        ArtifactKey{ArtifactType::kConfigSnapshot, std::nullopt}) {
      ++artifact.revision;
    }
  }
  Check(ArtifactRevisionAffectedAgents(before, after) == agents,
        "session artifact revision diff expands to every ordered agent");
}

void TestUnorderedPerAgentNodeInvalidatesOnlyItsTarget() {
  ArtifactRepository artifacts;
  const std::vector<AgentId> agents{Id("A"), Id("B"), Id("C")};
  artifacts.Reset(agents);
  artifacts.BeginStage(StageId::kDataLoad);
  artifacts.CompleteStage(StageId::kDataLoad);
  artifacts.CompleteLoopDetectThrough(Id("C"), agents);
  artifacts.CompleteOptimizeThrough(Id("C"), agents);
  artifacts.BeginStage(StageId::kMapUpdate);
  artifacts.CompleteStage(StageId::kMapUpdate);
  const auto before = artifacts.Snapshot();

  artifacts.BeginNode(NodeId::kMapUpdate, Id("B"));
  artifacts.CompleteNode(NodeId::kMapUpdate, Id("B"));
  const auto after = artifacts.Snapshot();
  CommittedRuntimeSnapshot before_session;
  before_session.ordered_agents = agents;
  before_session.artifacts = before;
  CommittedRuntimeSnapshot after_session;
  after_session.ordered_agents = agents;
  after_session.artifacts = after;
  Check(ArtifactRevisionAffectedAgents(before_session, after_session) ==
            std::vector<AgentId>{Id("B")},
        "unordered per-agent MapUpdate changes only its requested owner");
  Check(StateOf(after, ArtifactType::kPcdFile, Id("A")) ==
                ArtifactState::kReady &&
            StateOf(after, ArtifactType::kPcdFile, Id("C")) ==
                ArtifactState::kReady,
        "unordered MapUpdate preserves neighboring map artifacts");
}

void TestPoseSaveUsesAllAndOnlyReadyAgents() {
  ArtifactRepository artifacts;
  const std::vector<AgentId> agents{Id("A"), Id("B"), Id("C")};
  artifacts.Reset(agents);
  artifacts.BeginStage(StageId::kDataLoad);
  artifacts.CompleteStage(StageId::kDataLoad);
  artifacts.CompleteLoopDetectThrough(Id("C"), agents);
  artifacts.CompleteOptimizeThrough(Id("B"), agents);

  auto affected = artifacts.ExecutionAgents(NodeId::kPoseSave, Id("C"));
  Check(affected && affected.Value() ==
                        std::vector<AgentId>({Id("A"), Id("B")}),
        "session PoseSave ignores a target and selects every ready pose agent");
  Check(static_cast<bool>(
            artifacts.ValidateNode(NodeId::kPoseSave, std::nullopt)),
        "session PoseSave does not require a command target");

  artifacts.BeginNode(NodeId::kPoseSave, affected.Value());
  artifacts.CompleteNode(NodeId::kPoseSave, affected.Value());
  const auto snapshot = artifacts.Snapshot();
  Check(StateOf(snapshot, ArtifactType::kPoseFile, Id("A")) ==
                ArtifactState::kReady &&
            StateOf(snapshot, ArtifactType::kPoseFile, Id("B")) ==
                ArtifactState::kReady &&
            StateOf(snapshot, ArtifactType::kPoseFile, Id("C")) !=
                ArtifactState::kReady,
        "PoseSave artifact ownership matches its actual file-set");
}

void TestOrderedValidationUsesRawAndLoopPrefix() {
  ArtifactRepository artifacts;
  artifacts.Reset({Id("A"), Id("B")});
  artifacts.BeginStage(StageId::kDataLoad);
  artifacts.CompleteStage(StageId::kDataLoad);
  Check(static_cast<bool>(artifacts.ValidateNode(NodeId::kLoopDetect, Id("B"))),
        "LoopDetect(B) derives A..B replay dependencies from raw prefix");
  artifacts.CompleteLoopDetectThrough(Id("B"), {Id("A"), Id("B")});
  Check(static_cast<bool>(artifacts.ValidateNode(NodeId::kOptimize, Id("B"))),
        "Optimize(B) validates A..B loop prefix");
  const auto snapshot = artifacts.Snapshot();
  Check(StateOf(snapshot, ArtifactType::kLoopCandidates, Id("A")) ==
            ArtifactState::kReady &&
            StateOf(snapshot, ArtifactType::kLoopCandidates, Id("B")) ==
                ArtifactState::kReady,
        "loop replay marks the committed prefix ready");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, Id("A")) ==
            ArtifactState::kReady &&
            StateOf(snapshot, ArtifactType::kOptimizedPoses, Id("B")) ==
                ArtifactState::kStale,
        "loop replay metadata matches the actual optimizer prefix");

  ArtifactRepository three_agents;
  three_agents.Reset({Id("A"), Id("B"), Id("C")});
  three_agents.BeginStage(StageId::kDataLoad);
  three_agents.CompleteStage(StageId::kDataLoad);
  three_agents.CompleteLoopDetectThrough(Id("C"), {Id("A"), Id("B"), Id("C")});
  three_agents.CompleteOptimizeThrough(Id("B"), {Id("A"), Id("B"), Id("C")});
  const auto middle_replay = three_agents.Snapshot();
  Check(StateOf(middle_replay, ArtifactType::kLoopCandidates, Id("B")) ==
            ArtifactState::kReady &&
            StateOf(middle_replay, ArtifactType::kLoopCandidates, Id("C")) ==
                ArtifactState::kStale &&
            StateOf(middle_replay, ArtifactType::kOptimizedPoses, Id("C")) ==
                ArtifactState::kStale,
        "middle optimizer replay stales follower loop and pose artifacts");
}

void TestDescriptorStoreRebuildReplacesRepeatedAgent() {
  DescriptorStore store;
  const auto anchor = MakeDatabase(Id("A"), 2, 0.0);
  const auto follower = MakeDatabase(Id("B"), 3, 10.0);
  store.set_anchor_descriptor(Id("A"), anchor);
  store.merge_descriptor_db(Id("B"), follower);
  Check(store.total_db && store.total_db->getSize() == 5,
        "descriptor store combines ordered per-agent outputs");
  Check(store.per_agent_db.size() == 2 &&
            store.descriptor_order == std::vector<AgentId>({Id("A"), Id("B")}),
        "descriptor store retains reconstructable per-agent outputs");

  const auto repeated_follower = MakeDatabase(Id("B"), 3, 20.0);
  store.merge_descriptor_db(Id("B"), repeated_follower);
  Check(store.total_db && store.total_db->getSize() == 5,
        "repeated LoopDetect(B) replaces rather than appends descriptors");
  store.rebuild_descriptor_db();
  Check(store.total_db && store.total_db->getSize() == 5,
        "descriptor rebuild is deterministic from per-agent outputs");
}

}  // namespace
}  // namespace open_lmm

int main(int argc, char** argv) {
  if (argc != 3 || std::string_view(argv[1]) != "--suite") {
    std::cerr << "usage: execution_spec_tests --suite 1|2\n";
    return 2;
  }
  const std::string_view suite(argv[2]);
  if (suite == "1") {
  open_lmm::TestExecutionSpecIsSingleOrderedSource();
  std::cout << "execution spec registry passed\n";
  } else if (suite == "2") {
  open_lmm::TestOrderedValidationUsesRawAndLoopPrefix();
  std::cout << "ordered artifact validation passed\n";
  open_lmm::TestPoseSaveUsesAllAndOnlyReadyAgents();
  std::cout << "session PoseSave scope passed\n";
  open_lmm::TestArtifactRevisionDiffDefinesAffectedAgents();
  std::cout << "artifact revision affected-agent diff passed\n";
  open_lmm::TestUnorderedPerAgentNodeInvalidatesOnlyItsTarget();
  std::cout << "unordered per-agent invalidation passed\n";
  open_lmm::TestDescriptorStoreRebuildReplacesRepeatedAgent();
  std::cout << "artifact repository tests passed\n";
  } else {
    std::cerr << "unknown execution spec suite: " << suite << '\n';
    return 2;
  }
  return 0;
}
