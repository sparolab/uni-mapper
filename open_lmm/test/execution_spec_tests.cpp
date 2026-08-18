#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/server/artifact_repository.hpp>
#include <open_lmm/server/stage_runner.hpp>

#include <algorithm>
#include <cstdlib>
#include <iostream>
#include <memory>

namespace open_lmm {
namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

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
  void insert(char, std::size_t,
              const std::shared_ptr<IDescriptorKdtree>&) override {
    ++size_;
  }
  std::optional<std::tuple<char, std::size_t, Eigen::Isometry3d>> query(
      const std::shared_ptr<IDescriptorKdtree>&) const override {
    return std::nullopt;
  }
  std::vector<std::tuple<char, std::size_t, Eigen::Isometry3d>> queryK(
      const std::shared_ptr<IDescriptorKdtree>&,
      std::size_t) const override {
    return {};
  }

 private:
  std::size_t size_ = 0;
};

DescriptorIndexHandle MakeDatabase(char agent, std::size_t count,
                                   double offset) {
  (void)agent;
  (void)offset;
  return std::make_shared<TestDescriptorIndex>(count);
}

ArtifactState StateOf(const std::vector<ArtifactMetadata>& artifacts,
                      ArtifactType type,
                      std::optional<char> agent = std::nullopt) {
  for (const auto& artifact : artifacts) {
    if (artifact.key == ArtifactKey{type, agent}) return artifact.state;
  }
  return ArtifactState::kMissing;
}

void TestExecutionSpecIsSingleOrderedSource() {
  Check(ExecutionSpecs().size() == 5, "five node specs registered");
  Check(PipelineNodes() ==
            std::vector<NodeId>({NodeId::kDataLoad, NodeId::kLoopDetect,
                                 NodeId::kOptimize, NodeId::kMapUpdate,
                                 NodeId::kPoseSave}),
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
  Check(ProgressTotal(NodeId::kLoopDetect, {'A', 'B', 'C'}, 'B') == 2,
        "ordered progress total uses replay prefix");
}

void TestOrderedValidationUsesRawAndLoopPrefix() {
  ArtifactRepository artifacts;
  artifacts.Reset({'A', 'B'});
  artifacts.BeginStage(StageId::kDataLoad);
  artifacts.CompleteStage(StageId::kDataLoad);
  Check(static_cast<bool>(artifacts.ValidateNode(NodeId::kLoopDetect, 'B')),
        "LoopDetect(B) derives A..B replay dependencies from raw prefix");
  artifacts.CompleteLoopDetectThrough('B', {'A', 'B'});
  Check(static_cast<bool>(artifacts.ValidateNode(NodeId::kOptimize, 'B')),
        "Optimize(B) validates A..B loop prefix");
  const auto snapshot = artifacts.Snapshot();
  Check(StateOf(snapshot, ArtifactType::kLoopCandidates, 'A') ==
            ArtifactState::kReady &&
            StateOf(snapshot, ArtifactType::kLoopCandidates, 'B') ==
                ArtifactState::kReady,
        "loop replay marks the committed prefix ready");
  Check(StateOf(snapshot, ArtifactType::kOptimizedPoses, 'A') ==
            ArtifactState::kReady &&
            StateOf(snapshot, ArtifactType::kOptimizedPoses, 'B') ==
                ArtifactState::kStale,
        "loop replay metadata matches the actual optimizer prefix");

  ArtifactRepository three_agents;
  three_agents.Reset({'A', 'B', 'C'});
  three_agents.BeginStage(StageId::kDataLoad);
  three_agents.CompleteStage(StageId::kDataLoad);
  three_agents.CompleteLoopDetectThrough('C', {'A', 'B', 'C'});
  three_agents.CompleteOptimizeThrough('B', {'A', 'B', 'C'});
  const auto middle_replay = three_agents.Snapshot();
  Check(StateOf(middle_replay, ArtifactType::kLoopCandidates, 'B') ==
            ArtifactState::kReady &&
            StateOf(middle_replay, ArtifactType::kLoopCandidates, 'C') ==
                ArtifactState::kStale &&
            StateOf(middle_replay, ArtifactType::kOptimizedPoses, 'C') ==
                ArtifactState::kStale,
        "middle optimizer replay stales follower loop and pose artifacts");
}

void TestDescriptorStoreRebuildReplacesRepeatedAgent() {
  DescriptorStore store;
  const auto anchor = MakeDatabase('A', 2, 0.0);
  const auto follower = MakeDatabase('B', 3, 10.0);
  store.set_anchor_descriptor('A', anchor);
  store.merge_descriptor_db('B', follower);
  Check(store.total_db && store.total_db->getSize() == 5,
        "descriptor store combines ordered per-agent outputs");
  Check(store.per_agent_db.size() == 2 &&
            store.descriptor_order == std::vector<char>({'A', 'B'}),
        "descriptor store retains reconstructable per-agent outputs");

  const auto repeated_follower = MakeDatabase('B', 3, 20.0);
  store.merge_descriptor_db('B', repeated_follower);
  Check(store.total_db && store.total_db->getSize() == 5,
        "repeated LoopDetect(B) replaces rather than appends descriptors");
  store.rebuild_descriptor_db();
  Check(store.total_db && store.total_db->getSize() == 5,
        "descriptor rebuild is deterministic from per-agent outputs");
}

}  // namespace
}  // namespace open_lmm

int main() {
  open_lmm::TestExecutionSpecIsSingleOrderedSource();
  std::cout << "execution spec registry passed\n";
  open_lmm::TestOrderedValidationUsesRawAndLoopPrefix();
  std::cout << "ordered artifact validation passed\n";
  open_lmm::TestDescriptorStoreRebuildReplacesRepeatedAgent();
  std::cout << "execution spec tests passed\n";
  return 0;
}
