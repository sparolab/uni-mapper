#pragma once

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <memory>

#include <cstdint>
#include <compare>
#include <map>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

namespace open_lmm {

enum class StageId : uint8_t { kDataLoad, kAlignment, kMapUpdate, kSave };
enum class NodeId : uint8_t {
  kDataLoad, kLoopDetect, kOptimize, kMapUpdate, kPoseSave
};
enum class ArtifactType : uint8_t {
  kConfigSnapshot, kAgentInput, kRawData, kDescriptorState,
  kLoopCandidates, kMapAlignment, kOptimizerState, kOptimizedPoses, kGlobalMap,
  kPoseFile, kPcdFile, kProfileRecord,
};
enum class ConfigDomain : uint8_t {
  kGlobal, kDataLoader, kLoopDetector, kOptimizer, kDynamicRemover,
  kMapSave,
};
enum class ArtifactState : uint8_t { kMissing, kReady, kStale, kFailed };

struct ArtifactKey {
  ArtifactType type;
  std::optional<AgentId> agent;
  auto operator<=>(const ArtifactKey&) const = default;
};

struct ArtifactMetadata {
  ArtifactKey key;
  ArtifactState state = ArtifactState::kMissing;
  uint64_t revision = 0;
  std::string producer;
  std::string detail;
  std::string external_path;
  std::string fingerprint;
};

struct CommittedSessionSnapshot {
  uint64_t revision = 0;
  uint64_t config_revision = 0;
  std::vector<AgentId> ordered_agents;
  std::vector<ArtifactMetadata> artifacts;
  std::size_t descriptor_count = 0;
  std::map<AgentId, std::size_t> per_agent_descriptor_count;
};

enum class ExecutionScope : uint8_t { kPerAgent, kSession };

struct NodeExecutionSpec {
  NodeId id;
  std::string_view name;
  StageId stage;
  ExecutionScope scope;
  bool ordered = false;
  bool supports_cancellation = false;
  std::vector<ArtifactType> required_artifacts;
  std::vector<ArtifactType> produces;
  std::vector<ArtifactType> invalidates;
};

struct NodeDescriptor {
  NodeId id;
  std::string_view name;
  StageId stage;
  std::vector<ArtifactType> required_artifacts;
  std::vector<ArtifactType> produced_artifacts;
  bool ordered = false;
  bool supports_cancellation = false;
};

[[nodiscard]] const std::vector<NodeExecutionSpec>& ExecutionSpecs();
[[nodiscard]] const NodeExecutionSpec& ExecutionSpecFor(NodeId node);
[[nodiscard]] const NodeDescriptor& DescribeNode(NodeId node);
[[nodiscard]] const std::vector<NodeId>& PipelineNodes();
[[nodiscard]] const std::vector<StageId>& PipelineStages();
[[nodiscard]] std::vector<NodeId> StageNodes(StageId stage);
[[nodiscard]] Result<std::vector<AgentId>> OrderedAgentPrefix(
    const std::vector<AgentId>& ordered_agents, const AgentId& target_agent);
[[nodiscard]] std::size_t ProgressTotal(
    NodeId node, const std::vector<AgentId>& ordered_agents,
    std::optional<AgentId> target_agent);
[[nodiscard]] std::vector<ArtifactType> ProducedArtifacts(StageId stage);
[[nodiscard]] std::vector<ArtifactType> AffectedArtifacts(NodeId node);
[[nodiscard]] std::vector<ArtifactType> AffectedArtifacts(StageId stage);
[[nodiscard]] std::vector<ArtifactType> AffectedArtifacts(ConfigDomain domain);

class StageRunner {
 public:
  virtual ~StageRunner() = default;
  virtual void SetCancellationToken(std::shared_ptr<CancellationToken> token) = 0;
  [[nodiscard]] virtual CancellationCapability CancellationMetadata() const {
    return {};
  }
  virtual void SetAlignmentFeedbackBroker(
      std::shared_ptr<AlignmentFeedbackBroker>) {}
  virtual Result<void> RunStage(StageId stage) = 0;
  virtual Result<void> RunNode(NodeId node, std::optional<AgentId> agent) = 0;
  virtual Result<void> RunOptimizeThrough(const AgentId& target_agent) = 0;
  virtual Result<void> Reconfigure(ConfigDomain domain,
                                   uint64_t revision = 0) {
    (void)domain;
    (void)revision;
    return Result<void>::Ok();
  }
  [[nodiscard]] virtual std::vector<AgentId> AgentIds() const = 0;
  [[nodiscard]] virtual std::optional<CommittedSessionSnapshot>
  SessionSnapshot() const {
    return std::nullopt;
  }
  [[nodiscard]] virtual Result<VisualizationSnapshot>
  CreateVisualizationSnapshot(const AgentId&) const {
    return Result<VisualizationSnapshot>::Failure(
        Error::InvalidArgument("visualization snapshots are not supported"));
  }
};

}  // namespace open_lmm
