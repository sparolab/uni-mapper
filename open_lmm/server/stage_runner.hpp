#pragma once

#include <open_lmm/common/result.hpp>
#include <open_lmm/common/runtime_contracts.hpp>
#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/config_transaction.hpp>
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

struct CommittedSessionSnapshot {
  uint64_t revision = 0;
  uint64_t config_revision = 0;
  std::vector<AgentId> ordered_agents;
  std::vector<ArtifactMetadata> artifacts;
  std::size_t descriptor_count = 0;
  std::map<AgentId, std::size_t> per_agent_descriptor_count;
};

struct ArtifactExecutionSpec {
  ArtifactType type;
  ExecutionScope ownership;
};

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

[[nodiscard]] const std::vector<NodeExecutionSpec>& ExecutionSpecs();
[[nodiscard]] const std::vector<ArtifactExecutionSpec>& ArtifactExecutionSpecs();
[[nodiscard]] const NodeExecutionSpec& ExecutionSpecFor(NodeId node);
[[nodiscard]] ExecutionScope ArtifactOwnership(ArtifactType artifact);
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
[[nodiscard]] std::vector<AgentId> ArtifactRevisionAffectedAgents(
    const CommittedSessionSnapshot& before,
    const CommittedSessionSnapshot& after);

}  // namespace open_lmm
