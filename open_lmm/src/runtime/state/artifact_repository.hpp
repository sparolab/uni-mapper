#pragma once

#include <runtime/execution/stage_runner.hpp>

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm {

class ArtifactRepository {
 public:
  void RegisterAgents(const std::vector<AgentId>& agents);
  void Reset(const std::vector<AgentId>& agents);
  void BeginStage(StageId stage);
  void CompleteStage(StageId stage);
  void FailStage(StageId stage, std::string detail);
  void CompleteOptimizeThrough(const AgentId& target_agent,
                               const std::vector<AgentId>& ordered_agents);
  void CompleteLoopDetectThrough(const AgentId& target_agent,
                                 const std::vector<AgentId>& ordered_agents);
  Result<void> ValidateNode(NodeId node, std::optional<AgentId> agent) const;
  [[nodiscard]] Result<std::vector<AgentId>> ExecutionAgents(
      NodeId node, std::optional<AgentId> agent) const;
  void BeginNode(NodeId node, std::optional<AgentId> agent);
  void BeginNode(NodeId node, const std::vector<AgentId>& affected_agents);
  void CompleteNode(NodeId node, std::optional<AgentId> agent);
  void CompleteNode(NodeId node, const std::vector<AgentId>& affected_agents);
  void FailNode(NodeId node, std::optional<AgentId> agent, std::string detail);
  void FailNode(NodeId node, const std::vector<AgentId>& affected_agents,
                std::string detail);
  void ApplyConfig(ConfigDomain domain, uint64_t config_revision);
  void RecordExternalFile(ArtifactType type, AgentId agent,
                          std::string path, std::string fingerprint);
  [[nodiscard]] std::vector<ArtifactMetadata> Snapshot() const;
  void Restore(const std::vector<ArtifactMetadata>& snapshot);

 private:
  static bool IsPerAgent(ArtifactType type);
  [[nodiscard]] Result<std::vector<AgentId>> executionAgentsLocked(
      NodeId node, std::optional<AgentId> agent) const;
  void beginNodeLocked(NodeId node,
                       const std::vector<AgentId>& affected_agents);
  void completeNodeLocked(NodeId node,
                          const std::vector<AgentId>& affected_agents,
                          ArtifactState state, const std::string& detail);
  void setTypes(const std::vector<ArtifactType>& types, ArtifactState state,
                const std::string& producer, const std::string& detail = {});
  void invalidateDownstream(StageId stage);

  mutable std::mutex mutex_;
  std::map<ArtifactKey, ArtifactMetadata> artifacts_;
  std::vector<AgentId> agents_;
  uint64_t next_revision_ = 1;
};

}  // namespace open_lmm
