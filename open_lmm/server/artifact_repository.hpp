#pragma once

#include <open_lmm/server/stage_runner.hpp>

#include <cstdint>
#include <map>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace open_lmm {

enum class ArtifactState : uint8_t { kMissing, kReady, kStale, kFailed };

struct ArtifactKey {
  ArtifactType type;
  std::optional<char> agent;
  auto operator<=>(const ArtifactKey&) const = default;
};

struct ArtifactMetadata {
  ArtifactKey key;
  ArtifactState state = ArtifactState::kMissing;
  uint64_t revision = 0;
  std::string producer;
  std::string detail;
};

class ArtifactRepository {
 public:
  void RegisterAgents(const std::vector<char>& agents);
  void Reset(const std::vector<char>& agents);
  void BeginStage(StageId stage);
  void CompleteStage(StageId stage);
  void FailStage(StageId stage, std::string detail);
  void CompleteOptimizeThrough(char target_agent,
                               const std::vector<char>& ordered_agents);
  Result<void> ValidateNode(NodeId node, std::optional<char> agent) const;
  void BeginNode(NodeId node, std::optional<char> agent);
  void CompleteNode(NodeId node, std::optional<char> agent);
  void FailNode(NodeId node, std::optional<char> agent, std::string detail);
  void ApplyConfig(ConfigDomain domain, uint64_t config_revision);
  [[nodiscard]] ArtifactMetadata Get(const ArtifactKey& key) const;
  [[nodiscard]] std::vector<ArtifactMetadata> Snapshot() const;
  void Restore(const std::vector<ArtifactMetadata>& snapshot);

 private:
  static std::vector<ArtifactType> ProducedBy(StageId stage);
  static bool IsPerAgent(ArtifactType type);
  void setTypes(const std::vector<ArtifactType>& types, ArtifactState state,
                const std::string& producer, const std::string& detail = {});
  void invalidateDownstream(StageId stage);

  mutable std::mutex mutex_;
  std::map<ArtifactKey, ArtifactMetadata> artifacts_;
  std::vector<char> agents_;
  uint64_t next_revision_ = 1;
};

}  // namespace open_lmm
