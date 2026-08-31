#pragma once

#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/result.hpp>
#include <runtime/state/artifact_repository.hpp>
#include <storage/transactions/output_repository.hpp>
#include <runtime/state/runtime_state.hpp>

#include <filesystem>
#include <memory>
#include <vector>

namespace open_lmm {

enum class SaveExecutionMode : uint8_t {
  kStage,
  kPoseSave,
  kFallbackMapSave,
};

struct SaveExecutionRequest {
  const RuntimeState& state;
  SaveExecutionMode mode = SaveExecutionMode::kStage;
  std::filesystem::path output_directory;
  bool map_update_enabled = false;
  std::shared_ptr<CancellationToken> cancellation;
};

struct SaveExecutionSummary {
  std::vector<AgentId> pose_agents;
  std::vector<AgentId> fallback_map_agents;
  bool fallback_map_skipped = false;
};

// Prepares a full Save stage or one explicitly requested Save node into a
// caller-owned PendingOutputSet. The coordinator remains responsible for
// committing the candidate RuntimeState and the complete file set.
class SaveExecutor {
 public:
  Result<SaveExecutionSummary> Prepare(
      const SaveExecutionRequest& request, PendingOutputSet& pending,
      ArtifactRepository& artifacts) const;

 private:
  Result<void> PrepareOptimizedPoses(
      const SaveExecutionRequest& request,
      const std::vector<AgentId>& affected_agents, PendingOutputSet& pending,
      ArtifactRepository& artifacts) const;
  Result<void> PrepareFallbackMaps(
      const SaveExecutionRequest& request,
      const std::vector<AgentId>& affected_agents, PendingOutputSet& pending,
      ArtifactRepository& artifacts) const;
};

}  // namespace open_lmm
