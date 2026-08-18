#pragma once

#include <memory>
#include <optional>
#include <vector>

#include <open_lmm/server/runtime_state.hpp>

namespace open_lmm {

enum class ArtifactCompletionKind : uint8_t {
  kDataLoadStage,
  kDataLoadAgent,
  kAlignmentStage,
  kMapUpdateStage,
  kMapUpdateAgent,
  // SaveExecutor prepares the exact PoseSave/FallbackMapSave artifact editor
  // externally because one save command may complete both runtime nodes.
  kSavePrepared,
  kLoopDetectThrough,
  kOptimizeThrough,
};

// A stage executor returns an uncommitted candidate.  Revision advancement,
// artifact mutation and output installation remain one coordinator transaction.
struct ExecutionCandidate {
  uint64_t base_revision = 0;
  std::shared_ptr<const RuntimePayload> payload;
  std::vector<AgentId> execution_agents;
  ArtifactCompletionKind completion = ArtifactCompletionKind::kAlignmentStage;
  std::optional<AgentId> replay_target;
};

}  // namespace open_lmm
