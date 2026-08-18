#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include <open_lmm/server/execution/execution_candidate.hpp>
#include <open_lmm/server/output_repository.hpp>
#include <open_lmm/server/resource_governor.hpp>

namespace open_lmm {

class AlgorithmFactory;

// Stateless MapUpdate invocation. Output paths and scheduling policy are
// explicit command inputs rather than executor-owned session mirrors.
struct MapUpdateExecutionContext {
  std::shared_ptr<const SessionState> committed;
  std::shared_ptr<ResourceGovernor> governor;
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<const AlgorithmFactory> algorithms;
  std::filesystem::path output_directory;
  double save_voxel_size = 0.2;
  bool parallel = false;
  std::size_t max_parallel_agents = 1;
  PendingOutputSet* pending_files = nullptr;
};

class MapUpdateExecutor {
 public:
  [[nodiscard]] Result<ExecutionCandidate> Execute(
      MapUpdateExecutionContext context) const;
  [[nodiscard]] Result<ExecutionCandidate> ExecuteAgent(
      MapUpdateExecutionContext context, const AgentId& agent) const;

 private:
  [[nodiscard]] Result<ExecutionCandidate> execute(
      MapUpdateExecutionContext context,
      std::optional<AgentId> agent) const;
};

}  // namespace open_lmm
