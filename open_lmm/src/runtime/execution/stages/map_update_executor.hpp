#pragma once

#include <filesystem>
#include <memory>
#include <optional>

#include <runtime/execution/stages/execution_candidate.hpp>
#include <open_lmm/common/algorithm_progress.hpp>
#include <storage/transactions/output_repository.hpp>
#include <runtime/resources/resource_governor.hpp>

namespace open_lmm {

class AlgorithmProvider;

// Stateless MapUpdate invocation. Output paths and scheduling policy are
// explicit command inputs rather than executor-owned runtime mirrors.
struct MapUpdateExecutionContext {
  std::shared_ptr<const RuntimeState> committed;
  std::shared_ptr<ResourceGovernor> governor;
  std::shared_ptr<CancellationToken> cancellation;
  AlgorithmProgressCallback progress;
  std::shared_ptr<const AlgorithmProvider> algorithms;
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
