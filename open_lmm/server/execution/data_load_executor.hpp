#pragma once

#include <map>
#include <memory>
#include <vector>

#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/server/resource_governor.hpp>
#include <open_lmm/server/execution/execution_candidate.hpp>
#include <open_lmm/server/runtime_state.hpp>

namespace open_lmm {

class AlgorithmFactory;

// All inputs are per-invocation values derived from one committed runtime.
// The executor deliberately owns no config, agent list, or runtime policy.
struct DataLoadExecutionContext {
  std::shared_ptr<const RuntimeState> committed;
  std::vector<AgentPipelineCtx> contexts;
  std::shared_ptr<SharedDatabase> database;
  std::shared_ptr<ResourceGovernor> governor;
  std::shared_ptr<CancellationToken> cancellation;
  std::shared_ptr<BackendOptimizerBase> optimizer;
  std::shared_ptr<const AlgorithmFactory> algorithms;
  bool parallel = false;
  std::size_t max_parallel_agents = 1;
};

class DataLoadExecutor {
 public:
  [[nodiscard]] Result<ExecutionCandidate> Execute(
      DataLoadExecutionContext context) const;
  [[nodiscard]] Result<ExecutionCandidate> ExecuteAgent(
      DataLoadExecutionContext context, const AgentId& agent) const;
};

}  // namespace open_lmm
