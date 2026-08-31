#pragma once

#include <map>
#include <functional>
#include <memory>
#include <vector>

#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/common/algorithm_progress.hpp>
#include <open_lmm/common/visualization_snapshot.hpp>
#include <runtime/resources/resource_governor.hpp>
#include <runtime/execution/stages/execution_candidate.hpp>
#include <runtime/state/runtime_state.hpp>

namespace open_lmm {

class AlgorithmProvider;

// All inputs are per-invocation values derived from one committed runtime.
// The executor deliberately owns no config, agent list, or runtime policy.
struct DataLoadExecutionContext {
  std::shared_ptr<const RuntimeState> committed;
  std::vector<AgentPipelineCtx> contexts;
  std::shared_ptr<SharedDatabase> database;
  std::shared_ptr<ResourceGovernor> governor;
  std::shared_ptr<CancellationToken> cancellation;
  AlgorithmProgressCallback progress;
  std::shared_ptr<BackendOptimizerBase> optimizer;
  std::shared_ptr<const AlgorithmProvider> algorithms;
  // Presentation policy supplied by runtime composition. DataLoad consumes a
  // scalar and does not depend on map-server config ownership.
  float preview_voxel_size_m = 0.0F;
  bool parallel = false;
  std::size_t max_parallel_agents = 1;
  // Candidate-only read-model notification. The handle remains owned by the
  // execution candidate and must never be inserted into committed state here.
  std::function<void(const AgentId&, const AgentRawDataHandle&,
                     const VisualizationPointPreviewHandle&)>
      on_agent_loaded;
};

class DataLoadExecutor {
 public:
  [[nodiscard]] Result<ExecutionCandidate> Execute(
      DataLoadExecutionContext context) const;
  [[nodiscard]] Result<ExecutionCandidate> ExecuteAgent(
      DataLoadExecutionContext context, const AgentId& agent) const;
};

}  // namespace open_lmm
