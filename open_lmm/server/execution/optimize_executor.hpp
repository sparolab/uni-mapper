#pragma once

#include <functional>

#include <open_lmm/server/execution/execution_candidate.hpp>
#include <open_lmm/server/stage_ports.hpp>

namespace open_lmm {

class AlgorithmFactory;

class OptimizeExecutor {
 public:
  using OptimizerFactory = std::function<
      Result<std::shared_ptr<BackendOptimizerBase>>(const OptimizerConfig&)>;
  using OptimizeStep = std::function<Result<void>(
      AgentPipelineCtx&, SharedDatabase&,
      const std::shared_ptr<BackendOptimizerBase>&)>;

  OptimizeExecutor();
  explicit OptimizeExecutor(
      std::shared_ptr<const AlgorithmFactory> algorithms);
  OptimizeExecutor(OptimizerFactory optimizer_factory,
                   OptimizeStep optimize_step);

  [[nodiscard]] Result<ExecutionCandidate> ReplayThrough(
      std::shared_ptr<const SessionState> committed,
      const AgentId& target_agent, const ExecutionContext& context) const;

 private:
  OptimizerFactory optimizer_factory_;
  OptimizeStep optimize_step_;
  std::shared_ptr<const AlgorithmFactory> algorithms_;
};

}  // namespace open_lmm
