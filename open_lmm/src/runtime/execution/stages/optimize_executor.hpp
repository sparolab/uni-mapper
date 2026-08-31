#pragma once

#include <functional>

#include <runtime/execution/stages/execution_candidate.hpp>
#include <runtime/execution/stage_ports.hpp>

namespace open_lmm {

class AlgorithmProvider;

class OptimizeExecutor {
 public:
  using OptimizerFactory = std::function<
      Result<std::shared_ptr<BackendOptimizerBase>>(const OptimizerConfig&)>;
  using OptimizeStep = std::function<Result<void>(
      AgentPipelineCtx&, SharedDatabase&,
      const std::shared_ptr<BackendOptimizerBase>&)>;

  explicit OptimizeExecutor(
      std::shared_ptr<const AlgorithmProvider> algorithms);
  OptimizeExecutor(OptimizerFactory optimizer_factory,
                   OptimizeStep optimize_step);

  [[nodiscard]] Result<ExecutionCandidate> ReplayThrough(
      std::shared_ptr<const RuntimeState> committed,
      const AgentId& target_agent, const ExecutionContext& context) const;

 private:
  OptimizerFactory optimizer_factory_;
  OptimizeStep optimize_step_;
  std::shared_ptr<const AlgorithmProvider> algorithms_;
};

}  // namespace open_lmm
