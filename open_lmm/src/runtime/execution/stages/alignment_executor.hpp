#pragma once

#include <functional>

#include <runtime/execution/stages/execution_candidate.hpp>
#include <runtime/execution/stage_ports.hpp>

namespace open_lmm {

class AlgorithmProvider;

class AlignmentExecutor {
 public:
  using OptimizerFactory = std::function<
      Result<std::shared_ptr<BackendOptimizerBase>>(const OptimizerConfig&)>;
  using LoopStep = std::function<Result<void>(AgentPipelineCtx&,
                                               SharedDatabase&)>;
  using OptimizeStep = std::function<Result<void>(
      AgentPipelineCtx&, SharedDatabase&,
      const std::shared_ptr<BackendOptimizerBase>&)>;

  explicit AlignmentExecutor(
      std::shared_ptr<const AlgorithmProvider> algorithms);
  AlignmentExecutor(OptimizerFactory optimizer_factory, LoopStep loop_step,
                    OptimizeStep optimize_step);

  [[nodiscard]] Result<ExecutionCandidate> ExecuteStage(
      std::shared_ptr<const RuntimeState> committed,
      const ExecutionContext& context) const;
  [[nodiscard]] Result<ExecutionCandidate> ReplayLoopDetectThrough(
      std::shared_ptr<const RuntimeState> committed,
      const AgentId& target_agent, const ExecutionContext& context) const;

 private:
  Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizer(
      const RuntimeState& committed) const;
  static Result<void> ValidateBase(const RuntimeState& committed,
                                   const ExecutionContext& context);
  static Result<std::shared_ptr<const RuntimePayload>> BuildPayload(
      const RuntimeState& committed, std::vector<AgentPipelineCtx> contexts,
      std::shared_ptr<SharedDatabase> database,
      std::shared_ptr<BackendOptimizerBase> optimizer);

  OptimizerFactory optimizer_factory_;
  LoopStep loop_step_;
  OptimizeStep optimize_step_;
  std::shared_ptr<const AlgorithmProvider> algorithms_;
};

}  // namespace open_lmm
