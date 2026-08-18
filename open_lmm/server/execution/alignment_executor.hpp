#pragma once

#include <functional>

#include <open_lmm/server/execution/execution_candidate.hpp>
#include <open_lmm/server/stage_ports.hpp>

namespace open_lmm {

class AlgorithmFactory;

class AlignmentExecutor {
 public:
  using OptimizerFactory = std::function<
      Result<std::shared_ptr<BackendOptimizerBase>>(const OptimizerConfig&)>;
  using LoopStep = std::function<Result<void>(AgentPipelineCtx&,
                                               SharedDatabase&)>;
  using OptimizeStep = std::function<Result<void>(
      AgentPipelineCtx&, SharedDatabase&,
      const std::shared_ptr<BackendOptimizerBase>&)>;

  AlignmentExecutor();
  explicit AlignmentExecutor(
      std::shared_ptr<const AlgorithmFactory> algorithms);
  AlignmentExecutor(OptimizerFactory optimizer_factory, LoopStep loop_step,
                    OptimizeStep optimize_step);

  [[nodiscard]] Result<ExecutionCandidate> ExecuteStage(
      std::shared_ptr<const SessionState> committed,
      const ExecutionContext& context) const;
  [[nodiscard]] Result<ExecutionCandidate> ReplayLoopDetectThrough(
      std::shared_ptr<const SessionState> committed,
      const AgentId& target_agent, const ExecutionContext& context) const;

 private:
  Result<std::shared_ptr<BackendOptimizerBase>> CreateOptimizer(
      const SessionState& committed) const;
  static Result<void> ValidateBase(const SessionState& committed,
                                   const ExecutionContext& context);
  static Result<std::shared_ptr<const SessionPayload>> BuildPayload(
      const SessionState& committed, std::vector<AgentPipelineCtx> contexts,
      std::shared_ptr<SharedDatabase> database,
      std::shared_ptr<BackendOptimizerBase> optimizer);

  OptimizerFactory optimizer_factory_;
  LoopStep loop_step_;
  OptimizeStep optimize_step_;
  std::shared_ptr<const AlgorithmFactory> algorithms_;
};

}  // namespace open_lmm
