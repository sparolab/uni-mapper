#pragma once
#include <memory>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/core/backend_optimizer/backend_optimizer_base.hpp>

namespace open_lmm {

// BackendOptimizer는 에이전트 간 누적 팩터 그래프 상태를 유지하므로
// MapServer에서 단 하나의 인스턴스를 공유한다.
class OptimizeNode : public PipelineNodeBase {
 public:
  explicit OptimizeNode(std::shared_ptr<BackendOptimizerBase> optimizer)
      : optimizer_(std::move(optimizer)) {}

  Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                               SharedDatabase&   db) override {
    if (!ctx.raw_data || !ctx.loop_output) {
      return Result<ControlFlow>::Ok(ControlFlow::kSkip);
    }

    auto loops_valid = ValidateLoops(ctx, db);
    if (!loops_valid) return Result<ControlFlow>::Failure(loops_valid.GetError());

    std::map<char, AgentOptimizedData> all_opt;
    try {
      all_opt = optimizer_->Process(
          ctx.agent, *ctx.raw_data, ctx.loop_output->intra_loops,
          ctx.loop_output->inter_loops, db.raw_data);
    } catch (const std::exception& e) {
      return Result<ControlFlow>::Failure(Error::OptimizationFailed(
          "agent " + std::string{ctx.agent.id} + ": " + e.what()));
    }

    for (auto& [id, opt] : all_opt) {
      db.optimized_data[id] = std::move(opt);
    }

    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }

  const char* Name() const override { return "Optimize"; }

 private:
  static Result<void> ValidateLoops(const AgentPipelineCtx& ctx,
                                    const SharedDatabase& db) {
    const auto validate = [&](const LoopPair& loop) -> Result<void> {
      if (loop.from.first != ctx.agent.id ||
          loop.from.second >= ctx.raw_data->filtered_scans.size()) {
        return Result<void>::Failure(Error::InvalidArgument(
            "Loop source agent or scan index is invalid"));
      }
      const auto target = db.raw_data.find(loop.to.first);
      if (target == db.raw_data.end() ||
          loop.to.second >= target->second.filtered_scans.size() ||
          loop.to.second >= target->second.odom_poses.size()) {
        return Result<void>::Failure(Error::InvalidArgument(
            "Loop target agent or scan index is invalid"));
      }
      return Result<void>::Ok();
    };
    for (const auto& loop : ctx.loop_output->intra_loops) {
      auto result = validate(loop);
      if (!result) return result;
    }
    for (const auto& loop : ctx.loop_output->inter_loops) {
      auto result = validate(loop);
      if (!result) return result;
    }
    return Result<void>::Ok();
  }

  std::shared_ptr<BackendOptimizerBase> optimizer_;
};

}  // namespace open_lmm
