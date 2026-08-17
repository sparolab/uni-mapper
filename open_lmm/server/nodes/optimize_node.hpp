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

    auto all_opt = optimizer_->Process(
        ctx.agent, *ctx.raw_data,
        ctx.loop_output->intra_loops,
        ctx.loop_output->inter_loops,
        db.raw_data);

    for (auto& [id, opt] : all_opt) {
      db.optimized_data[id] = std::move(opt);
    }

    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }

  const char* Name() const override { return "Optimize"; }

 private:
  std::shared_ptr<BackendOptimizerBase> optimizer_;
};

}  // namespace open_lmm
