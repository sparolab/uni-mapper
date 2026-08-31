#pragma once
#include <functional>
#include <memory>
#include <runtime/execution/pipeline.hpp>
#include <domain/optimization/backend_optimizer_base.hpp>

namespace open_lmm {

// BackendOptimizer는 에이전트 간 누적 팩터 그래프 상태를 유지하므로
// MapServer에서 단 하나의 인스턴스를 공유한다.
class OptimizeNode : public PipelineNodeBase {
 public:
  using PreviewCallback =
      std::function<void(const AgentPipelineCtx&, const SharedDatabase&)>;

  explicit OptimizeNode(std::shared_ptr<BackendOptimizerBase> optimizer)
      : optimizer_(std::move(optimizer)) {}
  OptimizeNode(std::shared_ptr<BackendOptimizerBase> optimizer,
               AlgorithmExecutionContext algorithm_context,
               PreviewCallback preview = {})
      : optimizer_(std::move(optimizer)),
        algorithm_context_(std::move(algorithm_context)),
        preview_(std::move(preview)) {}

  Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                               SharedDatabase&   db) override {
    if (!ctx.raw_data || !ctx.loop_output) {
      return Result<ControlFlow>::Ok(ControlFlow::kSkip);
    }

    auto loops_valid = ValidateLoops(ctx, db);
    if (!loops_valid) return Result<ControlFlow>::Failure(loops_valid.GetError());
    if (!ctx.loop_output->accepted_global_T_agent ||
        !ctx.loop_output->accepted_alignment_method ||
        !ctx.loop_output->accepted_alignment_approval) {
      return Result<ControlFlow>::Failure(Error::InvalidArgument(
          "alignment output is missing its accepted global transform"));
    }

    AlgorithmExecutionContext algorithm_context = algorithm_context_;
    algorithm_context.agent = ctx.agent;
    algorithm_context.cancellation = ctx.cancellation;
    if (algorithm_context.progress) {
      algorithm_context.progress({ctx.agent.id, "optimize",
                                  AlgorithmProgressPhase::kOptimizeGraph, 0,
                                  1});
    }
    BackendOptimizerInput input{*ctx.raw_data,
                                ctx.loop_output->intra_loops,
                                ctx.loop_output->inter_loops,
                                db.raw_data};
    auto valid_alignment_map =
        db.descriptor_store.validate_agent_map(ctx.loop_output->alignment_map);
    if (!valid_alignment_map) {
      return Result<ControlFlow>::Failure(valid_alignment_map.GetError());
    }
    auto processed = optimizer_->Process(algorithm_context, input);
    if (!processed) {
      return Result<ControlFlow>::Failure(processed.GetError());
    }
    auto all_opt = std::move(processed).Value();

    std::map<AgentId, Eigen::Isometry3d> optimized_map_transforms;
    for (auto& [id, opt] : all_opt) {
      const auto raw = db.raw_data.find(id);
      if (raw != db.raw_data.end() && !opt.optimized_poses.empty()) {
        const auto& [index, global_pose] = opt.optimized_poses.front();
        if (index >= 0 &&
            static_cast<std::size_t>(index) < raw->second->odom_poses.size()) {
          optimized_map_transforms[id] =
              global_pose * raw->second->odom_poses[index].inverse();
        }
      }
      db.optimized_data[id] =
          std::make_shared<const AgentOptimizedData>(std::move(opt));
    }
    auto stored_map = db.descriptor_store.set_agent_map(
        ctx.agent.id, ctx.loop_output->alignment_map,
        *ctx.loop_output->accepted_global_T_agent,
        *ctx.loop_output->accepted_alignment_method,
        *ctx.loop_output->accepted_alignment_approval,
        ctx.loop_output->accepted_target_agent,
        ctx.loop_output->accepted_at_unix_ms);
    if (!stored_map) {
      return Result<ControlFlow>::Failure(stored_map.GetError());
    }
    db.descriptor_store.update_transforms(optimized_map_transforms);
    if (preview_) preview_(ctx, db);
    if (algorithm_context.progress) {
      algorithm_context.progress({ctx.agent.id, "optimize",
                                  AlgorithmProgressPhase::kOptimizeGraph, 1,
                                  1});
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
          loop.to.second >= target->second->filtered_scans.size() ||
          loop.to.second >= target->second->odom_poses.size()) {
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
  AlgorithmExecutionContext algorithm_context_;
  PreviewCallback preview_;
};

}  // namespace open_lmm
