#pragma once
#include <functional>
#include <memory>
#include <runtime/execution/legacy_pipeline/pipeline.hpp>
#include <domain/loop_detection/loop_detector_base.hpp>
#include <config/document/config.hpp>

namespace open_lmm {

// LoopDetectorKdtree는 내부 database_ 상태를 가지므로 에이전트마다 새 인스턴스 필요.
// DetectorFactory를 통해 Process() 호출 시마다 생성한다.
class LoopDetectNode : public PipelineNodeBase {
 public:
  using DetectorFactory =
      std::function<Result<std::unique_ptr<LoopDetectorBase>>() >;

  explicit LoopDetectNode(DetectorFactory factory)
      : factory_(std::move(factory)) {}
  LoopDetectNode(DetectorFactory factory,
                 AlgorithmExecutionContext algorithm_context)
      : factory_(std::move(factory)),
        algorithm_context_(std::move(algorithm_context)) {}

  Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                               SharedDatabase&   db) override {
    if (!ctx.raw_data) {
      return Result<ControlFlow>::Ok(ControlFlow::kSkip);
    }

    auto detector_result = factory_();
    if (!detector_result) {
      AlgorithmExecutionContext algorithm_context = algorithm_context_;
      algorithm_context.agent = ctx.agent;
      algorithm_context.cancellation = ctx.cancellation;
      algorithm_context.feedback = db.alignment_feedback;
      return Result<ControlFlow>::Failure(WithAlgorithmContext(
          detector_result.GetError(), algorithm_context));
    }
    auto detector = std::move(detector_result).Value();

    const auto stored = db.stored_alignments.find(ctx.agent.id);
    AlgorithmExecutionContext algorithm_context = algorithm_context_;
    algorithm_context.agent = ctx.agent;
    algorithm_context.cancellation = ctx.cancellation;
    algorithm_context.feedback = db.alignment_feedback;
    if (algorithm_context.progress) {
      algorithm_context.progress({ctx.agent.id, "loop_detect",
                                  AlgorithmProgressPhase::kDetectLoops, 0, 1});
    }
    LoopDetectorProcessInput input{
        .current          = *ctx.raw_data,
        .descriptor_store = db.descriptor_store,
        .all_raw_data     = db.raw_data,
        .all_optimized    = db.optimized_data,
        .stored_alignment = stored == db.stored_alignments.end()
                                ? nullptr
                                : &stored->second,
    };
    auto processed = detector->Process(algorithm_context, input);
    if (!processed) {
      return Result<ControlFlow>::Failure(processed.GetError());
    }
    auto output = std::move(processed).Value();
    if (ctx.cancellation && ctx.cancellation->IsCancellationRequested()) {
      return Result<ControlFlow>::Failure(
          WithAlgorithmContext(Error::Cancelled("before LoopDetect commit"),
                               algorithm_context));
    }

    // Mutate only the caller-owned working database. The RuntimeTransaction
    // installs this descriptor state and per-agent output together.
    if (ctx.agent.is_anchor()) {
      db.descriptor_store.set_anchor_descriptor(ctx.agent.id,
                                                output.agent_descriptors);
    } else {
      db.descriptor_store.merge_descriptor_db(ctx.agent.id,
                                              output.agent_descriptors);
    }
    ctx.loop_output =
        std::make_shared<const LoopDetectorOutput>(std::move(output));
    if (algorithm_context.progress) {
      algorithm_context.progress({ctx.agent.id, "loop_detect",
                                  AlgorithmProgressPhase::kDetectLoops, 1, 1});
    }

    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }

  const char* Name() const override { return "LoopDetect"; }

 private:
  DetectorFactory factory_;
  AlgorithmExecutionContext algorithm_context_;
};

}  // namespace open_lmm
