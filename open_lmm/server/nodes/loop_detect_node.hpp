#pragma once
#include <functional>
#include <memory>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/core/loop_detector/loop_detector_base.hpp>
#include <open_lmm/utils/config.hpp>

namespace open_lmm {

// LoopDetectorKdtree는 내부 database_ 상태를 가지므로 에이전트마다 새 인스턴스 필요.
// DetectorFactory를 통해 Process() 호출 시마다 생성한다.
class LoopDetectNode : public PipelineNodeBase {
 public:
  using DetectorFactory =
      std::function<Result<std::unique_ptr<LoopDetectorBase>>() >;

  explicit LoopDetectNode(DetectorFactory factory)
      : factory_(std::move(factory)) {}

  Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                               SharedDatabase&   db) override {
    if (!ctx.raw_data) {
      return Result<ControlFlow>::Ok(ControlFlow::kSkip);
    }

    auto detector_result = factory_();
    if (!detector_result) {
      return Result<ControlFlow>::Failure(detector_result.GetError());
    }
    auto detector = std::move(detector_result).Value();

    const auto stored = db.stored_alignments.find(ctx.agent.id);
    LoopDetectorInput input{
        .agent_ctx        = ctx.agent,
        .current          = *ctx.raw_data,
        .descriptor_store = db.descriptor_store,
        .all_raw_data     = db.raw_data,
        .all_optimized    = db.optimized_data,
        .alignment_feedback = db.alignment_feedback,
        .cancellation     = ctx.cancellation,
        .stored_alignment = stored == db.stored_alignments.end()
                                ? nullptr
                                : &stored->second,
    };
    try {
      ctx.loop_output = detector->Process(input);
    } catch (const CancellationException& e) {
      ctx.loop_output.reset();
      return Result<ControlFlow>::Failure(Error::Cancelled(e.what()));
    } catch (const std::exception& e) {
      ctx.loop_output.reset();
      return Result<ControlFlow>::Failure(
          Error::InvalidArgument("LoopDetect failed: " + std::string(e.what())));
    }
    if (ctx.cancellation && ctx.cancellation->IsCancellationRequested()) {
      ctx.loop_output.reset();
      return Result<ControlFlow>::Failure(
          Error::Cancelled("before LoopDetect commit"));
    }

    // DescriptorStore 업데이트 (server 레이어 책임)
    if (ctx.agent.is_anchor()) {
      db.descriptor_store.set_anchor_descriptor(
          std::move(ctx.loop_output->agent_db));
    } else {
      db.descriptor_store.merge_descriptor_db(
          std::move(ctx.loop_output->agent_db));
    }

    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }

  const char* Name() const override { return "LoopDetect"; }

 private:
  DetectorFactory factory_;
};

}  // namespace open_lmm
