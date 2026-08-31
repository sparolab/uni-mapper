#pragma once
#include <memory>
#include <runtime/execution/pipeline.hpp>
#include <domain/data_loader/data_loader_base.hpp>

namespace open_lmm {

class DataLoadNode : public PipelineNodeBase {
 public:
  explicit DataLoadNode(std::unique_ptr<DataLoaderBase> loader)
      : loader_(std::move(loader)) {}
  DataLoadNode(std::unique_ptr<DataLoaderBase> loader,
               AlgorithmExecutionContext algorithm_context)
      : loader_(std::move(loader)),
        algorithm_context_(std::move(algorithm_context)) {}

  Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                               SharedDatabase&   db) override {
    AlgorithmExecutionContext algorithm_context = algorithm_context_;
    algorithm_context.agent = ctx.agent;
    algorithm_context.cancellation = ctx.cancellation;
    auto raw_result = loader_->Process(
        algorithm_context, DataLoaderInput{ctx.data_dir});
    if (!raw_result) {
      return Result<ControlFlow>::Failure(raw_result.GetError());
    }
    AgentRawData raw = std::move(raw_result).Value();
    if (ctx.cancellation && ctx.cancellation->IsCancellationRequested()) {
      return Result<ControlFlow>::Failure(
          Error::Cancelled("before DataLoad commit"));
    }
    auto committed = std::make_shared<const AgentRawData>(std::move(raw));
    ctx.raw_data = committed;
    db.raw_data[ctx.agent.id] = std::move(committed);
    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }

  const char* Name() const override { return "DataLoad"; }

 private:
  std::unique_ptr<DataLoaderBase> loader_;
  AlgorithmExecutionContext algorithm_context_;
};

}  // namespace open_lmm
