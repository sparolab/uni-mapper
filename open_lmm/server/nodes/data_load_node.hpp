#pragma once
#include <memory>
#include <open_lmm/common/pipeline.hpp>
#include <open_lmm/core/data_loader/data_loader_base.hpp>

namespace open_lmm {

class DataLoadNode : public PipelineNodeBase {
 public:
  explicit DataLoadNode(std::unique_ptr<DataLoaderBase> loader)
      : loader_(std::move(loader)) {}

  Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                               SharedDatabase&   db) override {
    AgentRawData raw = loader_->Process(ctx.agent, ctx.data_dir);
    ctx.raw_data = raw;
    db.raw_data[ctx.agent.id] = std::move(raw);
    return Result<ControlFlow>::Ok(ControlFlow::kContinue);
  }

  const char* Name() const override { return "DataLoad"; }

 private:
  std::unique_ptr<DataLoaderBase> loader_;
};

}  // namespace open_lmm
