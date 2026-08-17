#include "pipeline.hpp"

#include <chrono>
#include <exception>
#include <sstream>
#include <string>

#include <open_lmm/common/profiling.hpp>
#include <open_lmm/utils/logging.hpp>

namespace open_lmm {

Result<void> Pipeline::Run(std::vector<AgentPipelineCtx>& contexts,
                           SharedDatabase& db) {
  for (auto& ctx : contexts) {
    if (ctx.cancellation && ctx.cancellation->IsCancellationRequested()) {
      return Result<void>::Failure(Error::Cancelled("before agent execution"));
    }
    for (auto& node : nodes_) {
      if (ctx.cancellation && ctx.cancellation->IsCancellationRequested()) {
        return Result<void>::Failure(Error::Cancelled("before node execution"));
      }
      if (ctx.flow == ControlFlow::kSkip) break;

      OPEN_LMM_ZONE_N("Pipeline.Node");
#if OPEN_LMM_ENABLE_TRACY
      const std::string zone_context =
          std::string{ctx.agent.id} + " " + node->Name();
      OPEN_LMM_ZONE_TEXT(zone_context);
#endif
#if OPEN_LMM_ENABLE_TIMING_LOG
      const auto started_at = std::chrono::steady_clock::now();
#endif
      auto result = [&]() -> Result<ControlFlow> {
        try {
          return node->Process(ctx, db);
        } catch (const std::exception& error) {
          return Result<ControlFlow>::Failure(Error::InvalidArgument(
              "agent " + std::string{ctx.agent.id} + ", module " +
              node->Name() + ": " + error.what()));
        }
      }();
#if OPEN_LMM_ENABLE_TIMING_LOG
      const auto elapsed = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - started_at);
      std::ostringstream message;
      message << "[PROFILE] agent=" << ctx.agent.id
              << " module=" << node->Name()
              << " elapsed_ms=" << elapsed.count();
      LogInfo(message.str());
#endif
      if (!result) {
        ctx.flow = ControlFlow::kKill;
        return Result<void>::Failure(result.GetError());
      }
      if (result.Value() == ControlFlow::kKill) {
        ctx.flow = ControlFlow::kKill;
        return Result<void>::Failure(Error::InvalidArgument(
            std::string("Pipeline node '") + node->Name() +
            "' requested termination"));
      }
      if (result.Value() == ControlFlow::kSkip) {
        ctx.flow = ControlFlow::kSkip;
        break;
      }
    }
    OPEN_LMM_FRAME_MARK();
  }
  return Result<void>::Ok();
}

}  // namespace open_lmm
