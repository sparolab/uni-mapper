#pragma once
#include <chrono>
#include <filesystem>
#include <memory>
#include <optional>
#include <vector>

#include <spdlog/spdlog.h>

#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/profiling.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/result.hpp>
#include <open_lmm/common/cancellation.hpp>
#include <open_lmm/common/shared_data.hpp>
#include <open_lmm/core/loop_detector/loop_detector_base.hpp>

namespace open_lmm {
namespace fs = std::filesystem;

enum class ControlFlow : uint8_t {
  kContinue,  // 다음 노드로
  kSkip,      // 이 에이전트 건너뜀 (에러 없음)
  kKill,      // 전체 파이프라인 중단
};

// 파이프라인을 흐르는 에이전트별 컨텍스트
struct AgentPipelineCtx {
  AgentContext agent;
  fs::path     data_dir;
  std::shared_ptr<CancellationToken> cancellation;
  ControlFlow  flow = ControlFlow::kContinue;

  // 각 노드가 채우는 중간 결과
  std::optional<AgentRawData>       raw_data;
  std::optional<LoopDetectorOutput> loop_output;
};

// 파이프라인 노드 기반 클래스
class PipelineNodeBase {
 public:
  virtual ~PipelineNodeBase() = default;
  virtual Result<ControlFlow> Process(AgentPipelineCtx& ctx,
                                      SharedDatabase&   db) = 0;
  virtual const char* Name() const = 0;
};

// 파이프라인 실행기 — 에이전트를 순차 처리, 각 에이전트에 노드 체인 적용
class Pipeline {
 public:
  Pipeline& AddNode(std::unique_ptr<PipelineNodeBase> node) {
    nodes_.push_back(std::move(node));
    return *this;
  }

  Result<void> Run(std::vector<AgentPipelineCtx>& contexts, SharedDatabase& db) {
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
        const std::string zone_context = std::string{ctx.agent.id} + " " + node->Name();
        OPEN_LMM_ZONE_TEXT(zone_context);
#endif
#if OPEN_LMM_ENABLE_TIMING_LOG
        const auto started_at = std::chrono::steady_clock::now();
#endif
        auto r = [&]() -> Result<ControlFlow> {
          try {
            return node->Process(ctx, db);
          } catch (const std::exception& e) {
            return Result<ControlFlow>::Failure(Error::InvalidArgument(
                "agent " + std::string{ctx.agent.id} + ", module " +
                node->Name() + ": " + e.what()));
          }
        }();
#if OPEN_LMM_ENABLE_TIMING_LOG
        const auto elapsed = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - started_at);
        spdlog::info("[PROFILE] agent={} module={} elapsed_ms={:.3f}",
                     ctx.agent.id, node->Name(), elapsed.count());
#endif
        if (!r) {
          ctx.flow = ControlFlow::kKill;
          return Result<void>::Failure(r.GetError());
        }
        if (r.Value() == ControlFlow::kKill) {
          ctx.flow = ControlFlow::kKill;
          return Result<void>::Failure(Error::InvalidArgument(
              std::string("Pipeline node '") + node->Name() +
              "' requested termination"));
        }
        if (r.Value() == ControlFlow::kSkip) {
          ctx.flow = ControlFlow::kSkip;
          break;
        }
      }
      OPEN_LMM_FRAME_MARK();
    }
    return Result<void>::Ok();
  }

 private:
  std::vector<std::unique_ptr<PipelineNodeBase>> nodes_;
};

}  // namespace open_lmm
