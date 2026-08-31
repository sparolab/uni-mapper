#pragma once
#include <memory>
#include <vector>

#include <open_lmm/common/result.hpp>
#include <runtime/model/agent_working_set.hpp>
#include <runtime/state/shared_data.hpp>

namespace open_lmm {

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

  Result<void> Run(std::vector<AgentPipelineCtx>& contexts,
                   SharedDatabase& db);

 private:
  std::vector<std::unique_ptr<PipelineNodeBase>> nodes_;
};

}  // namespace open_lmm
