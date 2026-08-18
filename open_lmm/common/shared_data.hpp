#pragma once

#include <map>
#include <memory>
#include <open_lmm/common/alignment_feedback.hpp>
#include <open_lmm/common/agent_data.hpp>

namespace open_lmm {

// server 레이어의 결과 레지스트리 — 각 모듈의 출력을 보관
struct SharedDatabase {
  AgentRawDataMap                    raw_data;        // immutable DataLoader 결과
  AgentOptimizedDataMap              optimized_data;  // immutable optimizer 결과
  DescriptorStore                    descriptor_store; // LoopDetector 공유 상태
  std::map<char, StoredAlignment>    stored_alignments;
  std::shared_ptr<AlignmentFeedbackBroker> alignment_feedback;
  // GraphStore 제거 — ISAM2가 BackendOptimizerIncremental 내부에서 상태 관리
};

}  // namespace open_lmm
