#pragma once

#include <map>
#include <open_lmm/common/agent_data.hpp>

namespace open_lmm {

// server 레이어의 결과 레지스트리 — 각 모듈의 출력을 보관
struct SharedDatabase {
  std::map<char, AgentRawData>       raw_data;        // DataLoader 결과
  std::map<char, AgentOptimizedData> optimized_data;  // BackendOptimizer 결과
  DescriptorStore                    descriptor_store; // LoopDetector 공유 상태
  // GraphStore 제거 — ISAM2가 BackendOptimizerIncremental 내부에서 상태 관리
};

}  // namespace open_lmm
