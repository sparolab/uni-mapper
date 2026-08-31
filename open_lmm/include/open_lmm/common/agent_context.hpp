#pragma once
#include <open_lmm/common/agent_id.hpp>

#include <cstdint>

namespace open_lmm {

enum class AgentRole : uint8_t {
  kAnchor,    // 첫 번째(기준) 에이전트 — 기준 좌표계, inter-loop 탐지 없음
  kFollower,  // 이후 에이전트 — inter-loop 탐지, loose prior
};

struct AgentContext {
  AgentId id;
  AgentSymbol symbol;
  AgentSymbolCatalogHandle catalog;
  AgentRole role;
  int       order;  // 처리 순서 (0-indexed, data_dir_list_ 인덱스)

  [[nodiscard]] bool is_anchor() const {
    return role == AgentRole::kAnchor;
  }
};

}  // namespace open_lmm
