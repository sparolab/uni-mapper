#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>

#include <open_lmm/common/agent_context.hpp>
#include <open_lmm/common/agent_data.hpp>
#include <open_lmm/common/cancellation.hpp>

namespace open_lmm {
namespace fs = std::filesystem;

enum class ControlFlow : uint8_t {
  kContinue,
  kSkip,
  kKill,
};

// Mutable per-agent values carried between execution nodes. This remains a
// private runtime model shared by state storage and execution orchestration.
struct AgentPipelineCtx {
  AgentContext agent;
  fs::path data_dir;
  std::shared_ptr<CancellationToken> cancellation;
  ControlFlow flow = ControlFlow::kContinue;
  AgentRawDataHandle raw_data;
  std::shared_ptr<const LoopDetectorOutput> loop_output;
};

}  // namespace open_lmm
