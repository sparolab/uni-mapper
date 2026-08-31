#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <vector>

#include <open_lmm/common/agent_data.hpp>

namespace open_lmm {

struct VisualizationAgentSource {
  AgentId agent;
  AgentRawDataHandle raw_data;
  AgentOptimizedDataHandle optimized_data;
  std::shared_ptr<const LoopDetectorOutput> loop_output;
};

// Immutable, projection-only view assembled at the runtime boundary. It keeps
// the projector independent of configuration, transactions, optimizers, and
// all other RuntimeState ownership.
struct VisualizationSource {
  uint64_t revision = 0;
  std::filesystem::path output_directory;
  std::vector<VisualizationAgentSource> agents;
};

}  // namespace open_lmm
