#pragma once

#include <memory>
#include <iomanip>
#include <sstream>
#include <string>
#include <string_view>
#include <utility>

#include <open_lmm/common/algorithm_execution_context.hpp>
#include <open_lmm/server/runtime_state.hpp>
#include <open_lmm/server/stage_ports.hpp>

namespace open_lmm {

inline std::string AlgorithmDocumentFingerprint(std::string_view canonical) {
  uint64_t hash = 14695981039346656037ULL;
  for (const unsigned char value : canonical) {
    hash ^= value;
    hash *= 1099511628211ULL;
  }
  std::ostringstream output;
  output << std::hex << std::setfill('0') << std::setw(16) << hash;
  return output.str();
}

// Converts one immutable runtime snapshot plus one command context into the
// only authority visible at a core algorithm boundary.
inline AlgorithmExecutionContext MakeAlgorithmExecutionContext(
    const RuntimeState& state, const ExecutionContext& command,
    AgentContext agent, const RuntimeConfigDocument& document,
    std::string schema_id, std::string operation, std::string plugin_id) {
  auto config = std::make_shared<const AlgorithmConfigSnapshot>(
      AlgorithmConfigSnapshot{std::move(schema_id), 1,
                              document.canonical_json,
                              AlgorithmDocumentFingerprint(
                                  document.canonical_json)});
  AlgorithmExecutionContext context;
  context.agent = std::move(agent);
  context.config = std::move(config);
  context.cancellation = command.cancellation;
  context.feedback = command.alignment_feedback;
  context.resource_budget.maximum_cpu_threads =
      state.config ? state.config->root.max_parallel_agents : 1;
  context.base_revision = state.revision;
  context.operation = std::move(operation);
  context.plugin_id = std::move(plugin_id);
  return context;
}

}  // namespace open_lmm
