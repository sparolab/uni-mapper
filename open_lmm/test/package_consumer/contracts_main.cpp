#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/runtime_contracts.hpp>

int main() {
  auto agent = open_lmm::AgentId::Parse("agent-folder-name");
  open_lmm::JobHandle job{1};
  return agent && job.value == 1 ? 0 : 1;
}
