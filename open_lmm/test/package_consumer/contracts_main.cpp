#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/runtime_contracts.hpp>

int main() {
  auto agent = open_lmm::AgentId::Parse("agent-folder-name");
  auto session = open_lmm::SessionId::Parse(
      "01234567-89ab-4def-8123-456789abcdef");
  return agent && session ? 0 : 1;
}
