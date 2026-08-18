#include <open_lmm/server/runtime_client.hpp>

#include <type_traits>

int main() {
  static_assert(std::is_move_constructible_v<open_lmm::RuntimeClient>);
  static_assert(!std::is_copy_constructible_v<open_lmm::RuntimeClient>);
  open_lmm::RuntimeClient client(1);
  const auto session = open_lmm::SessionId::Parse(
      "00000000-0000-4000-8000-000000000001");
  const auto agent = open_lmm::AgentId::Parse("consumer");
  if (!session || !agent) return 2;
  (void)client.NodeDescriptors(session.Value());
  (void)client.VisualizationSnapshot(session.Value(), agent.Value());
  (void)client.AlignmentFeedbackSnapshot(session.Value());
  (void)client.RuntimeSnapshot(session.Value());
  (void)client.SubscribeEvents(
      session.Value(), [](const open_lmm::SessionExecutionEvent&) {});
  (void)client.Wait(session.Value(), 1);
  open_lmm::ConfigCandidate candidate;
  candidate.domain = open_lmm::ConfigDomain::kMapSave;
  candidate.document_json = "{}";
  (void)client.ApplyConfig(session.Value(), candidate, {1, 1});
  candidate.domain = open_lmm::ConfigDomain::kGlobal;
  (void)client.CreateSession({"/tmp", "candidate"}, candidate);
  return client.SessionIds().empty() ? 0 : 1;
}
