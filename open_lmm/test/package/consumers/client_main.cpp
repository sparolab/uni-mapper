#include <open_lmm/server/runtime_client.hpp>

#include <type_traits>

int main() {
  static_assert(std::is_move_constructible_v<open_lmm::RuntimeClient>);
  static_assert(!std::is_copy_constructible_v<open_lmm::RuntimeClient>);
  open_lmm::RuntimeClient client(1);
  const auto agent = open_lmm::AgentId::Parse("consumer");
  if (!agent) return 2;
  (void)client.NodeDescriptors();
  (void)client.ConfigDocuments();
  (void)client.ConfigCandidates();
  (void)client.Visualization(agent.Value());
  (void)client.AlignmentFeedback();
  (void)client.Snapshot();
  (void)client.SubscribeEvents([](const open_lmm::ExecutionEvent&) {});
  (void)client.Wait({1});
  open_lmm::ConfigCandidate candidate;
  candidate.domain = open_lmm::ConfigDomain::kMapSave;
  candidate.document_json = "{}";
  (void)client.ApplyConfig(candidate, {1, 1});
  candidate.domain = open_lmm::ConfigDomain::kGlobal;
  (void)client.Open({"/tmp", "candidate"}, candidate);
  return 0;
}
