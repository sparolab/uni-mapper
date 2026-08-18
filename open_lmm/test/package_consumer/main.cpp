#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/common/runtime_contracts.hpp>
#include <open_lmm/core/alignment/alignment_decision_policy.hpp>
#include <open_lmm/core/alignment/alignment_proposer.hpp>
#include <open_lmm/core/alignment/loop_constraint_builder.hpp>
#include <open_lmm/core/descriptor/descriptor_artifact.hpp>
#include <open_lmm/core/descriptor/descriptor_engine.hpp>
#include <open_lmm/utils/config_schema.hpp>

#include <type_traits>

int main() {
  static_assert(std::is_abstract_v<open_lmm::DescriptorEngine>);
  static_assert(std::is_class_v<open_lmm::AlignmentDecisionPolicy>);
  static_assert(std::is_class_v<open_lmm::AlignmentProposer>);
  static_assert(std::is_class_v<open_lmm::LoopConstraintBuilder>);
  const auto root_schema = open_lmm::BuiltinConfigSchemaRegistry().Fragments(
      open_lmm::ConfigDocumentKind::kRoot);
  if (root_schema.empty()) return 2;
  auto test1 = open_lmm::AgentId::Parse("test1");
  if (!test1) return 3;
  auto catalog = open_lmm::AgentSymbolCatalog::Build({test1.Value()});
  if (!catalog || catalog.Value().SymbolFor(test1.Value()).Value().Byte() != 1) {
    return 4;
  }
  open_lmm::CancellationToken cancellation;
  cancellation.Request();
  if (!cancellation.IsCancellationRequested()) return 5;
  cancellation.Complete();
  const auto telemetry = cancellation.Telemetry();
  if (!telemetry.cancel_requested_at_unix_ns ||
      !telemetry.cancel_observed_at_unix_ns ||
      !telemetry.cancel_completed_at_unix_ns) {
    return 6;
  }
  open_lmm::JobHandle job{7};
  if (job.value != 7) return 7;
  return 0;
}
