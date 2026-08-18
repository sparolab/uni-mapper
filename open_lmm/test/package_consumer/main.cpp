#include <open_lmm/gui/gui_model.hpp>
#include <open_lmm/common/agent_id.hpp>
#include <open_lmm/server/stage_runner.hpp>
#include <open_lmm/utils/config_schema.hpp>

#include <type_traits>

int main() {
  static_assert(std::is_abstract_v<open_lmm::StageRunner>);
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
  open_lmm::GuiModel model;
  return model.CanSubmitCommand() ? 0 : 1;
}
