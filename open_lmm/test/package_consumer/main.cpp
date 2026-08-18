#include <open_lmm/gui/gui_model.hpp>
#include <open_lmm/server/stage_runner.hpp>

#include <type_traits>

int main() {
  static_assert(std::is_abstract_v<open_lmm::StageRunner>);
  open_lmm::GuiModel model;
  return model.CanSubmitCommand() ? 0 : 1;
}
