#include <open_lmm/gui/gui_plugin.hpp>
#include <open_lmm/gui/gui_runtime_host.hpp>

#include <memory>
#include <type_traits>

int main() {
  static_assert(std::is_move_constructible_v<open_lmm::GuiRuntimeHost>);
  static_assert(!std::is_copy_constructible_v<open_lmm::GuiRuntimeHost>);
  static_assert(std::is_polymorphic_v<open_lmm::GuiPlugin>);
  const auto rejected = open_lmm::GuiRuntimeHost::LoadAndStart(
      "", std::shared_ptr<open_lmm::RuntimeClient>{}, "");
  return rejected ? 1 : 0;
}
