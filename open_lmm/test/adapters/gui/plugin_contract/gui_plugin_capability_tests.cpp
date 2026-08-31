#include "plugins/fixtures/plugin_fixture_interface.hpp"

#include <adapters/gui/gui_plugin_host.hpp>
#include <plugins/host/load_module.hpp>

#include <cstdlib>
#include <iostream>

namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

}  // namespace

int main(int argc, char** argv) {
  Check(argc == 5, "GUI plugin fixture paths are required");
  PluginFixtureCounters counters;
  auto valid = open_lmm::load_plugin_v1<open_lmm::GuiPlugin>(
      argv[1], "gui", "{}", nullptr, &counters,
      open_lmm::PluginContractExpectation{
          .exact_capability = "gui:services-v3"});
  Check(valid && counters.creates == 1,
        "matching GUI services capability loads before create");
  std::move(valid).Value().reset();
  Check(counters.destroys == 1,
        "matching GUI capability preserves destroy-before-dlclose");

  for (int index = 2; index != argc; ++index) {
    counters = {};
    const auto rejected = open_lmm::load_plugin_v1<open_lmm::GuiPlugin>(
        argv[index], "gui", "{}", nullptr, &counters,
        open_lmm::PluginContractExpectation{
            .exact_capability = "gui:services-v3"});
    Check(!rejected && counters.creates == 0,
          "incompatible GUI capability is rejected before create");
  }
  Check(!open_lmm::GuiPluginHost::Load(argv[2]) &&
            !open_lmm::inspect_plugin_v1(
                argv[2], "gui",
                open_lmm::PluginContractExpectation{
                    .exact_capability = "gui:services-v3"}),
        "GUI host and preflight enforce the exact services contract");
  std::cout << "GUI plugin capability tests passed\n";
  return 0;
}
