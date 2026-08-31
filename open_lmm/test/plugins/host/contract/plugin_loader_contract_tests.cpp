#include "plugins/fixtures/plugin_fixture_interface.hpp"

#include <plugins/host/load_module.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  std::exit(1);
}

}  // namespace

int main(int argc, char** argv) {
  Check(argc == 17, "all plugin loader fixture paths are required");

  PluginFixtureCounters counters;
  open_lmm::PluginMetadata metadata;
  const open_lmm::PluginContractExpectation exact_contract{
      {"test:lifecycle"}, {"fixture"}, 1, {"fixture-1"}};
  const auto inspected = open_lmm::inspect_plugin_v1(
      argv[1], "test", exact_contract);
  Check(inspected && inspected.Value().name == "fixture" &&
            counters.creates == 0,
        "inspection validates exact metadata without creating an instance");
  Check(!open_lmm::inspect_plugin_v1(argv[1], "other") &&
            !open_lmm::inspect_plugin_v1(argv[1], "test", {{"other"}}),
        "kind and exact capability mismatches fail inspection");

  auto loaded = open_lmm::load_plugin_v1<PluginFixture>(
      argv[1], "test", "{}", &metadata, &counters,
      exact_contract);
  Check(loaded && metadata.kind == "test" && metadata.name == "fixture" &&
            metadata.capability == "test:lifecycle" && counters.creates == 1,
        "valid plugin exposes metadata and configured instance");
  auto instance = std::move(loaded).Value();
  Check(instance->Value() == 42, "loaded fixture exposes its interface");
  instance.reset();
  Check(counters.destroys == 1,
        "destroy executes while the captured DSO owner remains alive");

  Check(!open_lmm::load_plugin_v1<PluginFixture>(argv[2], "test", "{}"),
        "wrong ABI fails");
  Check(!open_lmm::load_plugin_v1<PluginFixture>(argv[3], "test", "{}"),
        "successful-null create result fails");
  Check(!open_lmm::load_plugin_v1<PluginFixture>(argv[4], "test", "{}"),
        "missing destroy fails");
  Check(!open_lmm::load_plugin_v1<PluginFixture>(argv[5], "test", "{}"),
        "missing entry symbol fails");
  Check(!open_lmm::inspect_plugin_v1(argv[6], "test") &&
            !open_lmm::load_plugin_v1<PluginFixture>(argv[6], "test", "{}"),
        "null plugin kind fails inspection and loading");
  Check(!open_lmm::inspect_plugin_v1(argv[7], "test") &&
            !open_lmm::load_plugin_v1<PluginFixture>(argv[7], "test", "{}"),
        "null plugin name fails inspection and loading");

  for (int index : {8, 9}) {
    const auto empty = open_lmm::inspect_plugin_v1(
        argv[index], "test", {{std::string_view{}}});
    Check(empty && empty.Value().capability.empty(),
          "null and empty capabilities normalize to an empty contract");
    Check(!open_lmm::inspect_plugin_v1(
              argv[index], "test", {{"required:capability"}}),
          "empty capability cannot satisfy a non-empty exact contract");
  }

  Check(!open_lmm::load_plugin_v1<PluginFixture>(argv[10], "test", "{}"),
        "throwing create callback is converted to loader failure");
  Check(!open_lmm::inspect_plugin_v1(argv[11], "test") &&
            !open_lmm::load_plugin_v1<PluginFixture>(argv[11], "test", "{}"),
        "throwing entry callback is converted to loader failure");
  Check(!open_lmm::inspect_plugin_v1(argv[12], "test") &&
            !open_lmm::load_plugin_v1<PluginFixture>(argv[12], "test", "{}"),
        "null entry callback result fails safely");

  const auto reject_before_create = [&](int index, const char* field) {
    PluginFixtureCounters rejected_counters;
    const auto rejected = open_lmm::load_plugin_v1<PluginFixture>(
        argv[index], "test", "{}", nullptr, &rejected_counters,
        exact_contract);
    Check(!rejected && rejected_counters.creates == 0 &&
              rejected.GetError().context.plugin == argv[index] &&
              rejected.GetError().message.find("expected '") !=
                  std::string::npos &&
              rejected.GetError().message.find("got '") != std::string::npos,
          field);
  };
  reject_before_create(8, "empty plugin capability reached create");
  reject_before_create(9, "null plugin capability reached create");
  reject_before_create(13, "stale plugin name reached create");
  reject_before_create(14, "stale plugin schema reached create");
  reject_before_create(15, "stale plugin build generation reached create");
  reject_before_create(16, "null plugin build generation reached create");

  PluginFixtureCounters capability_counters;
  const open_lmm::PluginContractExpectation wrong_capability{
      {"test:stale"}, {"fixture"}, 1, {"fixture-1"}};
  Check(!open_lmm::load_plugin_v1<PluginFixture>(
            argv[1], "test", "{}", nullptr, &capability_counters,
            wrong_capability) &&
            capability_counters.creates == 0,
        "stale capability reached create");

  std::cout << "plugin loader contract tests passed\n";
  return 0;
}
