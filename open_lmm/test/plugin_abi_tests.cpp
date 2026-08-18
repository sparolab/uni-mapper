#include "plugin_fixture_interface.hpp"

#include <open_lmm/common/descriptor_index.hpp>
#include <open_lmm/common/plugin_api.h>
#include <open_lmm/core/dynamic_remover/remover_factory/offline/interface_offline_plugin.hpp>
#include <open_lmm/core/dynamic_remover/remover_factory/online/interface_online_plugin.hpp>
#include <open_lmm/core/loop_detector/descriptor_factory/kdtree/database_kdtree.h>
#include <open_lmm/utils/load_module.hpp>
#include <open_lmm/utils/config_schema.hpp>

#include <cstdlib>
#include <iostream>
#include <map>
#include <string>

namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

std::string CanonicalConfig(open_lmm::ConfigDocumentKind kind,
                            nlohmann::json document) {
  auto validated = open_lmm::BuiltinConfigSchemaRegistry().Validate(
      kind, document, "plugin ABI fixture");
  Check(validated.IsOk(), "built-in plugin fixture config must validate");
  return validated.Value().CanonicalJson();
}

}  // namespace

int main(int argc, char** argv) {
  Check(argc >= 6, "plugin fixture paths are required");

  PluginFixtureCounters counters;
  open_lmm::PluginMetadata metadata;
  auto inspected = open_lmm::inspect_plugin_v1(argv[1], "test");
  Check(inspected.IsOk() && inspected.Value().name == "fixture" &&
            counters.creates == 0,
        "session-time plugin inspection validates metadata without create");
  auto inspected_wrong_kind = open_lmm::inspect_plugin_v1(argv[1], "other");
  Check(!inspected_wrong_kind &&
            inspected_wrong_kind.GetError().context.plugin == argv[1],
        "session-time plugin inspection reports kind and plugin context");
  auto valid = open_lmm::load_plugin_v1<PluginFixture>(
      argv[1], "test", "{}", &metadata, &counters);
  Check(valid.IsOk(), "valid ABI v1 plugin must load");
  Check(metadata.abi_version == OPEN_LMM_PLUGIN_ABI_VERSION_V1 &&
            metadata.kind == "test" && metadata.name == "fixture" &&
            metadata.capability == "test:lifecycle" &&
            metadata.config_schema_version == 1,
        "plugin metadata must be exposed");
  auto instance = std::move(valid).Value();
  Check(instance->Value() == 42 && counters.creates == 1,
        "plugin create must return the configured interface");
  instance.reset();
  Check(counters.destroys == 1,
        "plugin destroy must run before the library handle is released");

  auto wrong_kind = open_lmm::load_plugin_v1<PluginFixture>(
      argv[1], "other", "{}");
  Check(!wrong_kind.IsOk(), "plugin kind mismatch must fail");
  auto wrong_abi = open_lmm::load_plugin_v1<PluginFixture>(
      argv[2], "test", "{}");
  Check(!wrong_abi.IsOk(), "plugin ABI mismatch must fail");
  auto null_factory = open_lmm::load_plugin_v1<PluginFixture>(
      argv[3], "test", "{}");
  Check(!null_factory.IsOk(), "null plugin factory result must fail");
  auto missing_destroy = open_lmm::load_plugin_v1<PluginFixture>(
      argv[4], "test", "{}");
  Check(!missing_destroy.IsOk(), "missing plugin destroy must fail");
  auto missing_entry = open_lmm::load_plugin_v1<PluginFixture>(
      argv[5], "test", "{}");
  Check(!missing_entry.IsOk(), "missing plugin entry symbol must fail");

  std::map<std::string, std::string> built_ins;
  for (int index = 6; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto separator = argument.find('=');
    Check(separator != std::string::npos,
          "built-in plugin argument must use target=path");
    built_ins.emplace(argument.substr(0, separator),
                      argument.substr(separator + 1));
  }

  const std::string scan_context_json = CanonicalConfig(
      open_lmm::ConfigDocumentKind::kLoopDetector,
      {{"loop_detector", {{"loop_detector_type", "kdtree"},
                          {"model", "scan_context"}}}});
  if (auto path = built_ins.find("create_scan_context");
      path != built_ins.end()) {
    open_lmm::PluginMetadata built_in_metadata;
    auto scan_context = open_lmm::load_plugin_v1<IDescriptorKdtree>(
        path->second, "descriptor", scan_context_json, &built_in_metadata);
    Check(scan_context.IsOk(), "built-in ScanContext must use ABI v1 loader");
    auto scan_context_owner = std::move(scan_context).Value();
    Check(built_in_metadata.name == "scan_context" &&
              built_in_metadata.capability == "descriptor:kdtree" &&
              scan_context_owner->getDescriptorKey().size() == 20,
          "built-in plugin must receive its immutable config snapshot");
    auto empty_scan = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    auto descriptor = scan_context_owner->makeDescriptor(empty_scan);
    {
      DatabaseKdtree index(
          DatabaseKdtreeParams{20, 5, 0.2, 50}, scan_context_owner);
      index.insert(open_lmm::AgentId::Parse("A").Value(), 0, descriptor);
      scan_context_owner.reset();
      descriptor.reset();
      Check(index.getSize() == 1 && index.Clone()->getSize() == 1,
            "descriptor artifact must preserve its plugin handle owner");
    }
  }

  if (auto path = built_ins.find("create_solid"); path != built_ins.end()) {
    const auto config = CanonicalConfig(
        open_lmm::ConfigDocumentKind::kLoopDetector,
        {{"loop_detector", {{"loop_detector_type", "kdtree"},
                            {"model", "solid"}}}});
    auto solid = open_lmm::load_plugin_v1<IDescriptorKdtree>(
        path->second, "descriptor", config);
    Check(solid.IsOk(), "built-in SOLiD must use ABI v1 loader");
  }

  for (const char* target : {"create_hmm_mos", "create_dufomap",
                             "create_otd"}) {
    auto path = built_ins.find(target);
    if (path == built_ins.end()) continue;
    const std::string model = std::string(target).substr(7);
    const auto config = CanonicalConfig(
        open_lmm::ConfigDocumentKind::kDynamicRemover,
        {{"dynamic_remover", {{"dynamic_remover_type", "online"},
                              {"model", model}}}});
    auto plugin = open_lmm::load_plugin_v1<IOnlineRemoverPlugin>(
        path->second, "dynamic_remover_online", config);
    Check(plugin.IsOk(), "built-in online remover must use ABI v1 loader");
  }
  for (const char* target : {"create_free_dom", "create_erasor"}) {
    auto path = built_ins.find(target);
    if (path == built_ins.end()) continue;
    const std::string model = std::string(target).substr(7);
    const auto config = CanonicalConfig(
        open_lmm::ConfigDocumentKind::kDynamicRemover,
        {{"dynamic_remover", {{"dynamic_remover_type", "offline"},
                              {"model", model}}}});
    auto plugin = open_lmm::load_plugin_v1<IOfflineRemoverPlugin>(
        path->second, "dynamic_remover_offline", config);
    Check(plugin.IsOk(), "built-in offline remover must use ABI v1 loader");
  }

  std::cout << "plugin ABI tests passed\n";
  return 0;
}
