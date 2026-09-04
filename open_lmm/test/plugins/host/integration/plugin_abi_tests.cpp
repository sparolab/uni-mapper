#include <open_lmm/common/descriptor_index.hpp>
#include <open_lmm/common/plugin_api.h>
#include <domain/dynamic_removal/plugin/offline_plugin.hpp>
#include <domain/dynamic_removal/plugin/online_plugin.hpp>
#include <domain/dynamic_removal/dynamic_remover_base.hpp>
#include <domain/loop_detection/database/database_kdtree.h>
#include <domain/loop_detection/loop_detector_base.hpp>
#include <plugins/host/algorithm_factory.hpp>
#include <plugins/host/load_module.hpp>
#include <open_lmm/utils/config_schema.hpp>
#include "support/check.hpp"

#include <cstdlib>
#include <iostream>
#include <map>
#include <optional>
#include <string>
#include <string_view>

namespace {

std::string CanonicalConfig(open_lmm::ConfigDocumentKind kind,
                            nlohmann::json document) {
  auto validated = open_lmm::BuiltinConfigSchemaRegistry().Validate(
      kind, document, "plugin ABI fixture");
  Check(validated.IsOk(), "built-in plugin fixture config must validate");
  return validated.Value().CanonicalJson();
}

open_lmm::PluginContractExpectation DescriptorContract(
    std::string_view model) {
  return {{"descriptor:kdtree-v3"}, model, 1, {"open-lmm-3.0.0"}};
}

open_lmm::PluginContractExpectation RemoverContract(
    std::string_view model, bool online) {
  return {online ? std::optional<std::string_view>{
                       "dynamic_remover:online-v3"}
                 : std::optional<std::string_view>{
                       "dynamic_remover:offline-v3"},
          model, 1, {"open-lmm-3.0.0"}};
}

}  // namespace

int main(int argc, char** argv) {
  std::map<std::string, std::string> built_ins;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    const auto separator = argument.find('=');
    Check(separator != std::string::npos,
          "built-in plugin argument must use target=path");
    built_ins.emplace(argument.substr(0, separator),
                      argument.substr(separator + 1));
  }
  open_lmm::AlgorithmFactory algorithms;

  open_lmm::LoopDetectorConfig missing_descriptor;
  missing_descriptor.type = "kdtree";
  missing_descriptor.model = "missing_c7_fixture";
  missing_descriptor.plugin_config_json = "{}";
  auto missing_preflight =
      algorithms.PreflightDescriptor(missing_descriptor);
  Check(!missing_preflight &&
            missing_preflight.GetError().code ==
                open_lmm::Error::Code::kPluginLoadFailed &&
            missing_preflight.GetError().context.plugin ==
                "libcreate_missing_c7_fixture.so",
        "AlgorithmFactory preflight preserves plugin failure context");

  const std::string scan_context_json = CanonicalConfig(
      open_lmm::ConfigDocumentKind::kLoopDetector,
      {{"loop_detector", {{"loop_detector_type", "kdtree"},
                          {"model", "scan_context"}}}});
  if (auto path = built_ins.find("create_scan_context");
      path != built_ins.end()) {
    open_lmm::PluginMetadata built_in_metadata;
    auto scan_context = open_lmm::load_plugin_v1<IDescriptorKdtree>(
        path->second, "descriptor", scan_context_json, &built_in_metadata,
        nullptr, DescriptorContract("scan_context"));
    Check(scan_context.IsOk(), "built-in ScanContext must use ABI v1 loader");
    auto scan_context_owner = std::move(scan_context).Value();
    Check(built_in_metadata.name == "scan_context" &&
              built_in_metadata.capability == "descriptor:kdtree-v3" &&
              built_in_metadata.config_schema_version == 1 &&
              built_in_metadata.build_version == "open-lmm-3.0.0" &&
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

    open_lmm::LoopDetectorConfig factory_config;
    factory_config.type = "kdtree";
    factory_config.model = "scan_context";
    factory_config.plugin_config_json = scan_context_json;
    Check(algorithms.PreflightDescriptor(factory_config).IsOk(),
          "AlgorithmFactory must inspect descriptor plugins in the host");
    auto detector = algorithms.CreateLoopDetector(factory_config);
    Check(detector.IsOk() && detector.Value() != nullptr,
          "AlgorithmFactory must pass a loaded descriptor owner to the domain");
  }

  if (auto path = built_ins.find("create_solid"); path != built_ins.end()) {
    const auto config = CanonicalConfig(
        open_lmm::ConfigDocumentKind::kLoopDetector,
        {{"loop_detector", {{"loop_detector_type", "kdtree"},
                            {"model", "solid"}}}});
    open_lmm::PluginMetadata metadata;
    auto solid = open_lmm::load_plugin_v1<IDescriptorKdtree>(
        path->second, "descriptor", config, &metadata, nullptr,
        DescriptorContract("solid"));
    Check(solid.IsOk() && metadata.name == "solid" &&
              metadata.build_version == "open-lmm-3.0.0",
          "built-in SOLiD must expose the exact generation contract");
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
    {
      open_lmm::PluginMetadata metadata;
      auto plugin = open_lmm::load_plugin_v1<IOnlineRemoverPlugin>(
          path->second, "dynamic_remover_online", config, &metadata, nullptr,
          RemoverContract(model, true));
      Check(plugin.IsOk() && metadata.name == model &&
                metadata.capability == "dynamic_remover:online-v3" &&
                metadata.build_version == "open-lmm-3.0.0",
            "built-in online remover must expose exact metadata");
    }

    open_lmm::DynamicRemoverConfig factory_config;
    factory_config.type = "online";
    factory_config.model = model;
    factory_config.plugin_config_json = config;
    Check(algorithms.PreflightRemover(factory_config).IsOk(),
          "AlgorithmFactory must inspect online remover plugins in the host");
    {
      auto remover = algorithms.CreateDynamicRemover(factory_config);
      Check(remover.IsOk() && remover.Value() != nullptr,
            "AlgorithmFactory must pass an online plugin owner to the domain");
    }
  }
  for (const char* target : {"create_free_dom", "create_erasor"}) {
    auto path = built_ins.find(target);
    if (path == built_ins.end()) continue;
    const std::string model = std::string(target).substr(7);
    const auto config = CanonicalConfig(
        open_lmm::ConfigDocumentKind::kDynamicRemover,
        {{"dynamic_remover", {{"dynamic_remover_type", "offline"},
                              {"model", model}}}});
    {
      open_lmm::PluginMetadata metadata;
      auto plugin = open_lmm::load_plugin_v1<IOfflineRemoverPlugin>(
          path->second, "dynamic_remover_offline", config, &metadata, nullptr,
          RemoverContract(model, false));
      Check(plugin.IsOk() && metadata.name == model &&
                metadata.capability == "dynamic_remover:offline-v3" &&
                metadata.build_version == "open-lmm-3.0.0",
            "built-in offline remover must expose exact metadata");
    }

    open_lmm::DynamicRemoverConfig factory_config;
    factory_config.type = "offline";
    factory_config.model = model;
    factory_config.plugin_config_json = config;
    Check(algorithms.PreflightRemover(factory_config).IsOk(),
          "AlgorithmFactory must inspect offline remover plugins in the host");
    {
      auto remover = algorithms.CreateDynamicRemover(factory_config);
      Check(remover.IsOk() && remover.Value() != nullptr,
            "AlgorithmFactory must pass an offline plugin owner to the domain");
    }
  }

  std::cout << "plugin ABI tests passed\n";
  return 0;
}
