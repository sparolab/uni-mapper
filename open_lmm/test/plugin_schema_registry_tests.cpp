#include <open_lmm/utils/plugin_schema_registry.hpp>

#include <cstdlib>
#include <iostream>

namespace {

void Check(bool condition, const char* message) {
  if (condition) return;
  std::cerr << message << '\n';
  std::exit(EXIT_FAILURE);
}

open_lmm::PluginV2Metadata External(std::string fragment) {
  open_lmm::PluginV2Metadata metadata;
  metadata.kind = "descriptor";
  metadata.name = "fixture_display_name";
  metadata.plugin_id = "org.example.external_descriptor";
  metadata.plugin_version = "2.4.1";
  metadata.abi_minor = 1;
  metadata.capability_bits = OPEN_LMM_CAPABILITY_SCHEMA_FRAGMENT_V2;
  metadata.schema_id = "org.example.external_descriptor.schema";
  metadata.schema_version = 3;
  metadata.schema_fragment_json = std::move(fragment);
  metadata.selected_model = "external";
  return metadata;
}

}  // namespace

int main() {
  using namespace open_lmm;
  const auto builtin_before = BuiltinConfigSchemaRegistry().Fragments(
      ConfigDocumentKind::kLoopDetector).size();
  auto external = External(R"({
    "id":"org.example.external_descriptor.schema",
    "version":3,
    "document_kind":"loop_detector",
    "selector":{"pointer":"/loop_detector/model","equals":"external"},
    "fields":[{"pointer":"/loop_detector/external/tuning","type":"number",
               "default":0.25,"constraints":{"minimum":0.0,"maximum":1.0}}]
  })");
  auto decoded = DecodePluginSchemaFragment(external);
  Check(decoded && decoded.Value().selector &&
            decoded.Value().selector->plugin_name == external.plugin_id,
        "external fragment was not decoded from stable plugin metadata");
  auto session = BuildSessionSchemaRegistry(
      std::span<const PluginV2Metadata>(&external, 1));
  Check(session && session.Value().Fragments(ConfigDocumentKind::kLoopDetector)
                           .size() == builtin_before + 1,
        "external fixture did not extend a session-local registry");
  Check(BuiltinConfigSchemaRegistry().Fragments(ConfigDocumentKind::kLoopDetector)
            .size() == builtin_before,
        "session registry mutated the process-global builtin registry");

  auto identity_mismatch = external;
  identity_mismatch.schema_id = "wrong";
  Check(!DecodePluginSchemaFragment(identity_mismatch),
        "schema query/fragment identity mismatch was accepted");
  auto oversized = external;
  oversized.schema_fragment_json.assign(64U * 1024U + 1, 'x');
  Check(!DecodePluginSchemaFragment(oversized),
        "oversized plugin schema fragment was accepted");
  auto deeply_nested = external;
  deeply_nested.schema_fragment_json =
      R"({"id":"org.example.external_descriptor.schema","version":3,"document_kind":"loop_detector","fields":[{"pointer":"/loop_detector/external/tuning","type":"number","ui":)" +
      std::string(80, '[') + "0" + std::string(80, ']') + "}]}";
  Check(!DecodePluginSchemaFragment(deeply_nested),
        "deep plugin schema fragment was accepted before bounded parse");
  auto collision = External(R"({
    "id":"org.example.external_descriptor.schema","version":3,
    "document_kind":"loop_detector",
    "selector":{"pointer":"/loop_detector/model","equals":"external"},
    "fields":[{"pointer":"/loop_detector/model","type":"string"}]
  })");
  Check(!BuildSessionSchemaRegistry(
             std::span<const PluginV2Metadata>(&collision, 1)),
        "plugin schema collision with builtin catalog was accepted");
  auto wrong_domain = External(R"({
    "id":"org.example.external_descriptor.schema","version":3,
    "document_kind":"dynamic_remover",
    "selector":{"pointer":"/dynamic_remover/model","equals":"external"},
    "fields":[{"pointer":"/external/tuning","type":"number"}]
  })");
  Check(!DecodePluginSchemaFragment(wrong_domain),
        "descriptor schema escaped into the remover domain");
  auto selectorless = External(R"({
    "id":"org.example.external_descriptor.schema","version":3,
    "document_kind":"loop_detector",
    "fields":[{"pointer":"/external/tuning","type":"number"}]
  })");
  Check(!DecodePluginSchemaFragment(selectorless),
        "selector-less plugin schema was accepted");
  auto wrong_model = External(R"({
    "id":"org.example.external_descriptor.schema","version":3,
    "document_kind":"loop_detector",
    "selector":{"pointer":"/loop_detector/model","equals":"other"},
    "fields":[{"pointer":"/external/tuning","type":"number"}]
  })");
  Check(!DecodePluginSchemaFragment(wrong_model),
        "plugin schema selector ignored the selected model");
  auto escaped_field = External(R"({
    "id":"org.example.external_descriptor.schema","version":3,
    "document_kind":"loop_detector",
    "selector":{"pointer":"/loop_detector/model","equals":"external"},
    "fields":[{"pointer":"/dynamic_remover/escape","type":"number"}]
  })");
  Check(!DecodePluginSchemaFragment(escaped_field),
        "plugin schema field escaped its owning document root");
  auto escaped_rule = External(R"({
    "id":"org.example.external_descriptor.schema","version":3,
    "document_kind":"loop_detector",
    "selector":{"pointer":"/loop_detector/model","equals":"external"},
    "fields":[],
    "rules":[{"pointers":["/dynamic_remover/model"]}]
  })");
  Check(!DecodePluginSchemaFragment(escaped_rule),
        "unsupported cross-domain plugin schema rule was accepted");
  auto escaped_deprecation = External(R"({
    "id":"org.example.external_descriptor.schema","version":3,
    "document_kind":"loop_detector",
    "selector":{"pointer":"/loop_detector/model","equals":"external"},
    "fields":[{"pointer":"/loop_detector/external/old","type":"number",
               "deprecation":{"replacement_pointer":"/dynamic_remover/model"}}]
  })");
  Check(!DecodePluginSchemaFragment(escaped_deprecation),
        "unsupported cross-domain plugin deprecation was accepted");
  std::cout << "Plugin schema registry tests passed\n";
  return EXIT_SUCCESS;
}
