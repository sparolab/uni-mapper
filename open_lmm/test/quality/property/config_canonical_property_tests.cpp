#include <open_lmm/utils/config_schema.hpp>

#include "property_generator.hpp"

#include <nlohmann/json.hpp>

#include <array>
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>

namespace {

struct Fixture {
  open_lmm::ConfigDocumentKind kind;
  const char* json;
};

constexpr std::array<Fixture, 5> fixtures{{
    {open_lmm::ConfigDocumentKind::kMapServer,
     R"({"map_server":{"enable_map_updater":false}})"},
    {open_lmm::ConfigDocumentKind::kDataLoader,
     R"({"data_loader":{"data_loader_type":"file_based"}})"},
    {open_lmm::ConfigDocumentKind::kLoopDetector,
     R"({"loop_detector":{"loop_detector_type":"kdtree","model":"scan_context"},"alignment":{"headless_policy":"legacy_combined"}})"},
    {open_lmm::ConfigDocumentKind::kBackendOptimizer,
     R"({"backend_optimizer":{"backend_optimizer_type":"incremental"}})"},
    {open_lmm::ConfigDocumentKind::kDynamicRemover,
     R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"free_dom","fov_mask_name":"mask.png"}})"},
}};

}  // namespace

int main() {
  using open_lmm::test::property::Fail;
  using open_lmm::test::property::Generator;
  const uint64_t seed = open_lmm::test::property::Seed();
  const std::size_t cases = open_lmm::test::property::Cases(500);
  Generator generator(seed);
  const auto& registry = open_lmm::BuiltinConfigSchemaRegistry();

  for (std::size_t index = 0; index < cases; ++index) {
    const Fixture& fixture = fixtures[generator.Index(fixtures.size())];
    nlohmann::json document = nlohmann::json::parse(fixture.json);
    const int indent = generator.Index(3) == 0 ? 2 : -1;
    const std::string input = document.dump(indent);
    const auto first = registry.ParseAndValidate(fixture.kind, input,
                                                 "<property>");
    if (!first) {
      Fail("config-fixture-valid", seed, index,
           first.GetError().Message());
    }
    const std::string canonical = first.Value().CanonicalJson();
    const auto second = registry.ParseAndValidate(
        fixture.kind, canonical, "<property-canonical>");
    if (!second || second.Value().CanonicalJson() != canonical ||
        second.Value().Kind() != first.Value().Kind() ||
        second.Value().Version() != first.Value().Version()) {
      Fail("config-canonical-idempotence", seed, index,
           "canonical form did not round-trip exactly");
    }
    if (canonical.find("fov_mask_name") != std::string::npos ||
        canonical.find("legacy_combined") != std::string::npos) {
      Fail("config-migration-canonical", seed, index,
           "deprecated key or value survived canonicalization");
    }
  }
  std::cout << "config canonical properties passed seed=" << seed
            << " cases=" << cases << '\n';
  return 0;
}
