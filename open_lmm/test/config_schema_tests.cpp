#include <open_lmm/utils/config_schema.hpp>
#include <open_lmm/utils/config.hpp>

#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

int failures = 0;

void Expect(bool condition, const std::string& message) {
  if (condition) return;
  std::cerr << "FAIL: " << message << '\n';
  ++failures;
}

std::string Read(const std::filesystem::path& path) {
  std::ifstream stream(path);
  std::ostringstream contents;
  contents << stream.rdbuf();
  return contents.str();
}

void ValidateFile(const open_lmm::SchemaRegistry& registry,
                  open_lmm::ConfigDocumentKind kind,
                  const std::filesystem::path& path) {
  auto validated = registry.ParseAndValidate(kind, Read(path), path.string());
  Expect(validated.IsOk(), path.string() + " must satisfy the built-in schema");
  if (!validated) return;
  auto round_trip = registry.ParseAndValidate(
      kind, validated.Value().CanonicalJson(), path.string() + ":canonical");
  Expect(round_trip.IsOk(), path.string() + " canonical form must round-trip");
  if (round_trip) {
    Expect(round_trip.Value().CanonicalJson() == validated.Value().CanonicalJson(),
           path.string() + " canonical form must be stable");
  }
}

}  // namespace

int main() {
  using open_lmm::ConfigDocumentKind;
  const auto& registry = open_lmm::BuiltinConfigSchemaRegistry();
  const std::filesystem::path config_root =
      std::filesystem::path(OPEN_LMM_SOURCE_DIR) / "config";

  const auto replacement_root = std::filesystem::temp_directory_path() /
                                "open_lmm_config_snapshot_replacement_test";
  std::filesystem::remove_all(replacement_root);
  std::filesystem::create_directories(replacement_root);
  const auto selected = replacement_root / "module.json";
  const auto replacement = replacement_root / "replacement.json";
  {
    std::ofstream(selected)
        << R"({"map_server":{"enable_map_updater":false}})";
    std::ofstream(replacement)
        << R"({"map_server":{"enable_map_updater":true}})";
  }
  auto bounded_snapshot = open_lmm::LoadConfigFileBounded(
      selected, open_lmm::SchemaLimits{}.maximum_document_bytes);
  std::filesystem::rename(replacement, selected);
  auto snapshot_document = bounded_snapshot
      ? registry.ParseAndValidate(ConfigDocumentKind::kMapServer,
                                  bounded_snapshot.Value().ToJson(),
                                  selected.string())
      : open_lmm::Result<open_lmm::ValidatedConfigDocument>::Failure(
            bounded_snapshot.GetError());
  auto replacement_snapshot = open_lmm::LoadConfigFileBounded(
      selected, open_lmm::SchemaLimits{}.maximum_document_bytes);
  auto replacement_document = replacement_snapshot
      ? registry.ParseAndValidate(ConfigDocumentKind::kMapServer,
                                  replacement_snapshot.Value().ToJson(),
                                  selected.string())
      : open_lmm::Result<open_lmm::ValidatedConfigDocument>::Failure(
            replacement_snapshot.GetError());
  Expect(snapshot_document && replacement_document &&
             !snapshot_document.Value().Document()
                  .at("map_server")
                  .at("enable_map_updater")
                  .get<bool>() &&
             replacement_document.Value().Document()
                  .at("map_server")
                  .at("enable_map_updater")
                  .get<bool>(),
         "atomic path replacement must not mutate an acquired config snapshot");
  std::filesystem::remove_all(replacement_root);

  ValidateFile(registry, ConfigDocumentKind::kRoot,
               config_root / "config.json");
  ValidateFile(registry, ConfigDocumentKind::kMapServer,
               config_root / "server/config_map_server.json");
  ValidateFile(registry, ConfigDocumentKind::kDataLoader,
               config_root / "core/data_loader/file_based.json");
  ValidateFile(registry, ConfigDocumentKind::kBackendOptimizer,
               config_root / "core/backend_optimizer/incremental.json");
  for (const char* name : {"scan_context.json", "solid.json"}) {
    ValidateFile(registry, ConfigDocumentKind::kLoopDetector,
                 config_root / "core/loop_detector" / name);
  }
  auto default_loop_policy = registry.ParseAndValidate(
      ConfigDocumentKind::kLoopDetector,
      Read(config_root / "core/loop_detector/scan_context.json"));
  Expect(default_loop_policy &&
             default_loop_policy.Value().Document()
                     .at("alignment")
                     .at("headless_policy") == "kiss_then_descriptor",
         "loop schema must materialize kiss_then_descriptor as its default");
  auto migrated_loop_policy = registry.ParseAndValidate(
      ConfigDocumentKind::kLoopDetector,
      R"({"loop_detector":{"loop_detector_type":"kdtree","model":"scan_context"},"alignment":{"headless_policy":"legacy_combined"}})");
  Expect(migrated_loop_policy &&
             migrated_loop_policy.Value().Document()
                     .at("alignment")
                     .at("headless_policy") == "kiss_then_descriptor" &&
             migrated_loop_policy.Value().Warnings().size() == 1,
         "loop schema must migrate legacy_combined in the canonical document");
  open_lmm::ValidationOptions no_value_migration;
  no_value_migration.migrate_deprecated_keys = false;
  auto rejected_legacy_policy = registry.ParseAndValidate(
      ConfigDocumentKind::kLoopDetector,
      R"({"loop_detector":{"loop_detector_type":"kdtree","model":"scan_context"},"alignment":{"headless_policy":"legacy_combined"}})",
      "legacy policy", no_value_migration);
  Expect(!rejected_legacy_policy,
         "deprecated policy value must be rejected when migration is disabled");
  auto invalid_solid_fov = registry.ParseAndValidate(
      ConfigDocumentKind::kLoopDetector,
      R"({"loop_detector":{"loop_detector_type":"kdtree","model":"solid","fov_d":5.0,"fov_u":2.0}})");
  Expect(!invalid_solid_fov,
         "SOLiD cross-field constraints must be owned by the schema");
  auto fractional_solid_distance = registry.ParseAndValidate(
      ConfigDocumentKind::kLoopDetector,
      R"({"loop_detector":{"loop_detector_type":"kdtree","model":"solid","min_distance":3.5}})");
  Expect(!fractional_solid_distance,
         "SOLiD integer distance representation must reject fractions");
  auto zero_extrinsic_quaternion = registry.ParseAndValidate(
      ConfigDocumentKind::kDataLoader,
      R"({"data_loader":{"data_loader_type":"file_based","extrinsic":[0,0,0,0,0,0,0]}})");
  Expect(!zero_extrinsic_quaternion,
         "zero extrinsic quaternion must be rejected by the schema");
  auto near_zero_extrinsic_quaternion = registry.ParseAndValidate(
      ConfigDocumentKind::kDataLoader,
      R"({"data_loader":{"data_loader_type":"file_based","extrinsic":[0,0,0,1e-14,0,0,0]}})");
  Expect(!near_zero_extrinsic_quaternion,
         "near-zero extrinsic quaternion must be rejected by the schema");
  auto non_finite_extrinsic_item = registry.Validate(
      ConfigDocumentKind::kDataLoader,
      nlohmann::json{{"data_loader",
                      {{"data_loader_type", "file_based"},
                       {"extrinsic",
                        {std::numeric_limits<double>::infinity(), 0, 0, 0, 0,
                         0, 1}}}}});
  Expect(!non_finite_extrinsic_item,
         "extrinsic array items must all be finite");
  auto oversized_loader_float = registry.ParseAndValidate(
      ConfigDocumentKind::kDataLoader,
      R"({"data_loader":{"data_loader_type":"file_based","voxel_size":1e300}})");
  Expect(!oversized_loader_float,
         "float-backed loader fields must reject values above FLT_MAX");
  auto oversized_alignment_float = registry.ParseAndValidate(
      ConfigDocumentKind::kLoopDetector,
      R"({"loop_detector":{"loop_detector_type":"kdtree","model":"scan_context"},"alignment":{"kiss_voxel_size":1e300}})");
  Expect(!oversized_alignment_float,
         "float-backed alignment fields must reject values above FLT_MAX");
  auto oversized_optimizer_int = registry.Validate(
      ConfigDocumentKind::kBackendOptimizer,
      nlohmann::json{{"backend_optimizer",
                      {{"relinearizeSkip",
                        static_cast<uint64_t>(
                            std::numeric_limits<int>::max()) + 1ULL}}}});
  Expect(!oversized_optimizer_int,
         "int-backed UInt must reject values above INT_MAX");
  for (const char* name : {"dufomap.json", "erasor.json", "free_dom.json",
                           "hmm_mos.json", "otd.json"}) {
    ValidateFile(registry, ConfigDocumentKind::kDynamicRemover,
                 config_root / "core/dynamic_remover" / name);
  }

  auto migrated = registry.ParseAndValidate(
      ConfigDocumentKind::kDynamicRemover,
      R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"free_dom","fov_mask_name":"mask.png"}})");
  Expect(migrated.IsOk(), "deprecated FreeDOM key must migrate");
  if (migrated) {
    const auto& dynamic = migrated.Value().Document().at("dynamic_remover");
    Expect(dynamic.at("fov_mask_path") == "mask.png",
           "FreeDOM migration must preserve the value");
    Expect(!dynamic.contains("fov_mask_name"),
           "FreeDOM migration must remove the deprecated key");
    Expect(migrated.Value().Warnings().size() == 1,
           "FreeDOM migration must emit one warning");
  }
  auto invalid_free_dom_depth = registry.ParseAndValidate(
      ConfigDocumentKind::kDynamicRemover,
      R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"free_dom","voxel_depth":0}})");
  Expect(!invalid_free_dom_depth,
         "FreeDOM positive depth constraint must be owned by the schema");
  auto invalid_free_dom_fov_mode = registry.ParseAndValidate(
      ConfigDocumentKind::kDynamicRemover,
      R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"free_dom","fov_projection_mode":"unknown"}})");
  Expect(!invalid_free_dom_fov_mode,
         "FreeDOM unknown FOV projection mode must fail");

  auto unknown = registry.ParseAndValidate(
      ConfigDocumentKind::kMapServer,
      R"({"map_server":{"enable_map_updater":false,"typo":1}})");
  Expect(!unknown, "unknown keys must be rejected");
  if (!unknown) {
    Expect(unknown.GetError().context.json_pointer == "/map_server/typo",
           "unknown-key error must include its JSON pointer");
  }

  auto mismatch = registry.ParseAndValidate(
      ConfigDocumentKind::kDynamicRemover,
      R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"hmm_mos"}})");
  Expect(!mismatch, "plugin/type mismatch must be rejected");
  if (!mismatch) {
    Expect(mismatch.GetError().context.plugin == "dynamic_remover:hmm_mos",
           "plugin mismatch must include plugin context");
  }

  auto root = registry.ParseAndValidate(
      ConfigDocumentKind::kRoot,
      R"({"global":{"config_map_server":"m","config_data_loader":"d","config_loop_detector":"l","config_backend_optimizer":"o","config_dynamic_remover":"r"},"directory":{"root_dir_path":"in","root_save_dir":"out","sub_dir_list":["one"]}})");
  auto map = registry.ParseAndValidate(
      ConfigDocumentKind::kMapServer,
      R"({"map_server":{"anchor_agent_index":1}})");
  Expect(root && map, "session fixtures must validate individually");
  if (root && map) {
    Expect(!open_lmm::ValidateRuntimeConfigDocuments(root.Value(), map.Value()),
           "anchor index must be checked against the configured agent count");
  }

  open_lmm::SchemaFragment left{"collision.left", ConfigDocumentKind::kRoot,
                                1, std::nullopt,
                                {{"/same", open_lmm::SchemaValueType::kString}}};
  open_lmm::SchemaFragment right{"collision.right", ConfigDocumentKind::kRoot,
                                 1, std::nullopt,
                                 {{"/same", open_lmm::SchemaValueType::kString}}};
  Expect(!open_lmm::SchemaRegistry::Create({left, right}),
         "coexisting schema fragments must reject field collisions");

  open_lmm::SchemaLimits limits;
  limits.maximum_fragment_fields = 0;
  Expect(!open_lmm::SchemaRegistry::Create({left}, limits),
         "schema fragment field limits must be enforced");

  open_lmm::SchemaFragment rules{"rules", ConfigDocumentKind::kRoot, 1};
  rules.fields = {
      {"/mode", open_lmm::SchemaValueType::kString, false, "off"},
      {"/conditional", open_lmm::SchemaValueType::kString},
      {"/left", open_lmm::SchemaValueType::kString},
      {"/right", open_lmm::SchemaValueType::kString},
  };
  rules.rules = {
      {open_lmm::CrossFieldRuleKind::kRequiredWhenEquals,
       {"/mode", "/conditional"}, "on"},
      {open_lmm::CrossFieldRuleKind::kMutuallyExclusive,
       {"/left", "/right"}},
  };
  auto rule_registry = open_lmm::SchemaRegistry::Create({rules});
  Expect(rule_registry.IsOk(), "cross-field fixture schema must register");
  if (rule_registry) {
    Expect(!rule_registry.Value().ParseAndValidate(
               ConfigDocumentKind::kRoot,
               R"({"mode":"on","left":"a","right":"b"})"),
           "conditional-required and mutually-exclusive rules must reject invalid input");
  }

  open_lmm::SchemaFragment secret{"secret", ConfigDocumentKind::kRoot, 1};
  open_lmm::FieldSpec credential{"/token", open_lmm::SchemaValueType::kString,
                                 true};
  credential.secret = true;
  credential.constraints.allowed_values = {"expected"};
  secret.fields = {credential};
  auto secret_registry = open_lmm::SchemaRegistry::Create({secret});
  Expect(secret_registry.IsOk(), "secret fixture schema must register");
  if (secret_registry) {
    auto invalid_secret = secret_registry.Value().ParseAndValidate(
        ConfigDocumentKind::kRoot, R"({"token":"do-not-log"})");
    Expect(!invalid_secret && invalid_secret.GetError().context.actual == "[REDACTED]",
           "secret values must be redacted from validation context");
  }

  open_lmm::SchemaFragment nested_path{"nested-path", ConfigDocumentKind::kRoot,
                                       1};
  nested_path.fields = {
      {"/outer/", open_lmm::SchemaValueType::kString},
  };
  auto empty_path_registry =
      open_lmm::SchemaRegistry::Create({nested_path});
  Expect(!empty_path_registry,
         "schema paths containing an empty nested component must fail");
  if (!empty_path_registry) {
    Expect(empty_path_registry.GetError().code ==
               open_lmm::Error::Code::kInvalidArgument,
           "empty nested schema paths must return structured InvalidArgument");
  }

  nlohmann::json deeply_nested = nlohmann::json::object();
  nlohmann::json* cursor = &deeply_nested;
  for (std::size_t depth = 0; depth < 256; ++depth) {
    (*cursor)["child"] = nlohmann::json::object();
    cursor = &(*cursor)["child"];
  }
  auto deep_result = registry.Validate(ConfigDocumentKind::kRoot,
                                       deeply_nested, "deep-fixture");
  Expect(!deep_result,
         "deep programmatic JSON must fail without recursive traversal");
  if (!deep_result) {
    Expect(deep_result.GetError().context.expected.find("depth <=") !=
               std::string::npos,
           "deep JSON failure must retain the configured depth bound");
    Expect(deep_result.GetError().context.actual == "17",
           "depth traversal must stop at the first node beyond the bound");
  }

  return failures == 0 ? 0 : 1;
}
