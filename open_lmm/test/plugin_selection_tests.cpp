#include <config/domain/algorithm_config.hpp>

#include <cstdlib>
#include <cmath>
#include <iostream>
#include <limits>
#include <string>

namespace {

void Check(bool condition, const std::string& message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
}

template <typename T, typename Decoder>
open_lmm::Result<T> ValidateAndDecode(open_lmm::ConfigDocumentKind kind,
                                      std::string_view json,
                                      Decoder&& decoder) {
  auto validated = open_lmm::BuiltinConfigSchemaRegistry().ParseAndValidate(
      kind, json, "plugin-selection-test");
  if (!validated)
    return open_lmm::Result<T>::Failure(validated.GetError());
  return decoder(validated.Value());
}

}  // namespace

int main(int argc, char** argv) {
  Check(open_lmm::ValidateDescriptorPluginSelection("external_descriptor")
            .IsOk(),
        "unknown descriptor names remain eligible for external ABI loading");
  Check(open_lmm::ValidateDynamicRemoverPluginSelection(
            "online", "external_remover").IsOk(),
        "unknown remover names remain eligible for external ABI loading");
  Check(!open_lmm::ValidateDynamicRemoverPluginSelection(
             "offline", "hmm_mos"),
        "known online remover rejects an offline module combination");
  Check(!open_lmm::ValidateDynamicRemoverPluginSelection(
             "online", "free_dom"),
        "known offline remover rejects an online module combination");
  auto erasor = ValidateAndDecode<open_lmm::DynamicRemoverConfig>(
      open_lmm::ConfigDocumentKind::kDynamicRemover,
      R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"erasor"}})",
      open_lmm::DecodeDynamicRemoverConfig);
  Check(erasor && erasor.Value().thread_safety ==
                       open_lmm::PluginThreadSafety::kInstanceIsolatedParallel,
        "ERASOR declares task-instance isolated parallel capability");
  Check(erasor && erasor.Value().internal_cpu_threads == 1,
        "ERASOR defaults to one internally managed CPU thread");
  auto two_internal_threads = ValidateAndDecode<open_lmm::DynamicRemoverConfig>(
      open_lmm::ConfigDocumentKind::kDynamicRemover,
      R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"erasor","internal_cpu_threads":2}})",
      open_lmm::DecodeDynamicRemoverConfig);
  Check(two_internal_threads &&
            two_internal_threads.Value().internal_cpu_threads == 2,
        "ERASOR internal CPU thread request is preserved");
  auto invalid_internal_threads = ValidateAndDecode<open_lmm::DynamicRemoverConfig>(
      open_lmm::ConfigDocumentKind::kDynamicRemover,
      R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"erasor","internal_cpu_threads":0}})",
      open_lmm::DecodeDynamicRemoverConfig);
  Check(!invalid_internal_threads,
        "zero ERASOR internal CPU threads are rejected");
  auto external = ValidateAndDecode<open_lmm::DynamicRemoverConfig>(
      open_lmm::ConfigDocumentKind::kDynamicRemover,
      R"({"dynamic_remover":{"dynamic_remover_type":"online","model":"external_remover"}})",
      open_lmm::DecodeDynamicRemoverConfig);
  Check(external && external.Value().thread_safety ==
                         open_lmm::PluginThreadSafety::kSingleThreadOnly,
        "unverified remover defaults to single-thread-only capability");

  auto default_policy = ValidateAndDecode<open_lmm::LoopDetectorConfig>(
      open_lmm::ConfigDocumentKind::kLoopDetector,
      R"({"loop_detector":{"loop_detector_type":"kdtree","model":"external_descriptor"}})",
      open_lmm::DecodeLoopDetectorConfig);
  Check(default_policy &&
            default_policy.Value().headless_policy == "kiss_then_descriptor",
        "headless alignment defaults to one accepted fallback transform");
  auto legacy_policy = ValidateAndDecode<open_lmm::LoopDetectorConfig>(
      open_lmm::ConfigDocumentKind::kLoopDetector,
      R"({"loop_detector":{"loop_detector_type":"kdtree","model":"external_descriptor"},"alignment":{"headless_policy":"legacy_combined"}})",
      open_lmm::DecodeLoopDetectorConfig);
  Check(legacy_policy &&
            legacy_policy.Value().headless_policy == "kiss_then_descriptor",
        "legacy_combined is a deprecated kiss_then_descriptor alias");

  auto rigid_loader = ValidateAndDecode<open_lmm::DataLoaderConfig>(
      open_lmm::ConfigDocumentKind::kDataLoader,
      R"({"data_loader":{"data_loader_type":"file_based","extrinsic":[1,2,3,0,0,0.7071067811865475,0.7071067811865476]}})",
      open_lmm::DecodeDataLoaderConfig);
  Check(rigid_loader && rigid_loader.Value().extrinsic.matrix().allFinite() &&
            std::abs(rigid_loader.Value().extrinsic.linear().determinant() -
                     1.0) < 1.0e-12,
        "validated extrinsic must decode to a finite rigid transform");
  auto huge_quaternion_loader = ValidateAndDecode<open_lmm::DataLoaderConfig>(
      open_lmm::ConfigDocumentKind::kDataLoader,
      R"({"data_loader":{"data_loader_type":"file_based","extrinsic":[1,2,3,1e300,-1e300,1e300,-1e300]}})",
      open_lmm::DecodeDataLoaderConfig);
  Check(huge_quaternion_loader &&
            huge_quaternion_loader.Value().extrinsic.matrix().allFinite() &&
            std::abs(huge_quaternion_loader.Value()
                         .extrinsic.linear().determinant() - 1.0) < 1.0e-12,
        "scaled quaternion normalization must remain finite for huge values");

  auto unchecked_loader_fragments =
      open_lmm::BuiltinConfigSchemaRegistry().Fragments(
          open_lmm::ConfigDocumentKind::kDataLoader);
  for (auto& fragment : unchecked_loader_fragments) {
    for (auto& field : fragment.fields) {
      if (field.pointer == "/data_loader/voxel_size")
        field.constraints.maximum.reset();
    }
  }
  auto unchecked_loader_registry =
      open_lmm::SchemaRegistry::Create(std::move(unchecked_loader_fragments));
  Check(unchecked_loader_registry.IsOk(),
        "checked float conversion fixture registry must be valid");
  auto unchecked_loader_document = unchecked_loader_registry.Value().Validate(
      open_lmm::ConfigDocumentKind::kDataLoader,
      nlohmann::json{{"data_loader",
                      {{"data_loader_type", "file_based"},
                       {"voxel_size", 1.0e300}}}});
  Check(unchecked_loader_document.IsOk(),
        "checked float conversion fixture document must validate");
  auto checked_float_conversion =
      open_lmm::DecodeDataLoaderConfig(unchecked_loader_document.Value());
  Check(!checked_float_conversion,
        "typed decoder must reject a validated number that exceeds float");

  auto unchecked_fragments =
      open_lmm::BuiltinConfigSchemaRegistry().Fragments(
          open_lmm::ConfigDocumentKind::kBackendOptimizer);
  for (auto& fragment : unchecked_fragments) {
    for (auto& field : fragment.fields) {
      if (field.pointer == "/backend_optimizer/relinearizeSkip")
        field.constraints.maximum.reset();
    }
  }
  auto unchecked_registry =
      open_lmm::SchemaRegistry::Create(std::move(unchecked_fragments));
  Check(unchecked_registry.IsOk(),
        "checked-conversion fixture registry must be valid");
  auto unchecked_document = unchecked_registry.Value().Validate(
      open_lmm::ConfigDocumentKind::kBackendOptimizer,
      nlohmann::json{{"backend_optimizer",
                      {{"backend_optimizer_type", "incremental"},
                       {"relinearizeSkip", 2147483648ULL}}}});
  Check(unchecked_document.IsOk(),
        "checked-conversion fixture document must validate");
  auto checked_conversion =
      open_lmm::DecodeOptimizerConfig(unchecked_document.Value());
  Check(!checked_conversion,
        "typed decoder must reject a validated integer that exceeds int");

  for (int index = 1; index < argc; ++index) {
    const std::string selection = argv[index];
    const auto separator = selection.find(':');
    Check(separator != std::string::npos,
          "disabled selection must use kind:model");
    const std::string kind = selection.substr(0, separator);
    const std::string model = selection.substr(separator + 1);
    open_lmm::Result<void> result = kind == "descriptor"
        ? open_lmm::ValidateDescriptorPluginSelection(model)
        : open_lmm::ValidateDynamicRemoverPluginSelection(kind, model);
    Check(!result, "disabled built-in plugin must fail validation: " + model);
    Check(result.GetError().Message().find("not included") !=
              std::string::npos,
          "disabled built-in error must identify build exclusion: " + model);
  }

  std::cout << "plugin selection tests passed\n";
  return 0;
}
