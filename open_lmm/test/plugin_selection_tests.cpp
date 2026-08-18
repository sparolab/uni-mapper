#include <open_lmm/core/algorithm_config.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void Check(bool condition, const std::string& message) {
  if (condition) return;
  std::cerr << "FAILED: " << message << '\n';
  std::exit(1);
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
  auto erasor = open_lmm::ParseDynamicRemoverConfig(
      open_lmm::Config::FromJson(
          R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"erasor"}})"));
  Check(erasor && erasor.Value().thread_safety ==
                       open_lmm::PluginThreadSafety::kInstanceIsolatedParallel,
        "ERASOR declares task-instance isolated parallel capability");
  Check(erasor && erasor.Value().internal_cpu_threads == 1,
        "ERASOR defaults to one internally managed CPU thread");
  auto two_internal_threads = open_lmm::ParseDynamicRemoverConfig(
      open_lmm::Config::FromJson(
          R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"erasor","internal_cpu_threads":2}})"));
  Check(two_internal_threads &&
            two_internal_threads.Value().internal_cpu_threads == 2,
        "ERASOR internal CPU thread request is preserved");
  auto invalid_internal_threads = open_lmm::ParseDynamicRemoverConfig(
      open_lmm::Config::FromJson(
          R"({"dynamic_remover":{"dynamic_remover_type":"offline","model":"erasor","internal_cpu_threads":0}})"));
  Check(!invalid_internal_threads,
        "zero ERASOR internal CPU threads are rejected");
  auto external = open_lmm::ParseDynamicRemoverConfig(
      open_lmm::Config::FromJson(
          R"({"dynamic_remover":{"dynamic_remover_type":"online","model":"external_remover"}})"));
  Check(external && external.Value().thread_safety ==
                         open_lmm::PluginThreadSafety::kSingleThreadOnly,
        "unverified remover defaults to single-thread-only capability");

  auto default_policy = open_lmm::ParseLoopDetectorConfig(
      open_lmm::Config::FromJson(
          R"({"loop_detector":{"loop_detector_type":"kdtree","model":"external_descriptor"}})"));
  Check(default_policy &&
            default_policy.Value().headless_policy == "kiss_then_descriptor",
        "headless alignment defaults to one accepted fallback transform");
  auto legacy_policy = open_lmm::ParseLoopDetectorConfig(
      open_lmm::Config::FromJson(
          R"({"loop_detector":{"loop_detector_type":"kdtree","model":"external_descriptor"},"alignment":{"headless_policy":"legacy_combined"}})"));
  Check(legacy_policy &&
            legacy_policy.Value().headless_policy == "kiss_then_descriptor",
        "legacy_combined is a deprecated kiss_then_descriptor alias");

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
