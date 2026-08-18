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
