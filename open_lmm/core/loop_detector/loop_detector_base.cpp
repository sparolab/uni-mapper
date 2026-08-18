#include "loop_detector_base.hpp"
// #include "loop_detector_sc.hpp"

#include "scan_context_v2_adapter.hpp"

#include <open_lmm/utils/logging.hpp>

namespace open_lmm {

Result<void> InspectDescriptorPlugin(const LoopDetectorConfig& config) {
  const std::string library = "libcreate_" + config.model + ".so";
  if (config.model == "scan_context" && config.plugin_abi != "v1") {
    auto v2 = InspectScanContextV2Plugin(library, config.plugin_config_json);
    if (v2 || config.plugin_abi == "v2") return v2;
    LogWarning(
        "[plugin ABI] Scan Context v2 inspection failed; checking v1: " +
        v2.GetError().Message());
  } else if (config.plugin_abi == "v2") {
    return Result<void>::Failure(Error::InvalidArgument(
        "descriptor plugin '" + config.model +
        "' does not provide an ABI-v2 adapter"));
  }
  auto v1 = inspect_plugin_v1(library, "descriptor");
  if (!v1) return Result<void>::Failure(v1.GetError());
  return Result<void>::Ok();
}

}  // namespace open_lmm
