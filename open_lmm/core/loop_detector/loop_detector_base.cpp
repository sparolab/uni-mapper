#include "loop_detector_base.hpp"
// #include "loop_detector_sc.hpp"

#include <open_lmm/core/descriptor/generic_descriptor_v2_adapter.hpp>

#include <open_lmm/utils/logging.hpp>

namespace open_lmm {

Result<void> InspectDescriptorPlugin(const LoopDetectorConfig& config) {
  const std::string library = "libcreate_" + config.model + ".so";
  if (config.plugin_abi != "v1") {
    auto availability = ProbeGenericDescriptorV2Plugin(library);
    if (!availability) return Result<void>::Failure(availability.GetError());
    if (availability.Value() == DescriptorV2Availability::kAvailable) {
      return InspectGenericDescriptorV2Plugin(
          library, config.plugin_config_json);
    }
    if (config.plugin_abi == "v2") {
      return Result<void>::Failure(Error::PluginLoadFailed(
          "descriptor plugin lacks required ABI-v2 symbols or capabilities"));
    }
    LogWarning("[plugin ABI] descriptor v2 unavailable; checking v1");
  }
  auto v1 = inspect_plugin_v1(library, "descriptor");
  if (!v1) return Result<void>::Failure(v1.GetError());
  return Result<void>::Ok();
}

}  // namespace open_lmm
