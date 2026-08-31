#include "loop_detector_base.hpp"
// #include "loop_detector_sc.hpp"

namespace open_lmm {

Result<void> InspectDescriptorPlugin(const LoopDetectorConfig& config) {
  const std::string library = "libcreate_" + config.model + ".so";
  auto v1 = inspect_plugin_v1(library, "descriptor");
  if (!v1) return Result<void>::Failure(v1.GetError());
  return Result<void>::Ok();
}

}  // namespace open_lmm
