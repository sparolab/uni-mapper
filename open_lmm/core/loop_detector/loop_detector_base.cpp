#include "loop_detector_base.hpp"
// #include "loop_detector_sc.hpp"

#include "loop_detector_kdtree.hpp"

namespace open_lmm {

Result<std::unique_ptr<LoopDetectorBase>> LoopDetectorBase::createInstance(
    const LoopDetectorConfig& config) {
  if (config.type == "kdtree") {
    auto module = LoopDetectorKdtree::loadModule(
        "libcreate_" + config.model + ".so", config.plugin_config_json);
    if (!module) {
      return Result<std::unique_ptr<LoopDetectorBase>>::Failure(module.GetError());
    }
    return Result<std::unique_ptr<LoopDetectorBase>>::Ok(
        std::make_unique<LoopDetectorKdtree>(config, std::move(module).Value()));
  }
  return Result<std::unique_ptr<LoopDetectorBase>>::Failure(
      Error::InvalidArgument("Unknown loop_detector_type: '" +
                             config.type + "'. Supported: kdtree"));
};

}  // namespace open_lmm
