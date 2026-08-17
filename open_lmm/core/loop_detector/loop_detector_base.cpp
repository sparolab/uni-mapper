#include "loop_detector_base.hpp"
// #include "loop_detector_sc.hpp"

#include "loop_detector_kdtree.hpp"

namespace open_lmm {

Result<std::unique_ptr<LoopDetectorBase>> LoopDetectorBase::createInstance(
    Config config) {
  std::string loop_detector_type =
      config.param<std::string>("loop_detector", "loop_detector_type", "");
  if (loop_detector_type == "kdtree") {
    KdtreeParams params(config);
    auto module = LoopDetectorKdtree::loadModule(
        "libcreate_" + params.model + ".so");
    if (!module) {
      return Result<std::unique_ptr<LoopDetectorBase>>::Failure(module.GetError());
    }
    return Result<std::unique_ptr<LoopDetectorBase>>::Ok(
        std::make_unique<LoopDetectorKdtree>(params, std::move(module).Value()));
  } else if (loop_detector_type == "hashmap") {
    return Result<std::unique_ptr<LoopDetectorBase>>::Failure(
        Error::InvalidArgument("Hashmap loop detector is not implemented"));
  }
  return Result<std::unique_ptr<LoopDetectorBase>>::Failure(
      Error::InvalidArgument("Unknown loop_detector_type: '" +
                             loop_detector_type + "'. Supported: kdtree"));
};

}  // namespace open_lmm
