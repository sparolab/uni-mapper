#include "backend_optimizer_base.hpp"

#include "backend_optimizer_incremental.hpp"

namespace open_lmm {

Result<std::unique_ptr<BackendOptimizerBase>> BackendOptimizerBase::createInstance(
    const OptimizerConfig& config) {
  if (config.type == "incremental") {
    return Result<std::unique_ptr<BackendOptimizerBase>>::Ok(
        std::make_unique<BackendOptimizerIncremental>(config));
  }
  return Result<std::unique_ptr<BackendOptimizerBase>>::Failure(
      Error::InvalidArgument("Unknown backend_optimizer_type: '" +
          config.type + "'. Supported: incremental"));
};
}  // namespace open_lmm
