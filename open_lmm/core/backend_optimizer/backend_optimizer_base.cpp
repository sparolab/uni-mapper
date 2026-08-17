#include "backend_optimizer_base.hpp"

#include "backend_optimizer_incremental.hpp"
#include <spdlog/spdlog.h>

namespace open_lmm {

Result<std::unique_ptr<BackendOptimizerBase>> BackendOptimizerBase::createInstance(
    Config config) {
  std::string backend_optimizer_type = config.param<std::string>(
      "backend_optimizer", "backend_optimizer_type", "");
  if (backend_optimizer_type == "incremental") {
    return Result<std::unique_ptr<BackendOptimizerBase>>::Ok(
        std::make_unique<BackendOptimizerIncremental>(config));
  }
  return Result<std::unique_ptr<BackendOptimizerBase>>::Failure(
      Error::InvalidArgument("Unknown backend_optimizer_type: '" +
          backend_optimizer_type + "'. Supported: incremental"));
};
}  // namespace open_lmm
