#include "backend_optimizer_base.hpp"

#include "backend_optimizer_incremental.hpp"

namespace open_lmm {

std::unique_ptr<BackendOptimizerBase> BackendOptimizerBase::createInstance(
    Config config) {
  std::string backend_optimizer_type = config.param<std::string>(
      "backend_optimizer", "backend_optimizer_type", "");
  if (backend_optimizer_type == "incremental") {
    return std::make_unique<BackendOptimizerIncremental>(config);
  } else {
    throw std::invalid_argument(
        "[backend_optimizer_base.cpp] Invalid backend optimizer type: " +
        backend_optimizer_type);
  }
  // return nullptr;
};
}  // namespace open_lmm