#include "backend_optimizer_base.hpp"

#include "backend_optimizer_incremental.hpp"
#include <spdlog/spdlog.h>

namespace open_lmm {

std::unique_ptr<BackendOptimizerBase> BackendOptimizerBase::createInstance(
    Config config) {
  std::string backend_optimizer_type = config.param<std::string>(
      "backend_optimizer", "backend_optimizer_type", "");
  if (backend_optimizer_type == "incremental") {
    return std::make_unique<BackendOptimizerIncremental>(config);
  }
  spdlog::error("[BackendOptimizerBase] Unknown backend_optimizer_type: '{}'. "
                "Supported: incremental", backend_optimizer_type);
  std::exit(1);
};
}  // namespace open_lmm