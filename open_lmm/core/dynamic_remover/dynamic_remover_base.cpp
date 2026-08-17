#include "dynamic_remover_base.hpp"

#include "dynamic_remover_online.hpp"
#include "dynamic_remover_offline.hpp"
#include <spdlog/spdlog.h>

namespace open_lmm {

Result<std::shared_ptr<DynamicRemoverBase>> DynamicRemoverBase::createInstance(
    Config config) {
  std::string dynamic_remover_type =
      config.param<std::string>("dynamic_remover", "dynamic_remover_type", "");
  std::string dynamic_remover_model =
      config.param<std::string>("dynamic_remover", "model", "");
  if (dynamic_remover_type == "offline") {
    OfflineParams params;
    auto module = DynamicRemoverOffline::loadModule(
        "libcreate_" + params.model + ".so");
    if (!module) return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
        module.GetError());
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<DynamicRemoverOffline>(params,
                                                std::move(module).Value()));
  } else if (dynamic_remover_type == "online") {
    OnlineParams params;
    auto module = DynamicRemoverOnline::loadModule(
        "libcreate_" + params.model + ".so");
    if (!module) return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
        module.GetError());
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<DynamicRemoverOnline>(params,
                                               std::move(module).Value()));
  }
  return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
      Error::InvalidArgument("Unknown dynamic_remover_type: '" +
          dynamic_remover_type + "'. Supported: offline, online"));
};

}  // namespace open_lmm
