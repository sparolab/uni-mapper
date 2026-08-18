#include "dynamic_remover_base.hpp"

#include "dynamic_remover_online.hpp"
#include "dynamic_remover_offline.hpp"

namespace open_lmm {

Result<std::shared_ptr<DynamicRemoverBase>> DynamicRemoverBase::createInstance(
    const DynamicRemoverConfig& config) {
  if (config.type == "offline") {
    auto module = DynamicRemoverOffline::loadModule(
        "libcreate_" + config.model + ".so", config.plugin_config_json);
    if (!module) return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
        module.GetError());
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<DynamicRemoverOffline>(config,
                                                std::move(module).Value()));
  } else if (config.type == "online") {
    auto module = DynamicRemoverOnline::loadModule(
        "libcreate_" + config.model + ".so", config.plugin_config_json);
    if (!module) return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
        module.GetError());
    return Result<std::shared_ptr<DynamicRemoverBase>>::Ok(
        std::make_shared<DynamicRemoverOnline>(config,
                                               std::move(module).Value()));
  }
  return Result<std::shared_ptr<DynamicRemoverBase>>::Failure(
      Error::InvalidArgument("Unknown dynamic_remover_type: '" +
          config.type + "'. Supported: offline, online"));
};

}  // namespace open_lmm
