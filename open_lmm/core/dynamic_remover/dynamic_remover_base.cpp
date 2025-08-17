#include "dynamic_remover_base.hpp"

#include "dynamic_remover_offline.hpp"
#include "dynamic_remover_online.hpp"

namespace open_lmm {

std::shared_ptr<DynamicRemoverBase> DynamicRemoverBase::createInstance(
    Config config) {
  std::string dynamic_remover_type =
      config.param<std::string>("dynamic_remover", "dynamic_remover_type", "");
  std::string dynamic_remover_model =
      config.param<std::string>("dynamic_remover", "model", "");
  if (dynamic_remover_type == "offline") {
    auto dynamic_remover =
        std::make_shared<DynamicRemoverOffline>(OfflineParams());
    return dynamic_remover;
  } else if (dynamic_remover_type == "online") {
    auto dynamic_remover =
        std::make_shared<DynamicRemoverOnline>(OnlineParams());
    return dynamic_remover;
  } else {
    throw std::invalid_argument(
        "[dynamic_remover_base.cpp] Invalid dynamic remover type: " +
        dynamic_remover_type);
  }
};

}  // namespace open_lmm