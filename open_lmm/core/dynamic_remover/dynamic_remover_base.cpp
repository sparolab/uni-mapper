#include "dynamic_remover_base.hpp"

#include "dynamic_remover_online.hpp"
#include "dynamic_remover_offline.hpp"
#include <spdlog/spdlog.h>

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
  }
  spdlog::error("[DynamicRemoverBase] Unknown dynamic_remover_type: '{}'. "
                "Supported: offline, online", dynamic_remover_type);
  std::exit(1);
};

}  // namespace open_lmm