#pragma once

#include <string>

#include <open_lmm/common/plugin_api.h>
#include <open_lmm/utils/config.hpp>
#include <open_lmm/utils/logging.hpp>

namespace open_lmm {

inline Config PluginConfigSnapshot(const OpenLmmPluginConfigV1* input,
                                   const char* plugin_name) {
  if (!input || input->struct_size < sizeof(OpenLmmPluginConfigV1) ||
      (!input->json_data && input->json_size != 0)) {
    return Config::FromJson("{", plugin_name);
  }
  return Config::FromJson(
      std::string(input->json_data ? input->json_data : "", input->json_size),
      std::string("plugin:") + plugin_name);
}

template <typename Interface, typename Implementation, typename Params>
void* CreateConfiguredPlugin(const OpenLmmPluginConfigV1* input,
                             const char* plugin_name) noexcept {
  try {
    auto config = PluginConfigSnapshot(input, plugin_name);
    if (!config.is_valid()) return nullptr;
    Interface* instance = new Implementation(Params(config));
    return static_cast<void*>(instance);
  } catch (const std::exception& error) {
    LogError(std::string("failed to create plugin '") + plugin_name +
             "': " + error.what());
    return nullptr;
  } catch (...) {
    LogError(std::string("failed to create plugin '") + plugin_name +
             "': unknown exception");
    return nullptr;
  }
}

template <typename Interface>
void DestroyConfiguredPlugin(void* instance) noexcept {
  delete static_cast<Interface*>(instance);
}

}  // namespace open_lmm
