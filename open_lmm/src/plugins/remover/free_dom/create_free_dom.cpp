#include "free_dom.hpp"
#include <open_lmm/common/plugin_api.h>
#include <plugins/host/plugin_support.hpp>

namespace {
void* Create(const OpenLmmPluginConfigV1* config) noexcept {
  return open_lmm::CreateConfiguredPlugin<
      IOfflineRemoverPlugin, FreeDom, FreeDomParams>(config, "free_dom");
}
void Destroy(void* instance) noexcept {
  open_lmm::DestroyConfiguredPlugin<IOfflineRemoverPlugin>(instance);
}
const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "dynamic_remover_offline", "free_dom",
    &Create, &Destroy, "dynamic_remover:offline", 1, "open-lmm-1.0"};
}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}
