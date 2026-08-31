#include "hmm_mos.hpp"
#include <open_lmm/common/plugin_api.h>
#include <open_lmm/plugin_build_generation.hpp>
#include <plugins/host/plugin_support.hpp>

namespace {
void* Create(const OpenLmmPluginConfigV1* config) noexcept {
  return open_lmm::CreateConfiguredPlugin<
      IOnlineRemoverPlugin, HmmMos, HmmMosParams>(config, "hmm_mos");
}
void Destroy(void* instance) noexcept {
  open_lmm::DestroyConfiguredPlugin<IOnlineRemoverPlugin>(instance);
}
const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "dynamic_remover_online", "hmm_mos",
    &Create, &Destroy, "dynamic_remover:online-v3", 1,
    open_lmm::kPluginBuildGeneration};
}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}
