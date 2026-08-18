#include "solid.h"
#include <open_lmm/common/plugin_api.h>
#include <open_lmm/utils/plugin_support.hpp>

namespace {
void* Create(const OpenLmmPluginConfigV1* config) noexcept {
  return open_lmm::CreateConfiguredPlugin<
      IDescriptorKdtree, SOLiD, SolidParams>(config, "solid");
}
void Destroy(void* instance) noexcept {
  open_lmm::DestroyConfiguredPlugin<IDescriptorKdtree>(instance);
}
const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "descriptor", "solid",
    &Create, &Destroy, "descriptor:kdtree", 1, "open-lmm-1.0"};
}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}
