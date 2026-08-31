#include "scan_context.h"
#include <open_lmm/common/plugin_api.h>
#include <open_lmm/plugin_build_generation.hpp>
#include <plugins/host/plugin_support.hpp>

namespace {
void* Create(const OpenLmmPluginConfigV1* config) noexcept {
  return open_lmm::CreateConfiguredPlugin<
      IDescriptorKdtree, ScanContext, ScanContextParams>(config,
                                                        "scan_context");
}
void Destroy(void* instance) noexcept {
  open_lmm::DestroyConfiguredPlugin<IDescriptorKdtree>(instance);
}
const OpenLmmPluginApiV1 kApi{
    OPEN_LMM_PLUGIN_ABI_VERSION_V1, "descriptor", "scan_context",
    &Create, &Destroy, "descriptor:kdtree-v3", 1,
    open_lmm::kPluginBuildGeneration};
}  // namespace

extern "C" const OpenLmmPluginApiV1* open_lmm_plugin_entry() {
  return &kApi;
}
