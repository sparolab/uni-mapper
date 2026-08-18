#include <open_lmm/common/plugin_api.h>

int main(void) {
  OpenLmmPluginApiV1 legacy = {0};
  legacy.abi_version = OPEN_LMM_PLUGIN_ABI_VERSION_V1;
  return legacy.abi_version == 1u ? 0 : 1;
}
