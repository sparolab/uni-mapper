#include <open_lmm/common/plugin_api_v2.h>

int main(void) {
  open_lmm_plugin_descriptor_v2 descriptor = {0};
  descriptor.struct_size = sizeof(descriptor);
  descriptor.abi_major = OPEN_LMM_PLUGIN_ABI_V2_MAJOR;
  descriptor.abi_minor = OPEN_LMM_PLUGIN_ABI_V2_MINOR;
  return descriptor.abi_major == 2u ? 0 : 1;
}
