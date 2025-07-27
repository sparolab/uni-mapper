#include "scan_context.h"

extern "C" IDescriptorKdtree* create_descriptor_kdtree_module() {

  ScanContextParams params;
  return new ScanContext(params);
}
