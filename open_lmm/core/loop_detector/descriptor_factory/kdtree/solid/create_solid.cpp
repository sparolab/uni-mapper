#include "solid.h"

extern "C" IDescriptorKdtree* create_descriptor_kdtree_module() {
  SolidParams params;
  return new SOLiD(params);
}
