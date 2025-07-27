#include "dufomap.hpp"

extern "C" IOnlineRemoverPlugin* create_dynamic_remover_module() {
  DUFOMapParams params;
  return new DUFOMap(params);
}