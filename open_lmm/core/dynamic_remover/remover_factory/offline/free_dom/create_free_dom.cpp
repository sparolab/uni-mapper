#include "free_dom.hpp"

extern "C" IOfflineRemoverPlugin* create_dynamic_remover_module() {
  FreeDomParams params;
  return new FreeDom(params);
}