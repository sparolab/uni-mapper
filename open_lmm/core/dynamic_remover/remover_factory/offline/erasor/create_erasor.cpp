#include "erasor.hpp"
#include "utils.hpp"

extern "C" IOfflineRemoverPlugin* create_dynamic_remover_module() {
  common::Config params;
  return new ErasorServer(params);
}