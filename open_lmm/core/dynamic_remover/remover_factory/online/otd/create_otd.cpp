#include "otd.hpp"

extern "C" IOnlineRemoverPlugin* create_dynamic_remover_module() {
  OTDParams params;
  return new OTD(params);
}