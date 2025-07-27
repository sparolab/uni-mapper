#include "hmm_mos.hpp"

extern "C" IOnlineRemoverPlugin* create_dynamic_remover_module() {
  HmmMosParams params;
  return new HmmMos(params);
}