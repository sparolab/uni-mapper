
add_library(open_lmm_contracts SHARED
  agent_id.cpp
  runtime_api.cpp
)
openlmm_set_global_target_properties(open_lmm_contracts)
