
add_library(open_lmm_dynamic_remover SHARED)
target_sources(open_lmm_dynamic_remover PRIVATE
  dynamic_remover_base.cpp
  dynamic_remover_online.cpp
  dynamic_remover_offline.cpp
)
target_link_libraries(open_lmm_dynamic_remover
 PUBLIC
  Eigen3::Eigen
  PCL::PCL
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
 PRIVATE
  open_lmm_profiling
  small_gicp
)
openlmm_set_global_target_properties(open_lmm_dynamic_remover)
