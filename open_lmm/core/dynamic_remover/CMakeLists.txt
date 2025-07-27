
add_subdirectory(remover_factory)


add_library(open_lmm_dynamic_remover STATIC)
target_sources(open_lmm_dynamic_remover PRIVATE
  dynamic_remover_base.cpp
  dynamic_remover_online.cpp
  dynamic_remover_offline.cpp
)
target_link_libraries(open_lmm_dynamic_remover
  Eigen3::Eigen
  open_lmm_common
  dynamic_removal_factory
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_dynamic_remover)

