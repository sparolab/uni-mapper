
add_library(open_lmm_map_updater STATIC)
target_sources(open_lmm_map_updater PRIVATE
  map_updater.cpp
)
target_link_libraries(open_lmm_map_updater PUBLIC
  Eigen3::Eigen
  open_lmm_data_loader
  open_lmm_dynamic_remover
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_map_updater)
