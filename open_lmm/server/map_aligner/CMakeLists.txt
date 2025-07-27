
add_library(open_lmm_map_aligner STATIC)
target_sources(open_lmm_map_aligner PRIVATE
  map_aligner.cpp
)
target_link_libraries(open_lmm_map_aligner PUBLIC
  Eigen3::Eigen
  open_lmm_data_loader
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_map_aligner)
