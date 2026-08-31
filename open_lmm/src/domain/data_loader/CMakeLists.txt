
add_library(open_lmm_data_loader SHARED)
target_sources(open_lmm_data_loader PRIVATE
  data_loader_base.cpp
  data_loader_file.cpp
)
target_link_libraries(open_lmm_data_loader
 PUBLIC
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  Eigen3::Eigen
  PCL::PCL
 PRIVATE
  open_lmm_profiling
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_data_loader)
