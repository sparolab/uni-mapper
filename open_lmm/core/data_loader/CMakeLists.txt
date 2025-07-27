
add_library(open_lmm_data_loader STATIC)
target_sources(open_lmm_data_loader PRIVATE
  data_loader_base.cpp
  data_loader_file.cpp
)
target_link_libraries(open_lmm_data_loader
  Eigen3::Eigen
  open_lmm_common
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_data_loader)
