
add_library(open_lmm_backend_optimizer STATIC)
target_sources(open_lmm_backend_optimizer PRIVATE
  backend_optimizer_base.cpp
  backend_optimizer_incremental.cpp
)
target_link_libraries(open_lmm_backend_optimizer
  Eigen3::Eigen
  open_lmm_common
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_backend_optimizer)
