
add_library(open_lmm_backend_optimizer SHARED)
target_sources(open_lmm_backend_optimizer PRIVATE
  backend_optimizer_base.cpp
  backend_optimizer_incremental.cpp
)
target_link_libraries(open_lmm_backend_optimizer
 PUBLIC
  Eigen3::Eigen
  PCL::PCL
  gtsam
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
 PRIVATE
  open_lmm_profiling
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_backend_optimizer)
