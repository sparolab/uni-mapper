add_library(open_lmm_plugin_host_objects OBJECT
  algorithm_factory.cpp
)
target_link_libraries(open_lmm_plugin_host_objects PRIVATE
  Eigen3::Eigen
  PCL::PCL
  gtsam
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_profiling
  nlohmann_json::nlohmann_json
)
openlmm_set_global_target_properties(open_lmm_plugin_host_objects)
