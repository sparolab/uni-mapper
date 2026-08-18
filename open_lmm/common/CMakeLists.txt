
add_library(open_lmm_contracts SHARED
  agent_id.cpp
  plugin_host_v2.cpp
  runtime_contracts.cpp
)
target_link_libraries(open_lmm_contracts PRIVATE ${CMAKE_DL_LIBS})
openlmm_set_global_target_properties(open_lmm_contracts)

add_library(open_lmm_common SHARED)
target_sources(open_lmm_common PRIVATE
  pose_conversion.cpp
  registration.cpp
  registration_log.cpp
  registration_pointcloud.cpp
  pointcloud_utils.cpp
)
target_link_libraries(open_lmm_common
 PUBLIC
  open_lmm_contracts
  Eigen3::Eigen
  PCL::PCL
 PRIVATE
  open_lmm_profiling
  small_gicp
)
openlmm_set_global_target_properties(open_lmm_common)
