
add_library(open_lmm_common SHARED)
target_sources(open_lmm_common PRIVATE
  agent_id.cpp
  pose_conversion.cpp
  registration.cpp
  registration_log.cpp
  registration_pointcloud.cpp
  pointcloud_utils.cpp
)
target_link_libraries(open_lmm_common
 PUBLIC
  Eigen3::Eigen
  PCL::PCL
 PRIVATE
  open_lmm_profiling
  small_gicp
)
openlmm_set_global_target_properties(open_lmm_common)
