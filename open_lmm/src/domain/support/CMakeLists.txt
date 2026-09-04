add_library(open_lmm_domain_support_objects OBJECT
  pose_conversion.cpp
  registration.cpp
  registration_pointcloud.cpp
  pointcloud_utils.cpp)
target_link_libraries(open_lmm_domain_support_objects PRIVATE
  open_lmm_contracts
  open_lmm_utils
  open_lmm_profiling
  Eigen3::Eigen
  PCL::PCL
  small_gicp)
openlmm_set_global_target_properties(open_lmm_domain_support_objects)

# Stable compatibility DSO for shared domain-support symbols.
add_library(open_lmm_common SHARED
  $<TARGET_OBJECTS:open_lmm_domain_support_objects>)
target_link_libraries(open_lmm_common
 PUBLIC
  open_lmm_contracts
  Eigen3::Eigen
  PCL::PCL
 PRIVATE
  open_lmm_utils
  open_lmm_profiling
  small_gicp)
openlmm_set_global_target_properties(open_lmm_common)
