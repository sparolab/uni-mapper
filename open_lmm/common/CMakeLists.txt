
add_library(open_lmm_common STATIC)
target_sources(open_lmm_common PRIVATE
  pose_conversion.cpp
  registration.cpp
  pointcloud_utils.cpp
)
target_link_libraries(open_lmm_common PUBLIC 
  Eigen3::Eigen
  PCL::PCL
  gtsam
  small_gicp
  TBB::tbb
  TBB::tbbmalloc
)
openlmm_set_global_target_properties(open_lmm_common)
