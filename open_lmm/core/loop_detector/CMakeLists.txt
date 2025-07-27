
add_subdirectory(descriptor_factory)

add_library(open_lmm_loop_detector STATIC)
target_sources(open_lmm_loop_detector PRIVATE
  loop_detector_base.cpp
  loop_detector_kdtree.cpp
)
target_link_libraries(open_lmm_loop_detector
  Eigen3::Eigen
  open_lmm_common
  kiss_matcher::kiss_matcher_core
  descriptor_factory
  nanoflann::nanoflann
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_loop_detector)