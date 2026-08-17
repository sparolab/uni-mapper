
add_subdirectory(descriptor_factory)

add_library(open_lmm_loop_detector STATIC)
target_sources(open_lmm_loop_detector PRIVATE
  loop_detector_base.cpp
  loop_detector_kdtree.cpp
  kiss_alignment_proposer.cpp
  descriptor_alignment_proposer.cpp
  map_alignment_refiner.cpp
  map_alignment_coordinator.cpp
)
target_link_libraries(open_lmm_loop_detector
  open_lmm_profiling
  Eigen3::Eigen
  open_lmm_common
  GkCM::GkCM
  kiss_matcher::kiss_matcher_core
  descriptor_factory
  nanoflann::nanoflann
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_loop_detector)
