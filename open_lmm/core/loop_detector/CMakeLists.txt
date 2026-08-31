
add_subdirectory(descriptor_factory)

add_library(open_lmm_loop_detector SHARED)
target_sources(open_lmm_loop_detector PRIVATE
  alignment_map_builder.cpp
  loop_detector_base.cpp
  loop_detector_kdtree.cpp
  kiss_alignment_proposer.cpp
  descriptor_alignment_proposer.cpp
  map_alignment_refiner.cpp
  map_alignment_coordinator.cpp
)
target_link_libraries(open_lmm_loop_detector
 PUBLIC
  Eigen3::Eigen
  PCL::PCL
  open_lmm_common
  open_lmm_descriptor
  open_lmm_algorithm_config
  open_lmm_utils
 PRIVATE
  open_lmm_alignment
  open_lmm_profiling
  GkCM::GkCM
  kiss_matcher::kiss_matcher_core
  descriptor_factory
  nanoflann::nanoflann
  tqdmcpp::tqdmcpp
)
openlmm_set_global_target_properties(open_lmm_loop_detector)
