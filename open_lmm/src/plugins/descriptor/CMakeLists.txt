if(OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT)
  add_subdirectory(scan_context)
endif()
if(OPEN_LMM_BUILD_DESCRIPTOR_SOLID)
  add_subdirectory(solid)
endif()

add_library(descriptor_kdtree STATIC)
target_sources(descriptor_kdtree PRIVATE
  ${PROJECT_SOURCE_DIR}/src/domain/loop_detection/database/database_kdtree.cpp
)
target_link_libraries(descriptor_kdtree PRIVATE
  Eigen3::Eigen
  PCL::PCL
  nanoflann::nanoflann
  open_lmm_descriptor
)
openlmm_set_global_target_properties(descriptor_kdtree)

add_library(descriptor_factory INTERFACE)
target_link_libraries(descriptor_factory INTERFACE
  descriptor_kdtree
)
