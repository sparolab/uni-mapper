
add_subdirectory(scan_context)
add_subdirectory(solid)

add_library(descriptor_kdtree STATIC)
target_sources(descriptor_kdtree PRIVATE
  database_kdtree.cpp
)
target_link_libraries(descriptor_kdtree PUBLIC
  scan_context
  solid
  nanoflann::nanoflann
)
openlmm_set_global_target_properties(descriptor_kdtree)