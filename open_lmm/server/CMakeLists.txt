
add_subdirectory(map_aligner)
add_subdirectory(map_updater)


add_library(open_lmm_map_server STATIC)
target_sources(open_lmm_map_server PRIVATE
  map_server.cpp
)
target_link_libraries(open_lmm_map_server PUBLIC 
  Eigen3::Eigen
  open_lmm_map_aligner
  open_lmm_map_updater
)
openlmm_set_global_target_properties(open_lmm_map_server)