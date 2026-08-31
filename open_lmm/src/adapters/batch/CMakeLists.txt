# The batch launcher is deliberately a core-package executable. ROS component
# construction never reuses it and never starts a batch pipeline implicitly.
add_executable(open_lmm_batch main.cpp)
target_link_libraries(open_lmm_batch PRIVATE
  open_lmm_client
  open_lmm_utils
)
openlmm_set_global_target_properties(open_lmm_batch)
set_target_properties(open_lmm_batch PROPERTIES
  INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
)
