
add_library(open_lmm_contracts SHARED
  agent_id.cpp
  runtime_contracts.cpp
)
openlmm_set_global_target_properties(open_lmm_contracts)

# The C++ DSO loader is runtime implementation, not part of the lightweight
# contracts surface.  Keep it in a non-exported library so contracts-only
# consumers never inherit libdl or plugin-host headers.
add_library(open_lmm_plugin_host SHARED plugin_host_v2.cpp)
target_link_libraries(open_lmm_plugin_host
  PUBLIC open_lmm_contracts
  PRIVATE ${CMAKE_DL_LIBS})
openlmm_set_global_target_properties(open_lmm_plugin_host)

add_library(open_lmm_common SHARED)
target_sources(open_lmm_common PRIVATE
  pose_conversion.cpp
  registration.cpp
  registration_log.cpp
  registration_pointcloud.cpp
  pointcloud_utils.cpp
)
target_link_libraries(open_lmm_common
 PUBLIC
  open_lmm_contracts
  Eigen3::Eigen
  PCL::PCL
 PRIVATE
  open_lmm_profiling
  small_gicp
)
openlmm_set_global_target_properties(open_lmm_common)
