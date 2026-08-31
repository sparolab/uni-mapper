add_library(open_lmm_map_server SHARED)
target_sources(open_lmm_map_server PRIVATE
  $<TARGET_OBJECTS:open_lmm_plugin_host_objects>
  $<TARGET_OBJECTS:open_lmm_config_application_objects>
  $<TARGET_OBJECTS:open_lmm_storage_objects>
  $<TARGET_OBJECTS:open_lmm_runtime_state_objects>
  $<TARGET_OBJECTS:open_lmm_runtime_resources_objects>
  $<TARGET_OBJECTS:open_lmm_runtime_execution_objects>
  $<TARGET_OBJECTS:open_lmm_visualization_projection_objects>
  $<TARGET_OBJECTS:open_lmm_runtime_control_objects>
  $<TARGET_OBJECTS:open_lmm_runtime_service_objects>
  $<TARGET_OBJECTS:open_lmm_runtime_composition_objects>
  $<TARGET_OBJECTS:open_lmm_batch_compat_objects>)
target_link_libraries(open_lmm_map_server
 PUBLIC
  Eigen3::Eigen
  PCL::PCL
  gtsam
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
 PRIVATE
  "$<BUILD_INTERFACE:$<TARGET_FILE:open_lmm_backend_optimizer>>"
  open_lmm_profiling
  nlohmann_json::nlohmann_json
  "$<BUILD_INTERFACE:$<TARGET_FILE:open_lmm_loop_detector>>"
  "$<BUILD_INTERFACE:$<TARGET_FILE:open_lmm_descriptor>>"
  "$<BUILD_INTERFACE:$<TARGET_FILE:open_lmm_data_loader>>"
  "$<BUILD_INTERFACE:$<TARGET_FILE:open_lmm_dynamic_remover>>"
)
add_dependencies(open_lmm_map_server
  open_lmm_backend_optimizer
  open_lmm_loop_detector
  open_lmm_descriptor
  open_lmm_data_loader
  open_lmm_dynamic_remover)
# Executables linking the façade must let the linker resolve its private
# DT_NEEDED libraries from the build tree without publishing them as targets.
target_link_options(open_lmm_map_server INTERFACE
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_backend_optimizer>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_loop_detector>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_data_loader>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_dynamic_remover>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_descriptor>>")
openlmm_set_global_target_properties(open_lmm_map_server)
