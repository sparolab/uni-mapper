function(openlmm_add_gui_object_owner target)
  add_library(${target} OBJECT ${ARGN})
  target_link_libraries(${target} PRIVATE
    open_lmm_client
    open_lmm_utils)
  target_include_directories(${target} PRIVATE
    $<BUILD_INTERFACE:${nlohmann_json_SOURCE_DIR}/include>)
  openlmm_set_global_target_properties(${target})
endfunction()

openlmm_add_gui_object_owner(open_lmm_gui_model_objects
  model/gui_event_queue.cpp
  model/gui_model.cpp)

openlmm_add_gui_object_owner(open_lmm_gui_adapter_objects
  config_editor.cpp
  gui_controller_bridge.cpp
  gui_plugin_host.cpp
  gui_runtime_host.cpp)

openlmm_add_gui_object_owner(open_lmm_gui_presentation_objects
  presentation/map_presentation_state.cpp
  presentation/visualization_repository.cpp
  presentation/visualization_snapshot_worker.cpp)

add_library(open_lmm_gui_core SHARED
  $<TARGET_OBJECTS:open_lmm_gui_model_objects>
  $<TARGET_OBJECTS:open_lmm_gui_adapter_objects>
  $<TARGET_OBJECTS:open_lmm_gui_presentation_objects>)
target_link_libraries(open_lmm_gui_core
  PUBLIC open_lmm_client
  PRIVATE open_lmm_utils)
target_include_directories(open_lmm_gui_core PUBLIC
  $<BUILD_INTERFACE:${nlohmann_json_SOURCE_DIR}/include>)
openlmm_set_global_target_properties(open_lmm_gui_core)

if(OPEN_LMM_BUILD_IRIDESCENCE_GUI)
  add_library(open_lmm_iridescence_gui SHARED
    iridescence/iridescence_gui.cpp)
  target_link_libraries(open_lmm_iridescence_gui PRIVATE
    open_lmm_gui_core Iridescence::Iridescence)
  openlmm_set_create_target_properties(open_lmm_iridescence_gui)
endif()
