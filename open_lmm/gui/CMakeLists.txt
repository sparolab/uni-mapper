add_library(open_lmm_gui_core SHARED
  config_editor.cpp
  gui_controller_bridge.cpp
  gui_event_queue.cpp
  gui_model.cpp
  gui_plugin_host.cpp
  visualization_repository.cpp
  visualization_snapshot_worker.cpp
)
target_link_libraries(open_lmm_gui_core PUBLIC open_lmm_map_server open_lmm_utils)
openlmm_set_global_target_properties(open_lmm_gui_core)

if(OPEN_LMM_BUILD_IRIDESCENCE_GUI)
  add_library(open_lmm_iridescence_gui SHARED iridescence_gui.cpp)
  target_link_libraries(open_lmm_iridescence_gui PRIVATE
    open_lmm_gui_core Iridescence::Iridescence)
  openlmm_set_create_target_properties(open_lmm_iridescence_gui)
endif()
