foreach(fixture IN ITEMS valid old_capability empty_capability null_capability)
  add_library(open_lmm_gui_plugin_fixture_${fixture} MODULE
    adapters/gui/fixtures/gui_plugin_fixture.cpp)
  target_include_directories(open_lmm_gui_plugin_fixture_${fixture} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(open_lmm_gui_plugin_fixture_${fixture} PRIVATE
    open_lmm_common)
  openlmm_set_global_target_properties(open_lmm_gui_plugin_fixture_${fixture})
endforeach()
target_compile_definitions(open_lmm_gui_plugin_fixture_old_capability PRIVATE
  OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE=1)
target_compile_definitions(open_lmm_gui_plugin_fixture_empty_capability PRIVATE
  OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE=2)
target_compile_definitions(open_lmm_gui_plugin_fixture_null_capability PRIVATE
  OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE=3)

function(openlmm_add_gui_owner_suite target selector layer module owner invariants)
  add_executable(${target}
    adapters/gui/fault_concurrency/gui_plugin_tests.cpp)
  target_link_libraries(${target} PRIVATE
    open_lmm_gui_core open_lmm_utils nlohmann_json::nlohmann_json)
  openlmm_set_global_target_properties(${target})
  openlmm_add_test(
    NAME ${target} TARGET ${target} COMMAND_ARGS ${selector}
    LAYER ${layer} MODULE ${module} OWNER ${owner}
    INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan tsan)
endfunction()

openlmm_add_gui_owner_suite(
  open_lmm_gui_model_tests model L2 adapters.gui.model GuiModel
  "INV-13;INV-17")
openlmm_add_gui_owner_suite(
  open_lmm_gui_presentation_tests presentation L2
  adapters.gui.presentation GuiPresentationState
  "INV-09;INV-10;INV-13;INV-16")
openlmm_add_gui_owner_suite(
  open_lmm_gui_snapshot_worker_tests worker L5
  adapters.gui.snapshot_worker VisualizationSnapshotWorker
  "INV-08;INV-10;INV-13;INV-17")
openlmm_add_gui_owner_suite(
  open_lmm_gui_runtime_bridge_tests bridge L3
  adapters.gui.runtime_bridge GuiRuntimeBridge
  "INV-08;INV-13;INV-17")
if(TARGET create_scan_context)
  add_dependencies(open_lmm_gui_runtime_bridge_tests create_scan_context)
endif()
if(TARGET create_free_dom)
  add_dependencies(open_lmm_gui_runtime_bridge_tests create_free_dom)
endif()

add_executable(open_lmm_gui_plugin_capability_tests
  adapters/gui/plugin_contract/gui_plugin_capability_tests.cpp)
target_include_directories(open_lmm_gui_plugin_capability_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_gui_plugin_capability_tests PRIVATE
  open_lmm_gui_core open_lmm_utils)
openlmm_set_global_target_properties(open_lmm_gui_plugin_capability_tests)
add_dependencies(open_lmm_gui_plugin_capability_tests
  open_lmm_gui_plugin_fixture_valid
  open_lmm_gui_plugin_fixture_old_capability
  open_lmm_gui_plugin_fixture_empty_capability
  open_lmm_gui_plugin_fixture_null_capability)
openlmm_add_test(
  NAME open_lmm_gui_plugin_capability_tests
  TARGET open_lmm_gui_plugin_capability_tests
  LAYER L2 MODULE adapters.gui OWNER GuiPluginHost
  INVARIANTS INV-13 INV-15 INV-17
  LANES pr SANITIZERS asan-ubsan
  COMMAND_ARGS
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_valid>
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_old_capability>
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_empty_capability>
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_null_capability>)
