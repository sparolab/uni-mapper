add_executable(open_lmm_gui_owner_tests gui_owner_tests.cpp)
target_include_directories(open_lmm_gui_owner_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_gui_owner_tests PRIVATE
  open_lmm_gui_core open_lmm::utils Threads::Threads)
openlmm_gui_enable_warnings(open_lmm_gui_owner_tests)

function(openlmm_add_gui_owner_test name selector labels)
  add_test(NAME ${name} COMMAND open_lmm_gui_owner_tests ${selector})
  set_tests_properties(${name} PROPERTIES
    LABELS "${labels};lane:pr"
    TIMEOUT 60
    ENVIRONMENT
      "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:open_lmm::client>:$ENV{LD_LIBRARY_PATH}")
endfunction()

openlmm_add_gui_owner_test(
  open_lmm_gui_model_tests model
  "layer:L2;module:applications.gui.model;owner:GuiModel;invariant:INV-13;invariant:INV-17;sanitizer:asan-ubsan;sanitizer:tsan")
openlmm_add_gui_owner_test(
  open_lmm_gui_presentation_tests presentation
  "layer:L2;module:applications.gui.presentation;owner:GuiPresentationState;invariant:INV-09;invariant:INV-10;invariant:INV-13;invariant:INV-16;sanitizer:asan-ubsan;sanitizer:tsan")
openlmm_add_gui_owner_test(
  open_lmm_gui_snapshot_worker_tests worker
  "layer:L5;module:applications.gui.snapshot_worker;owner:VisualizationSnapshotWorker;invariant:INV-08;invariant:INV-10;invariant:INV-13;invariant:INV-17;sanitizer:asan-ubsan;sanitizer:tsan")
openlmm_add_gui_owner_test(
  open_lmm_gui_runtime_bridge_tests bridge
  "layer:L3;module:applications.gui.runtime_bridge;owner:GuiRuntimeBridge;invariant:INV-08;invariant:INV-13;invariant:INV-17;sanitizer:asan-ubsan;sanitizer:tsan")

add_executable(open_lmm_gui_presentation_resource_tests
  presentation_resource_tests.cpp)
target_include_directories(open_lmm_gui_presentation_resource_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/src/presentation)
target_link_libraries(open_lmm_gui_presentation_resource_tests PRIVATE
  open_lmm_gui_core)
openlmm_gui_enable_warnings(open_lmm_gui_presentation_resource_tests)
add_test(NAME open_lmm_gui_presentation_resource_tests
  COMMAND open_lmm_gui_presentation_resource_tests)
set_tests_properties(open_lmm_gui_presentation_resource_tests PROPERTIES
  LABELS
    "layer:L2;module:applications.gui.presentation;owner:VisualizationRepository;invariant:INV-10;invariant:INV-13;invariant:INV-17;lane:pr;sanitizer:asan-ubsan"
  TIMEOUT 60
  ENVIRONMENT
    "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:open_lmm::client>:$ENV{LD_LIBRARY_PATH}")

foreach(fixture IN ITEMS
    valid old_capability empty_capability null_capability self_closing)
  add_library(open_lmm_gui_plugin_fixture_${fixture} MODULE
    fixtures/gui_plugin_fixture.cpp)
  target_include_directories(open_lmm_gui_plugin_fixture_${fixture} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(open_lmm_gui_plugin_fixture_${fixture} PRIVATE
    open_lmm_gui_core)
  openlmm_gui_enable_warnings(open_lmm_gui_plugin_fixture_${fixture})
endforeach()
target_compile_definitions(open_lmm_gui_plugin_fixture_old_capability PRIVATE
  OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE=1)
target_compile_definitions(open_lmm_gui_plugin_fixture_empty_capability PRIVATE
  OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE=2)
target_compile_definitions(open_lmm_gui_plugin_fixture_null_capability PRIVATE
  OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE=3)
target_compile_definitions(open_lmm_gui_plugin_fixture_self_closing PRIVATE
  OPEN_LMM_GUI_PLUGIN_FIXTURE_MODE=4)

add_executable(open_lmm_gui_plugin_capability_tests
  gui_plugin_capability_tests.cpp)
target_include_directories(open_lmm_gui_plugin_capability_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/src
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_gui_plugin_capability_tests PRIVATE
  open_lmm_gui_core)
openlmm_gui_enable_warnings(open_lmm_gui_plugin_capability_tests)
add_test(NAME open_lmm_gui_plugin_capability_tests
  COMMAND open_lmm_gui_plugin_capability_tests
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_valid>
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_old_capability>
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_empty_capability>
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_null_capability>)
set_tests_properties(open_lmm_gui_plugin_capability_tests PROPERTIES
  LABELS
    "layer:L2;module:applications.gui;owner:GuiPluginHost;invariant:INV-13;invariant:INV-15;invariant:INV-17;lane:pr;sanitizer:asan-ubsan"
  TIMEOUT 60)

add_executable(open_lmm_gui_standalone_smoke_tests standalone_smoke_tests.cpp)
target_include_directories(open_lmm_gui_standalone_smoke_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_features(open_lmm_gui_standalone_smoke_tests PRIVATE cxx_std_20)
openlmm_gui_enable_warnings(open_lmm_gui_standalone_smoke_tests)
add_test(NAME open_lmm_gui_standalone_help_tests
  COMMAND $<TARGET_FILE:open_lmm_gui_app> --help)
add_test(NAME open_lmm_gui_standalone_smoke_tests
  COMMAND open_lmm_gui_standalone_smoke_tests
    $<TARGET_FILE:open_lmm_gui_app>
    $<TARGET_FILE:open_lmm_gui_plugin_fixture_self_closing>)
set_tests_properties(
  open_lmm_gui_standalone_help_tests open_lmm_gui_standalone_smoke_tests
  PROPERTIES
    LABELS "layer:L5;module:applications.gui.standalone;owner:open_lmm_gui;invariant:INV-13;invariant:INV-17;lane:pr;sanitizer:asan-ubsan"
    TIMEOUT 60
    ENVIRONMENT
      "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:open_lmm::client>:$ENV{LD_LIBRARY_PATH}")

if(TARGET open_lmm_iridescence_gui)
  find_program(OPEN_LMM_GUI_XVFB_RUN xvfb-run REQUIRED)
  add_executable(open_lmm_gui_iridescence_smoke_tests
    iridescence_smoke_tests.cpp)
  target_include_directories(open_lmm_gui_iridescence_smoke_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_compile_features(open_lmm_gui_iridescence_smoke_tests PRIVATE
    cxx_std_20)
  target_link_libraries(open_lmm_gui_iridescence_smoke_tests PRIVATE
    open_lmm::plugin_sdk ${CMAKE_DL_LIBS})
  openlmm_gui_enable_warnings(open_lmm_gui_iridescence_smoke_tests)
  add_test(NAME open_lmm_gui_iridescence_smoke_tests
    COMMAND ${OPEN_LMM_GUI_XVFB_RUN} -a
      $<TARGET_FILE:open_lmm_gui_iridescence_smoke_tests>
      $<TARGET_FILE:open_lmm_gui_app>
      $<TARGET_FILE:open_lmm_iridescence_gui>)
  set_tests_properties(open_lmm_gui_iridescence_smoke_tests PROPERTIES
    LABELS
      "layer:L5;module:applications.gui.iridescence;owner:IridescenceGui;invariant:INV-13;invariant:INV-17;lane:pr"
    TIMEOUT 60
    ENVIRONMENT
      "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:open_lmm::client>:$ENV{LD_LIBRARY_PATH}")
endif()

add_subdirectory(loader_probe)

get_filename_component(open_lmm_gui_core_prefix
  "${open_lmm_DIR}/../../.." ABSOLUTE)
if(NOT DEFINED OPEN_LMM_GUI_REPOSITORY_ROOT OR
   OPEN_LMM_GUI_REPOSITORY_ROOT STREQUAL "")
  get_filename_component(OPEN_LMM_GUI_REPOSITORY_ROOT
    "${PROJECT_SOURCE_DIR}/../.." ABSOLUTE)
endif()
function(openlmm_add_gui_package_test name owner)
  add_test(NAME ${name}
    COMMAND ${CMAKE_COMMAND}
      -DOPEN_LMM_GUI_BUILD_DIR=${PROJECT_BINARY_DIR}
      -DOPEN_LMM_GUI_SOURCE_DIR=${PROJECT_SOURCE_DIR}
      -DOPEN_LMM_GUI_REPOSITORY_ROOT=${OPEN_LMM_GUI_REPOSITORY_ROOT}
      -DOPEN_LMM_GUI_CORE_PREFIX=${open_lmm_gui_core_prefix}
      -DOPEN_LMM_GUI_PACKAGE_TEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/${name}
      -DOPEN_LMM_GUI_C_COMPILER=${CMAKE_C_COMPILER}
      -DOPEN_LMM_GUI_CXX_COMPILER=${CMAKE_CXX_COMPILER}
      "-DOPEN_LMM_GUI_CXX_FLAGS=${CMAKE_CXX_FLAGS}"
      -P ${CMAKE_CURRENT_SOURCE_DIR}/package/gui_package_tests.cmake)
  set_tests_properties(${name} PROPERTIES
    LABELS
      "layer:L5;module:applications.gui.package;owner:${owner};invariant:INV-15;invariant:INV-17;lane:pr"
    RESOURCE_LOCK open_lmm_gui_package_install_manifest
    TIMEOUT 180)
endfunction()

openlmm_add_gui_package_test(
  open_lmm_gui_direct_package_consumer_tests OpenLmmGui)
openlmm_add_gui_package_test(
  open_lmm_gui_compat_forwarding_tests Compat-A)
openlmm_add_gui_package_test(
  open_lmm_gui_install_ownership_tests GuiInstallManifest)
