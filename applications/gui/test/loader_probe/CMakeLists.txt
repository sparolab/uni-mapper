function(add_gui_probe_fixture name mode)
  add_library(open_lmm_gui_probe_fixture_${name} MODULE fixture.cpp)
  target_compile_features(open_lmm_gui_probe_fixture_${name} PRIVATE cxx_std_20)
  target_compile_definitions(open_lmm_gui_probe_fixture_${name} PRIVATE
    OPEN_LMM_GUI_PROBE_FIXTURE_MODE=${mode})
  target_include_directories(open_lmm_gui_probe_fixture_${name} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include)
  target_link_libraries(open_lmm_gui_probe_fixture_${name} PRIVATE
    open_lmm::client open_lmm::plugin_sdk)
  target_compile_options(open_lmm_gui_probe_fixture_${name} PRIVATE
    $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-fno-gnu-unique>)
  openlmm_gui_enable_warnings(open_lmm_gui_probe_fixture_${name})
  set_target_properties(open_lmm_gui_probe_fixture_${name} PROPERTIES
    CXX_VISIBILITY_PRESET hidden
    VISIBILITY_INLINES_HIDDEN YES)
endfunction()

add_gui_probe_fixture(valid 0)
add_gui_probe_fixture(old_capability 1)
add_gui_probe_fixture(null_capability 2)
add_gui_probe_fixture(empty_capability 3)
add_gui_probe_fixture(wrong_abi 4)
add_gui_probe_fixture(wrong_kind 5)
add_gui_probe_fixture(null_kind 6)
add_gui_probe_fixture(null_name 7)
add_gui_probe_fixture(null_create 8)
add_gui_probe_fixture(null_destroy 9)
add_gui_probe_fixture(create_null 10)
add_gui_probe_fixture(create_throw_standard 11)
add_gui_probe_fixture(create_throw_unknown 12)
add_gui_probe_fixture(entry_null 13)
add_gui_probe_fixture(entry_throw_standard 14)
add_gui_probe_fixture(entry_throw_unknown 15)
add_gui_probe_fixture(missing_symbol 16)
add_gui_probe_fixture(start_result_failure 17)
add_gui_probe_fixture(start_throw_standard 18)
add_gui_probe_fixture(start_throw_unknown 19)

add_executable(open_lmm_gui_loader_public_contract_tests
  probe_tests.cpp
  ${PROJECT_SOURCE_DIR}/src/host/gui_plugin_host.cpp
  ${PROJECT_SOURCE_DIR}/src/host/gui_plugin_module.cpp)
target_compile_features(open_lmm_gui_loader_public_contract_tests PRIVATE
  cxx_std_20)
target_include_directories(open_lmm_gui_loader_public_contract_tests PRIVATE
  ${PROJECT_SOURCE_DIR}/include
  ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(open_lmm_gui_loader_public_contract_tests PRIVATE
  open_lmm::client open_lmm::plugin_sdk Threads::Threads ${CMAKE_DL_LIBS})
get_target_property(open_lmm_gui_loader_probe_links
  open_lmm_gui_loader_public_contract_tests LINK_LIBRARIES)
foreach(forbidden_link IN ITEMS
    open_lmm_gui_core open_lmm::utils open_lmm::runtime
    open_lmm_map_server open_lmm_runtime_service_objects)
  list(FIND open_lmm_gui_loader_probe_links "${forbidden_link}"
    forbidden_link_index)
  if(NOT forbidden_link_index EQUAL -1)
    message(FATAL_ERROR
      "Loader-B public-contract probe links forbidden target: ${forbidden_link}")
  endif()
endforeach()
openlmm_gui_enable_warnings(open_lmm_gui_loader_public_contract_tests)

set(open_lmm_gui_probe_fixture_paths
    $<TARGET_FILE:open_lmm_gui_probe_fixture_valid>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_old_capability>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_null_capability>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_empty_capability>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_wrong_abi>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_wrong_kind>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_null_kind>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_null_name>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_null_create>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_null_destroy>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_create_null>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_create_throw_standard>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_create_throw_unknown>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_entry_null>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_entry_throw_standard>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_entry_throw_unknown>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_missing_symbol>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_start_result_failure>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_start_throw_standard>
    $<TARGET_FILE:open_lmm_gui_probe_fixture_start_throw_unknown>)

function(add_gui_loader_probe_test name mode)
  add_test(NAME ${name}
    COMMAND open_lmm_gui_loader_public_contract_tests
      ${mode} ${open_lmm_gui_probe_fixture_paths})
  set_tests_properties(${name} PROPERTIES
    LABELS
      "layer:L2;module:applications.gui.loader;owner:GuiPluginModule;invariant:INV-13;invariant:INV-15;invariant:INV-17;lane:pr;sanitizer:asan-ubsan"
    TIMEOUT 180)
endfunction()

add_gui_loader_probe_test(
  open_lmm_gui_loader_public_contract_tests public-contract)
add_gui_loader_probe_test(
  open_lmm_gui_plugin_lifetime_tests lifetime)
add_gui_loader_probe_test(
  open_lmm_gui_plugin_stress_tests stress)

set_tests_properties(open_lmm_gui_plugin_stress_tests PROPERTIES
  LABELS
    "layer:L2;module:applications.gui.loader;owner:GuiPluginModule;invariant:INV-13;invariant:INV-15;invariant:INV-17;lane:pr;sanitizer:asan-ubsan"
  TIMEOUT 300)
