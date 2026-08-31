add_executable(open_lmm_config_schema_tests
  config/schema/contract/config_schema_tests.cpp)
target_compile_definitions(open_lmm_config_schema_tests PRIVATE
  OPEN_LMM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(open_lmm_config_schema_tests PRIVATE open_lmm_utils)
openlmm_set_global_target_properties(open_lmm_config_schema_tests)
openlmm_add_test(
  NAME open_lmm_config_schema_tests TARGET open_lmm_config_schema_tests
  LAYER L2 MODULE config.schema OWNER ConfigSchema
  INVARIANTS INV-03 INV-12 INV-16 LANES pr SANITIZERS asan-ubsan)

foreach(fixture IN ITEMS
    valid wrong_abi null_factory missing_destroy no_entry null_kind null_name
    empty_capability null_capability create_throw entry_throw null_entry)
  add_library(open_lmm_plugin_fixture_${fixture} MODULE
    plugins/fixtures/plugin_fixture.cpp)
  target_include_directories(open_lmm_plugin_fixture_${fixture} PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR})
  target_link_libraries(open_lmm_plugin_fixture_${fixture} PRIVATE
    open_lmm_common)
  openlmm_set_global_target_properties(open_lmm_plugin_fixture_${fixture})
endforeach()
target_compile_definitions(open_lmm_plugin_fixture_wrong_abi PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=1)
target_compile_definitions(open_lmm_plugin_fixture_null_factory PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=2)
target_compile_definitions(open_lmm_plugin_fixture_missing_destroy PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=3)
target_compile_definitions(open_lmm_plugin_fixture_no_entry PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=4)
target_compile_definitions(open_lmm_plugin_fixture_null_kind PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=5)
target_compile_definitions(open_lmm_plugin_fixture_null_name PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=6)
target_compile_definitions(open_lmm_plugin_fixture_empty_capability PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=7)
target_compile_definitions(open_lmm_plugin_fixture_null_capability PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=8)
target_compile_definitions(open_lmm_plugin_fixture_create_throw PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=9)
target_compile_definitions(open_lmm_plugin_fixture_entry_throw PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=10)
target_compile_definitions(open_lmm_plugin_fixture_null_entry PRIVATE
  OPEN_LMM_PLUGIN_FIXTURE_MODE=11)

add_executable(open_lmm_plugin_loader_contract_tests
  plugins/host/contract/plugin_loader_contract_tests.cpp)
target_include_directories(open_lmm_plugin_loader_contract_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_plugin_loader_contract_tests PRIVATE
  open_lmm_common open_lmm_utils)
foreach(fixture IN ITEMS
    valid wrong_abi null_factory missing_destroy no_entry null_kind null_name
    empty_capability null_capability create_throw entry_throw null_entry)
  add_dependencies(open_lmm_plugin_loader_contract_tests
    open_lmm_plugin_fixture_${fixture})
endforeach()
openlmm_set_global_target_properties(open_lmm_plugin_loader_contract_tests)
openlmm_add_test(
  NAME open_lmm_plugin_loader_contract_tests
  TARGET open_lmm_plugin_loader_contract_tests
  LAYER L2 MODULE plugins.host OWNER PluginLoader
  INVARIANTS INV-12 INV-15 INV-17
  LANES pr SANITIZERS asan-ubsan
  COMMAND_ARGS
    $<TARGET_FILE:open_lmm_plugin_fixture_valid>
    $<TARGET_FILE:open_lmm_plugin_fixture_wrong_abi>
    $<TARGET_FILE:open_lmm_plugin_fixture_null_factory>
    $<TARGET_FILE:open_lmm_plugin_fixture_missing_destroy>
    $<TARGET_FILE:open_lmm_plugin_fixture_no_entry>
    $<TARGET_FILE:open_lmm_plugin_fixture_null_kind>
    $<TARGET_FILE:open_lmm_plugin_fixture_null_name>
    $<TARGET_FILE:open_lmm_plugin_fixture_empty_capability>
    $<TARGET_FILE:open_lmm_plugin_fixture_null_capability>
    $<TARGET_FILE:open_lmm_plugin_fixture_create_throw>
    $<TARGET_FILE:open_lmm_plugin_fixture_entry_throw>
    $<TARGET_FILE:open_lmm_plugin_fixture_null_entry>)

add_executable(open_lmm_algorithm_factory_contract_tests
  plugins/host/contract/algorithm_factory_contract_tests.cpp)
target_link_libraries(open_lmm_algorithm_factory_contract_tests PRIVATE
  open_lmm_plugin_host_objects open_lmm_common open_lmm_utils
  open_lmm_data_loader open_lmm_loop_detector open_lmm_backend_optimizer
  open_lmm_dynamic_remover Eigen3::Eigen PCL::PCL nanoflann::nanoflann)
openlmm_set_global_target_properties(open_lmm_algorithm_factory_contract_tests)
openlmm_add_test(
  NAME open_lmm_algorithm_factory_contract_tests
  TARGET open_lmm_algorithm_factory_contract_tests
  LAYER L2 MODULE plugins.host OWNER AlgorithmFactory
  INVARIANTS INV-12 INV-15
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_plugin_abi_tests
  plugins/host/integration/plugin_abi_tests.cpp)
target_include_directories(open_lmm_plugin_abi_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_plugin_abi_tests PRIVATE
  open_lmm_map_server
  open_lmm_common
  open_lmm_utils
  open_lmm_loop_detector
  Eigen3::Eigen
  PCL::PCL
  nanoflann::nanoflann)
openlmm_set_global_target_properties(open_lmm_plugin_abi_tests)
set(_open_lmm_built_in_plugin_test_arguments)
foreach(plugin_target IN ITEMS
    create_scan_context create_solid create_hmm_mos create_dufomap create_otd
    create_free_dom create_erasor)
  if(TARGET ${plugin_target})
    add_dependencies(open_lmm_plugin_abi_tests ${plugin_target})
    list(APPEND _open_lmm_built_in_plugin_test_arguments
      "${plugin_target}=$<TARGET_FILE:${plugin_target}>")
  endif()
endforeach()
openlmm_add_test(
  NAME open_lmm_plugin_abi_tests TARGET open_lmm_plugin_abi_tests
  LAYER L3 MODULE plugins.host OWNER AlgorithmFactory
  INVARIANTS INV-12 INV-15 LANES pr SANITIZERS asan-ubsan
  COMMAND_ARGS ${_open_lmm_built_in_plugin_test_arguments})

add_executable(open_lmm_plugin_selection_tests
  config/domain/contract/plugin_selection_tests.cpp)
target_link_libraries(open_lmm_plugin_selection_tests PRIVATE
  open_lmm_algorithm_config)
openlmm_set_global_target_properties(open_lmm_plugin_selection_tests)
set(_open_lmm_disabled_plugin_test_arguments)
if(NOT OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT)
  list(APPEND _open_lmm_disabled_plugin_test_arguments
    descriptor:scan_context)
endif()
if(NOT OPEN_LMM_BUILD_DESCRIPTOR_SOLID)
  list(APPEND _open_lmm_disabled_plugin_test_arguments descriptor:solid)
endif()
if(NOT OPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS)
  list(APPEND _open_lmm_disabled_plugin_test_arguments online:hmm_mos)
endif()
if(NOT OPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP)
  list(APPEND _open_lmm_disabled_plugin_test_arguments online:dufomap)
endif()
if(NOT OPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD)
  list(APPEND _open_lmm_disabled_plugin_test_arguments online:otd)
endif()
if(NOT OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM)
  list(APPEND _open_lmm_disabled_plugin_test_arguments offline:free_dom)
endif()
if(NOT OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR)
  list(APPEND _open_lmm_disabled_plugin_test_arguments offline:erasor)
endif()
openlmm_add_test(
  NAME open_lmm_plugin_selection_tests TARGET open_lmm_plugin_selection_tests
  LAYER L2 MODULE config.domain OWNER AlgorithmConfig
  INVARIANTS INV-12 INV-15 LANES pr SANITIZERS asan-ubsan
  COMMAND_ARGS ${_open_lmm_disabled_plugin_test_arguments})
