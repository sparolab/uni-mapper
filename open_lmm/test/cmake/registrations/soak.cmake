add_library(open_lmm_soak_support STATIC
  support/soak/soak_metrics.cpp
  support/soak/owner_stress_support.cpp)
target_include_directories(open_lmm_soak_support PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_soak_support PUBLIC
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_soak_support)

set(open_lmm_soak_sanitizer_name "none")
if(OPEN_LMM_ENABLE_ASAN_UBSAN)
  set(open_lmm_soak_sanitizer_name "asan-ubsan")
elseif(OPEN_LMM_ENABLE_TSAN)
  set(open_lmm_soak_sanitizer_name "tsan")
endif()

set(open_lmm_stress_iterations 100)
set(open_lmm_stress_warmup 10)
set(open_lmm_stress_profile fast)
if(NOT open_lmm_soak_sanitizer_name STREQUAL "none")
  set(open_lmm_stress_iterations 50)
  set(open_lmm_stress_warmup 5)
  set(open_lmm_stress_profile sanitizer)
endif()

add_executable(open_lmm_soak_metrics_contract_tests
  soak/contract/soak_metrics_tests.cpp)
target_compile_definitions(open_lmm_soak_metrics_contract_tests PRIVATE
  OPEN_LMM_SOAK_SCHEMA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/soak/schema")
target_link_libraries(open_lmm_soak_metrics_contract_tests PRIVATE
  open_lmm_soak_support)
openlmm_set_global_target_properties(open_lmm_soak_metrics_contract_tests)
openlmm_add_test(
  NAME open_lmm_soak_metrics_contract_tests
  TARGET open_lmm_soak_metrics_contract_tests
  LAYER L2 MODULE workflows.soak OWNER SoakMetrics
  INVARIANTS INV-16 INV-18
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_soak_aggregate
  soak/soak_aggregate.cpp)
target_link_libraries(open_lmm_soak_aggregate PRIVATE
  open_lmm_soak_support)
openlmm_set_global_target_properties(open_lmm_soak_aggregate)

add_executable(open_lmm_runtime_lifecycle_stress_tests
  soak/runtime_lifecycle_stress.cpp)
target_compile_definitions(open_lmm_runtime_lifecycle_stress_tests PRIVATE
  OPEN_LMM_SOAK_SANITIZER_NAME="${open_lmm_soak_sanitizer_name}")
target_link_libraries(open_lmm_runtime_lifecycle_stress_tests PRIVATE
  open_lmm_soak_support
  open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_runtime_lifecycle_stress_tests)
openlmm_add_test(
  NAME open_lmm_runtime_lifecycle_stress_tests
  TARGET open_lmm_runtime_lifecycle_stress_tests
  COMMAND_ARGS --iterations ${open_lmm_stress_iterations}
    --warmup ${open_lmm_stress_warmup} --seed 104729
    --profile ${open_lmm_stress_profile}
    --scenario runtime-lifecycle
  LAYER L6 MODULE workflows.soak OWNER RuntimeService
  INVARIANTS INV-01 INV-02 INV-04 INV-07 INV-08 INV-09 INV-16 INV-17 INV-18
  LANES pr SANITIZERS asan-ubsan tsan
  TIMEOUT 300)

add_executable(open_lmm_runtime_concurrency_stress_tests
  soak/runtime_concurrency_stress.cpp)
target_compile_definitions(open_lmm_runtime_concurrency_stress_tests PRIVATE
  OPEN_LMM_SOAK_SANITIZER_NAME="${open_lmm_soak_sanitizer_name}")
target_link_libraries(open_lmm_runtime_concurrency_stress_tests PRIVATE
  open_lmm_soak_support
  open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_runtime_concurrency_stress_tests)
openlmm_add_test(
  NAME open_lmm_runtime_concurrency_stress_tests
  TARGET open_lmm_runtime_concurrency_stress_tests
  COMMAND_ARGS --iterations ${open_lmm_stress_iterations}
    --warmup ${open_lmm_stress_warmup} --seed 179424673
    --profile ${open_lmm_stress_profile}
  LAYER L6 MODULE workflows.soak OWNER RuntimeService
  INVARIANTS INV-01 INV-02 INV-04 INV-07 INV-08 INV-16 INV-17 INV-18
  LANES pr SANITIZERS asan-ubsan tsan
  TIMEOUT 300)

add_executable(open_lmm_transaction_fault_stress_tests
  soak/transaction_fault_stress.cpp)
target_compile_definitions(open_lmm_transaction_fault_stress_tests PRIVATE
  OPEN_LMM_SOAK_SANITIZER_NAME="${open_lmm_soak_sanitizer_name}")
target_link_libraries(open_lmm_transaction_fault_stress_tests PRIVATE
  open_lmm_soak_support
  open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_transaction_fault_stress_tests)
openlmm_add_test(
  NAME open_lmm_transaction_fault_stress_tests
  TARGET open_lmm_transaction_fault_stress_tests
  COMMAND_ARGS --iterations ${open_lmm_stress_iterations}
    --warmup ${open_lmm_stress_warmup} --seed 15485863
    --profile ${open_lmm_stress_profile}
  LAYER L6 MODULE workflows.soak OWNER FileSetTransaction
  INVARIANTS INV-03 INV-04 INV-07 INV-16 INV-17 INV-18
  LANES pr SANITIZERS asan-ubsan
  TIMEOUT 300)

add_executable(open_lmm_visualization_stress_tests
  soak/visualization_stress.cpp)
target_compile_definitions(open_lmm_visualization_stress_tests PRIVATE
  OPEN_LMM_SOAK_SANITIZER_NAME="${open_lmm_soak_sanitizer_name}")
target_link_libraries(open_lmm_visualization_stress_tests PRIVATE
  open_lmm_soak_support
  open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_visualization_stress_tests)
openlmm_add_test(
  NAME open_lmm_visualization_stress_tests
  TARGET open_lmm_visualization_stress_tests
  COMMAND_ARGS --iterations ${open_lmm_stress_iterations}
    --warmup ${open_lmm_stress_warmup} --seed 32452843
    --profile ${open_lmm_stress_profile}
  LAYER L6 MODULE workflows.soak OWNER VisualizationProjector
  INVARIANTS INV-10 INV-11 INV-16 INV-17 INV-18
  LANES pr SANITIZERS asan-ubsan tsan
  TIMEOUT 300)

add_executable(open_lmm_plugin_stress_tests
  soak/plugin_stress.cpp)
target_include_directories(open_lmm_plugin_stress_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_definitions(open_lmm_plugin_stress_tests PRIVATE
  OPEN_LMM_SOAK_SANITIZER_NAME="${open_lmm_soak_sanitizer_name}")
target_link_libraries(open_lmm_plugin_stress_tests PRIVATE
  open_lmm_soak_support
  open_lmm_common
  open_lmm_utils)
foreach(fixture IN ITEMS
    valid wrong_abi null_factory missing_destroy no_entry null_kind null_name
    empty_capability null_capability create_throw entry_throw null_entry)
  add_dependencies(open_lmm_plugin_stress_tests
    open_lmm_plugin_fixture_${fixture})
endforeach()
if(TARGET create_scan_context)
  add_dependencies(open_lmm_plugin_stress_tests create_scan_context)
  set(open_lmm_soak_builtin_plugin_arg $<TARGET_FILE:create_scan_context>)
else()
  set(open_lmm_soak_builtin_plugin_arg)
endif()
openlmm_set_global_target_properties(open_lmm_plugin_stress_tests)
openlmm_add_test(
  NAME open_lmm_plugin_stress_tests
  TARGET open_lmm_plugin_stress_tests
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
    $<TARGET_FILE:open_lmm_plugin_fixture_null_entry>
    ${open_lmm_soak_builtin_plugin_arg}
    --iterations ${open_lmm_stress_iterations}
    --warmup ${open_lmm_stress_warmup} --seed 49979687
    --profile ${open_lmm_stress_profile}
  LAYER L6 MODULE workflows.soak OWNER PluginLoader
  INVARIANTS INV-12 INV-15 INV-16 INV-17 INV-18
  LANES pr SANITIZERS asan-ubsan
  TIMEOUT 300)

add_executable(open_lmm_resource_stress_tests
  soak/resource_stress.cpp)
target_compile_definitions(open_lmm_resource_stress_tests PRIVATE
  OPEN_LMM_SOAK_SANITIZER_NAME="${open_lmm_soak_sanitizer_name}")
target_link_libraries(open_lmm_resource_stress_tests PRIVATE
  open_lmm_soak_support
  open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_resource_stress_tests)
openlmm_add_test(
  NAME open_lmm_resource_stress_tests
  TARGET open_lmm_resource_stress_tests
  COMMAND_ARGS --iterations ${open_lmm_stress_iterations}
    --warmup ${open_lmm_stress_warmup} --seed 67867967
    --profile ${open_lmm_stress_profile}
  LAYER L6 MODULE workflows.soak OWNER ResourceGovernor
  INVARIANTS INV-09 INV-16 INV-17 INV-18
  LANES pr SANITIZERS asan-ubsan tsan
  TIMEOUT 300)

add_executable(open_lmm_config_apply_stress_tests
  soak/config_apply_stress.cpp)
target_compile_definitions(open_lmm_config_apply_stress_tests PRIVATE
  OPEN_LMM_SOAK_SANITIZER_NAME="${open_lmm_soak_sanitizer_name}")
target_link_libraries(open_lmm_config_apply_stress_tests PRIVATE
  open_lmm_soak_support
  open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_config_apply_stress_tests)
openlmm_add_test(
  NAME open_lmm_config_apply_stress_tests
  TARGET open_lmm_config_apply_stress_tests
  COMMAND_ARGS --iterations ${open_lmm_stress_iterations}
    --warmup ${open_lmm_stress_warmup} --seed 86028121
    --profile ${open_lmm_stress_profile}
  LAYER L6 MODULE workflows.soak OWNER ConfigTransaction
  INVARIANTS INV-03 INV-04 INV-07 INV-16 INV-17 INV-18
  LANES pr SANITIZERS asan-ubsan tsan
  TIMEOUT 300)
