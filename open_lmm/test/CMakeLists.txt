add_executable(open_lmm_safety_regression_tests
  safety_regression_tests.cpp
)
target_link_libraries(open_lmm_safety_regression_tests PRIVATE
  open_lmm_map_server
  open_lmm_data_loader
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
)
openlmm_set_global_target_properties(open_lmm_safety_regression_tests)
target_link_options(open_lmm_safety_regression_tests PRIVATE
  LINKER:--no-as-needed
)
add_test(NAME open_lmm_safety_regression_tests
         COMMAND open_lmm_safety_regression_tests)

add_executable(open_lmm_algorithm_contract_tests
  algorithm_contract_tests.cpp
)
target_link_libraries(open_lmm_algorithm_contract_tests PRIVATE
  open_lmm_data_loader
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
)
openlmm_set_global_target_properties(open_lmm_algorithm_contract_tests)
add_test(NAME open_lmm_algorithm_contract_tests
         COMMAND open_lmm_algorithm_contract_tests)

add_executable(open_lmm_descriptor_engine_tests
  descriptor_engine_tests.cpp
)
target_link_libraries(open_lmm_descriptor_engine_tests PRIVATE
  open_lmm_descriptor
)
openlmm_set_global_target_properties(open_lmm_descriptor_engine_tests)
add_test(NAME open_lmm_descriptor_engine_tests
         COMMAND open_lmm_descriptor_engine_tests)

add_executable(open_lmm_descriptor_database_tests
  descriptor_database_tests.cpp
)
target_link_libraries(open_lmm_descriptor_database_tests PRIVATE
  descriptor_kdtree
  open_lmm_descriptor
)
openlmm_set_global_target_properties(open_lmm_descriptor_database_tests)
add_test(NAME open_lmm_descriptor_database_tests
         COMMAND open_lmm_descriptor_database_tests)

add_executable(open_lmm_alignment_decision_policy_tests
  alignment_decision_policy_tests.cpp
)
target_link_libraries(open_lmm_alignment_decision_policy_tests PRIVATE
  open_lmm_alignment
)
openlmm_set_global_target_properties(open_lmm_alignment_decision_policy_tests)
add_test(NAME open_lmm_alignment_decision_policy_tests
         COMMAND open_lmm_alignment_decision_policy_tests)

add_executable(open_lmm_alignment_contract_fixture_tests
  alignment_contract_fixture_tests.cpp
)
target_link_libraries(open_lmm_alignment_contract_fixture_tests PRIVATE
  open_lmm_alignment
)
openlmm_set_global_target_properties(open_lmm_alignment_contract_fixture_tests)
add_test(NAME open_lmm_alignment_contract_fixture_tests
         COMMAND open_lmm_alignment_contract_fixture_tests)

add_executable(open_lmm_agent_id_tests agent_id_tests.cpp)
target_link_libraries(open_lmm_agent_id_tests PRIVATE
  open_lmm_common
  open_lmm_utils
  gtsam)
openlmm_set_global_target_properties(open_lmm_agent_id_tests)
add_test(NAME open_lmm_agent_id_tests COMMAND open_lmm_agent_id_tests)

add_executable(open_lmm_artifact_compare artifact_compare.cpp)
target_link_libraries(open_lmm_artifact_compare PRIVATE
  PCL::PCL
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_artifact_compare)

add_executable(open_lmm_pipeline_controller_tests
  pipeline_controller_tests.cpp
)
target_link_libraries(open_lmm_pipeline_controller_tests PRIVATE
  open_lmm_map_server
  open_lmm_loop_detector
)
openlmm_set_global_target_properties(open_lmm_pipeline_controller_tests)
add_test(NAME open_lmm_pipeline_controller_tests
         COMMAND open_lmm_pipeline_controller_tests)

add_executable(open_lmm_runtime_service_tests runtime_service_tests.cpp)
target_link_libraries(open_lmm_runtime_service_tests PRIVATE
  open_lmm_map_server
  open_lmm_client)
if(TARGET create_scan_context)
  add_dependencies(open_lmm_runtime_service_tests create_scan_context)
endif()
if(TARGET create_free_dom)
  add_dependencies(open_lmm_runtime_service_tests create_free_dom)
endif()
openlmm_set_global_target_properties(open_lmm_runtime_service_tests)
add_test(NAME open_lmm_runtime_service_tests
         COMMAND open_lmm_runtime_service_tests)

add_executable(open_lmm_bounded_executor_tests bounded_executor_tests.cpp)
target_link_libraries(open_lmm_bounded_executor_tests PRIVATE
  open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_bounded_executor_tests)
add_test(NAME open_lmm_bounded_executor_tests
         COMMAND open_lmm_bounded_executor_tests)

add_executable(open_lmm_runtime_service_verify runtime_service_verify.cpp)
target_link_libraries(open_lmm_runtime_service_verify PRIVATE
  open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_runtime_service_verify)

add_executable(open_lmm_runtime_transaction_tests
  runtime_transaction_tests.cpp
)
target_link_libraries(open_lmm_runtime_transaction_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_runtime_transaction_tests)
add_test(NAME open_lmm_runtime_transaction_tests
         COMMAND open_lmm_runtime_transaction_tests)

add_executable(open_lmm_config_transaction_tests
  config_transaction_tests.cpp
)
target_link_libraries(open_lmm_config_transaction_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_config_transaction_tests)
add_test(NAME open_lmm_config_transaction_tests
         COMMAND open_lmm_config_transaction_tests)

add_executable(open_lmm_stage_executor_fixture_tests
  stage_executor_fixture_tests.cpp
)
target_link_libraries(open_lmm_stage_executor_fixture_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_stage_executor_fixture_tests)
add_test(NAME open_lmm_stage_executor_fixture_tests
         COMMAND open_lmm_stage_executor_fixture_tests)

add_executable(open_lmm_orchestration_executor_fixture_tests
  orchestration_executor_fixture_tests.cpp
)
target_link_libraries(open_lmm_orchestration_executor_fixture_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(
  open_lmm_orchestration_executor_fixture_tests)
add_test(NAME open_lmm_orchestration_executor_fixture_tests
         COMMAND open_lmm_orchestration_executor_fixture_tests)

add_executable(open_lmm_save_executor_tests save_executor_tests.cpp)
target_link_libraries(open_lmm_save_executor_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_save_executor_tests)
add_test(NAME open_lmm_save_executor_tests COMMAND open_lmm_save_executor_tests)

add_executable(open_lmm_runtime_bootstrapper_tests
  runtime_bootstrapper_tests.cpp
)
target_link_libraries(open_lmm_runtime_bootstrapper_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_runtime_bootstrapper_tests)
add_test(NAME open_lmm_runtime_bootstrapper_tests
         COMMAND open_lmm_runtime_bootstrapper_tests)
add_executable(open_lmm_execution_spec_tests
  execution_spec_tests.cpp
)
target_link_libraries(open_lmm_execution_spec_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_execution_spec_tests)
add_test(NAME open_lmm_execution_spec_tests
         COMMAND open_lmm_execution_spec_tests)

# Dataset-backed ordered replay verification. This is intentionally not a
# CTest because it requires the test1/test2 dataset configured by the caller.
add_executable(open_lmm_replay_verify replay_verify.cpp)
target_link_libraries(open_lmm_replay_verify PRIVATE open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_replay_verify)

add_executable(open_lmm_controller_concurrency_tests
  controller_concurrency_tests.cpp
  ../server/pipeline_controller.cpp
  ../server/artifact_repository.cpp
  ../server/stage_runner.cpp
)
target_link_libraries(open_lmm_controller_concurrency_tests PRIVATE
  open_lmm_common
  open_lmm_utils
  open_lmm_profiling
  Eigen3::Eigen
)
openlmm_set_global_target_properties(open_lmm_controller_concurrency_tests)
add_test(NAME open_lmm_controller_concurrency_tests
         COMMAND open_lmm_controller_concurrency_tests)

add_executable(open_lmm_map_alignment_coordinator_tests
  map_alignment_coordinator_tests.cpp
)
target_link_libraries(open_lmm_map_alignment_coordinator_tests PRIVATE
  open_lmm_loop_detector
)
openlmm_set_global_target_properties(open_lmm_map_alignment_coordinator_tests)
add_test(NAME open_lmm_map_alignment_coordinator_tests
         COMMAND open_lmm_map_alignment_coordinator_tests)

foreach(fixture IN ITEMS valid old_capability empty_capability null_capability)
  add_library(open_lmm_gui_plugin_fixture_${fixture} MODULE
    gui_plugin_fixture.cpp)
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

add_executable(open_lmm_gui_plugin_tests gui_plugin_tests.cpp)
target_link_libraries(open_lmm_gui_plugin_tests PRIVATE
  open_lmm_gui_core
  open_lmm_utils
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_gui_plugin_tests)
add_dependencies(open_lmm_gui_plugin_tests
  open_lmm_gui_plugin_fixture_valid
  open_lmm_gui_plugin_fixture_old_capability
  open_lmm_gui_plugin_fixture_empty_capability
  open_lmm_gui_plugin_fixture_null_capability)
add_test(NAME open_lmm_gui_plugin_tests COMMAND open_lmm_gui_plugin_tests
  $<TARGET_FILE:open_lmm_gui_plugin_fixture_valid>
  $<TARGET_FILE:open_lmm_gui_plugin_fixture_old_capability>
  $<TARGET_FILE:open_lmm_gui_plugin_fixture_empty_capability>
  $<TARGET_FILE:open_lmm_gui_plugin_fixture_null_capability>)

add_executable(open_lmm_profiling_macro_tests
  profiling_macro_tests.cpp
)
target_link_libraries(open_lmm_profiling_macro_tests PRIVATE
  open_lmm_profiling
)
openlmm_set_global_target_properties(open_lmm_profiling_macro_tests)
add_test(NAME open_lmm_profiling_macro_tests
         COMMAND open_lmm_profiling_macro_tests)

add_executable(open_lmm_logging_tests logging_tests.cpp)
target_link_libraries(open_lmm_logging_tests PRIVATE open_lmm_utils)
openlmm_set_global_target_properties(open_lmm_logging_tests)
add_test(NAME open_lmm_logging_tests COMMAND open_lmm_logging_tests)

add_executable(open_lmm_config_schema_tests config_schema_tests.cpp)
target_compile_definitions(open_lmm_config_schema_tests PRIVATE
  OPEN_LMM_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
target_link_libraries(open_lmm_config_schema_tests PRIVATE open_lmm_utils)
openlmm_set_global_target_properties(open_lmm_config_schema_tests)
add_test(NAME open_lmm_config_schema_tests COMMAND open_lmm_config_schema_tests)

foreach(fixture IN ITEMS valid wrong_abi null_factory missing_destroy no_entry)
  add_library(open_lmm_plugin_fixture_${fixture} MODULE plugin_fixture.cpp)
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

add_executable(open_lmm_plugin_abi_tests plugin_abi_tests.cpp)
target_include_directories(open_lmm_plugin_abi_tests PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_plugin_abi_tests PRIVATE
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
    list(APPEND _open_lmm_built_in_plugin_test_arguments
      "${plugin_target}=$<TARGET_FILE:${plugin_target}>")
  endif()
endforeach()
add_test(NAME open_lmm_plugin_abi_tests COMMAND open_lmm_plugin_abi_tests
  $<TARGET_FILE:open_lmm_plugin_fixture_valid>
  $<TARGET_FILE:open_lmm_plugin_fixture_wrong_abi>
  $<TARGET_FILE:open_lmm_plugin_fixture_null_factory>
  $<TARGET_FILE:open_lmm_plugin_fixture_missing_destroy>
  $<TARGET_FILE:open_lmm_plugin_fixture_no_entry>
  ${_open_lmm_built_in_plugin_test_arguments})

add_executable(open_lmm_plugin_selection_tests plugin_selection_tests.cpp)
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
add_test(NAME open_lmm_plugin_selection_tests
  COMMAND open_lmm_plugin_selection_tests
          ${_open_lmm_disabled_plugin_test_arguments})

if(OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT AND
   OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM)
  add_executable(open_lmm_self_contained_e2e_tests
    self_contained_e2e_tests.cpp)
  target_link_libraries(open_lmm_self_contained_e2e_tests PRIVATE
    open_lmm_map_server)
  add_dependencies(open_lmm_self_contained_e2e_tests
    create_scan_context
    create_free_dom)
  openlmm_set_global_target_properties(open_lmm_self_contained_e2e_tests)
  add_test(NAME open_lmm_self_contained_e2e_tests
           COMMAND open_lmm_self_contained_e2e_tests)
endif()

# A sourced, older install prefix may appear in LD_LIBRARY_PATH ahead of the
# build-tree RUNPATH. Unit tests must exercise the libraries just compiled.
set(_open_lmm_build_library_path
  "$<TARGET_FILE_DIR:open_lmm_batch>:"
  "$<TARGET_FILE_DIR:open_lmm_map_server>:"
  "$<TARGET_FILE_DIR:open_lmm_data_loader>:"
  "$<TARGET_FILE_DIR:open_lmm_loop_detector>:"
  "$<TARGET_FILE_DIR:open_lmm_backend_optimizer>:"
  "$<TARGET_FILE_DIR:open_lmm_dynamic_remover>:"
  "$<TARGET_FILE_DIR:open_lmm_algorithm_config>:"
  "$<TARGET_FILE_DIR:open_lmm_gui_core>:"
  "$<TARGET_FILE_DIR:open_lmm_contracts>:"
  "$<TARGET_FILE_DIR:open_lmm_client>:"
  "$<TARGET_FILE_DIR:open_lmm_common>:"
  "$<TARGET_FILE_DIR:open_lmm_descriptor>:"
  "$<TARGET_FILE_DIR:open_lmm_utils>:")
foreach(plugin_support_target IN ITEMS
    scan_context solid hmm_mos dufomap otd free_dom erasor)
  if(TARGET ${plugin_support_target})
    list(APPEND _open_lmm_build_library_path
      "$<TARGET_FILE_DIR:${plugin_support_target}>:")
  endif()
endforeach()
if(NOT OPEN_LMM_ENABLE_ASAN_UBSAN AND NOT OPEN_LMM_ENABLE_TSAN)
  list(APPEND _open_lmm_build_library_path "$ENV{LD_LIBRARY_PATH}")
endif()
string(JOIN "" _open_lmm_build_library_path ${_open_lmm_build_library_path})
set_tests_properties(
  open_lmm_safety_regression_tests
  open_lmm_agent_id_tests
  open_lmm_pipeline_controller_tests
  open_lmm_runtime_service_tests
  open_lmm_bounded_executor_tests
  open_lmm_runtime_transaction_tests
  open_lmm_config_transaction_tests
  open_lmm_runtime_bootstrapper_tests
  open_lmm_execution_spec_tests
  open_lmm_controller_concurrency_tests
  open_lmm_map_alignment_coordinator_tests
  open_lmm_gui_plugin_tests
  open_lmm_profiling_macro_tests
  open_lmm_logging_tests
  open_lmm_config_schema_tests
  open_lmm_plugin_abi_tests
  open_lmm_plugin_selection_tests
  PROPERTIES ENVIRONMENT "LD_LIBRARY_PATH=${_open_lmm_build_library_path}")
if(TARGET open_lmm_self_contained_e2e_tests)
  set_tests_properties(open_lmm_self_contained_e2e_tests PROPERTIES
    ENVIRONMENT "LD_LIBRARY_PATH=${_open_lmm_build_library_path}")
endif()

add_test(
  NAME open_lmm_architecture_boundary_tests
  COMMAND ${CMAKE_COMMAND}
          -DOPEN_LMM_SOURCE_DIR=${PROJECT_SOURCE_DIR}
          -P ${CMAKE_CURRENT_SOURCE_DIR}/architecture_boundary_tests.cmake
)

add_test(
  NAME open_lmm_release_policy_tests
  COMMAND ${CMAKE_COMMAND}
          -DOPEN_LMM_REPOSITORY_ROOT=${PROJECT_SOURCE_DIR}/..
          -P ${CMAKE_CURRENT_SOURCE_DIR}/release_policy_tests.cmake
)

# Package installation mutates the selected prefix and is intentionally kept
# out of an ament build. Standalone CI runs it serially after all targets build.
if(NOT (DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" EQUAL "2"))
  add_test(
    NAME open_lmm_package_consumer_tests
    COMMAND ${CMAKE_COMMAND}
            -DOPEN_LMM_BUILD_DIR=${PROJECT_BINARY_DIR}
            -DOPEN_LMM_SOURCE_DIR=${PROJECT_SOURCE_DIR}
            -DOPEN_LMM_PACKAGE_TEST_ROOT=${PROJECT_BINARY_DIR}/package-test
            -P ${CMAKE_CURRENT_SOURCE_DIR}/package_consumer_tests.cmake
  )
  set_tests_properties(open_lmm_package_consumer_tests PROPERTIES
    LABELS package
    RUN_SERIAL TRUE
  )
endif()
