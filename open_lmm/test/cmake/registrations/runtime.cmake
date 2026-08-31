function(openlmm_add_pipeline_controller_suite
    target selector layer module invariants)
  add_executable(${target}
    runtime/control/contract/pipeline_controller_tests.cpp)
  target_compile_definitions(${target} PRIVATE
    OPEN_LMM_PIPELINE_CONTROLLER_SUITE=${selector})
  target_link_libraries(${target} PRIVATE
    open_lmm_map_server open_lmm_loop_detector)
  openlmm_set_global_target_properties(${target})
  openlmm_add_test(
    NAME ${target} TARGET ${target}
    LAYER ${layer} MODULE ${module} OWNER PipelineController
    INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan tsan)
endfunction()

openlmm_add_pipeline_controller_suite(
  open_lmm_pipeline_controller_tests 1 L3 runtime.control
  "INV-02;INV-03;INV-06;INV-18")
openlmm_add_pipeline_controller_suite(
  open_lmm_pipeline_controller_fault_concurrency_tests 2 L5 runtime.control
  "INV-04;INV-05;INV-06;INV-07;INV-17;INV-18")
openlmm_add_pipeline_controller_suite(
  open_lmm_pipeline_controller_alignment_tests 3 L3
  runtime.control.alignment "INV-04;INV-17;INV-18")

function(openlmm_add_runtime_service_suite target selector layer invariants)
  add_executable(${target}
    runtime/service/fault_concurrency/runtime_service_tests.cpp)
  target_compile_definitions(${target} PRIVATE
    OPEN_LMM_RUNTIME_SERVICE_SUITE=${selector})
  target_link_libraries(${target} PRIVATE open_lmm_map_server)
  openlmm_set_global_target_properties(${target})
  set(runtime_service_timeout 60)
  if(OPEN_LMM_ENABLE_COVERAGE)
    # Clang's branch instrumentation can make the lifecycle-heavy contract
    # suite substantially slower on a contended CI runner. Keep the normal
    # PR timeout unchanged while retaining a finite deadlock guard here.
    set(runtime_service_timeout 120)
  endif()
  openlmm_add_test(
    NAME ${target} TARGET ${target}
    LAYER ${layer} MODULE runtime.service OWNER RuntimeService
    INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan tsan
    TIMEOUT ${runtime_service_timeout})
endfunction()

openlmm_add_runtime_service_suite(
  open_lmm_runtime_service_tests 1 L3 "INV-07;INV-08;INV-16;INV-18")
openlmm_add_runtime_service_suite(
  open_lmm_runtime_service_fault_concurrency_tests 2 L5
  "INV-04;INV-07;INV-08;INV-17;INV-18")

add_executable(open_lmm_runtime_client_tests
  runtime/client/contract/runtime_client_tests.cpp)
target_link_libraries(open_lmm_runtime_client_tests PRIVATE open_lmm_client)
if(TARGET create_scan_context)
  add_dependencies(open_lmm_runtime_client_tests create_scan_context)
endif()
if(TARGET create_free_dom)
  add_dependencies(open_lmm_runtime_client_tests create_free_dom)
endif()
openlmm_set_global_target_properties(open_lmm_runtime_client_tests)
openlmm_add_test(
  NAME open_lmm_runtime_client_tests TARGET open_lmm_runtime_client_tests
  LAYER L3 MODULE runtime.client OWNER RuntimeClient
  INVARIANTS INV-07 INV-08 INV-13 INV-14 INV-17
  LANES pr SANITIZERS asan-ubsan)

function(openlmm_add_resource_suite target selector layer owner invariants)
  add_executable(${target}
    runtime/resources/fault_concurrency/bounded_executor_tests.cpp)
  target_compile_definitions(${target} PRIVATE OPEN_LMM_RESOURCE_SUITE=${selector})
  target_link_libraries(${target} PRIVATE
    open_lmm_foundation_concurrency_objects open_lmm_runtime_resources_objects
    open_lmm_common open_lmm_algorithm_config open_lmm_utils
    open_lmm_profiling Eigen3::Eigen PCL::PCL gtsam
    nlohmann_json::nlohmann_json)
  openlmm_set_global_target_properties(${target})
  openlmm_add_test(
    NAME ${target} TARGET ${target}
    LAYER ${layer} MODULE runtime.resources OWNER ${owner}
    INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan tsan)
endfunction()

openlmm_add_resource_suite(
  open_lmm_resource_governor_tests 1 L2 ResourceGovernor "INV-16")
openlmm_add_resource_suite(
  open_lmm_bounded_executor_tests 2 L5 BoundedExecutor "INV-16;INV-17")

add_executable(open_lmm_runtime_transaction_tests
  runtime/state/integration/runtime_transaction_tests.cpp
)
target_link_libraries(open_lmm_runtime_transaction_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_runtime_transaction_tests)
openlmm_add_test(
  NAME open_lmm_runtime_transaction_tests
  TARGET open_lmm_runtime_transaction_tests
  LAYER L3 MODULE runtime.state OWNER RuntimeTransaction
  INVARIANTS INV-01 INV-02 INV-03 INV-04 INV-07
  LANES pr SANITIZERS asan-ubsan tsan)
add_executable(open_lmm_runtime_state_store_tests
  runtime/state/contract/runtime_state_store_tests.cpp)
target_link_libraries(open_lmm_runtime_state_store_tests PRIVATE
  open_lmm_runtime_model_objects open_lmm_runtime_state_objects
  open_lmm_common open_lmm_algorithm_config
  open_lmm_utils open_lmm_profiling Eigen3::Eigen PCL::PCL gtsam
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_runtime_state_store_tests)
openlmm_add_test(
  NAME open_lmm_runtime_state_store_tests
  TARGET open_lmm_runtime_state_store_tests
  LAYER L2 MODULE runtime.state OWNER RuntimeStateStore
  INVARIANTS INV-01 INV-03 INV-04 INV-07 INV-11
  LANES pr SANITIZERS asan-ubsan tsan)

add_executable(open_lmm_output_repository_tests
  storage/transactions/contract/output_repository_tests.cpp)
target_link_libraries(open_lmm_output_repository_tests PRIVATE
  open_lmm_storage_objects open_lmm_common open_lmm_algorithm_config
  open_lmm_utils open_lmm_profiling Eigen3::Eigen PCL::PCL gtsam
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_output_repository_tests)
openlmm_add_test(
  NAME open_lmm_output_repository_tests
  TARGET open_lmm_output_repository_tests
  LAYER L2 MODULE storage.transactions OWNER OutputRepository
  INVARIANTS INV-03 INV-04 INV-07
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_config_transaction_tests
  config/application/fault_concurrency/config_transaction_tests.cpp
)
target_link_libraries(open_lmm_config_transaction_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_config_transaction_tests)
openlmm_add_test(
  NAME open_lmm_config_transaction_tests TARGET open_lmm_config_transaction_tests
  LAYER L5 MODULE config.application OWNER ConfigTransaction
  INVARIANTS INV-03 INV-04 INV-07 INV-17
  LANES pr SANITIZERS asan-ubsan tsan)

add_executable(open_lmm_stage_executor_fixture_tests
  runtime/execution/integration/data_load_map_update_executor_tests.cpp
)
target_link_libraries(open_lmm_stage_executor_fixture_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_stage_executor_fixture_tests)
openlmm_add_test(
  NAME open_lmm_stage_executor_fixture_tests
  TARGET open_lmm_stage_executor_fixture_tests
  LAYER L3 MODULE runtime.execution OWNER DataLoadMapUpdateExecutors
  INVARIANTS INV-02 INV-03 INV-04 INV-16
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_stage_executor_tests
  runtime/execution/contract/stage_executor_tests.cpp)
target_link_libraries(open_lmm_stage_executor_tests PRIVATE open_lmm_map_server)
if(TARGET create_scan_context)
  add_dependencies(open_lmm_stage_executor_tests create_scan_context)
endif()
if(TARGET create_free_dom)
  add_dependencies(open_lmm_stage_executor_tests create_free_dom)
endif()
openlmm_set_global_target_properties(open_lmm_stage_executor_tests)
openlmm_add_test(
  NAME open_lmm_stage_executor_tests TARGET open_lmm_stage_executor_tests
  LAYER L3 MODULE runtime.execution OWNER StageExecutor
  INVARIANTS INV-01 INV-02 INV-03 INV-04 INV-07 INV-10
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_stage_coordinator_tests
  runtime/execution/contract/stage_coordinator_tests.cpp)
target_link_libraries(open_lmm_stage_coordinator_tests PRIVATE
  open_lmm_map_server)
if(TARGET create_scan_context)
  add_dependencies(open_lmm_stage_coordinator_tests create_scan_context)
endif()
if(TARGET create_free_dom)
  add_dependencies(open_lmm_stage_coordinator_tests create_free_dom)
endif()
openlmm_set_global_target_properties(open_lmm_stage_coordinator_tests)
openlmm_add_test(
  NAME open_lmm_stage_coordinator_tests
  TARGET open_lmm_stage_coordinator_tests
  LAYER L3 MODULE runtime.execution OWNER StageCoordinator
  INVARIANTS INV-01 INV-02 INV-03 INV-04 INV-07
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_map_update_executor_tests
  runtime/execution/fault_concurrency/map_update_executor_tests.cpp
)
target_link_libraries(open_lmm_map_update_executor_tests PRIVATE
  open_lmm_map_server
  open_lmm_data_loader
  open_lmm_dynamic_remover
)
openlmm_set_global_target_properties(open_lmm_map_update_executor_tests)
openlmm_add_test(
  NAME open_lmm_map_update_executor_tests TARGET open_lmm_map_update_executor_tests
  LAYER L5 MODULE runtime.execution OWNER MapUpdateExecutor
  INVARIANTS INV-02 INV-04 INV-16 INV-17
  LANES pr SANITIZERS asan-ubsan tsan)

add_executable(open_lmm_orchestration_executor_fixture_tests
  runtime/execution/integration/alignment_optimize_executor_tests.cpp
)
target_link_libraries(open_lmm_orchestration_executor_fixture_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(
  open_lmm_orchestration_executor_fixture_tests)
openlmm_add_test(
  NAME open_lmm_orchestration_executor_fixture_tests
  TARGET open_lmm_orchestration_executor_fixture_tests
  LAYER L3 MODULE runtime.execution OWNER AlignmentOptimizeExecutors
  INVARIANTS INV-02 INV-03 INV-04 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_save_executor_tests
  runtime/execution/integration/save_executor_tests.cpp)
target_link_libraries(open_lmm_save_executor_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_save_executor_tests)
openlmm_add_test(
  NAME open_lmm_save_executor_tests TARGET open_lmm_save_executor_tests
  LAYER L3 MODULE runtime.execution OWNER SaveExecutor
  INVARIANTS INV-03 INV-04 INV-07 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_runtime_bootstrapper_tests
  runtime/composition/integration/runtime_bootstrapper_tests.cpp
)
target_link_libraries(open_lmm_runtime_bootstrapper_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_runtime_bootstrapper_tests)
openlmm_add_test(
  NAME open_lmm_runtime_bootstrapper_tests
  TARGET open_lmm_runtime_bootstrapper_tests
  LAYER L3 MODULE runtime.composition OWNER RuntimeBootstrapper
  INVARIANTS INV-03 INV-04 INV-07 INV-16
  LANES pr SANITIZERS asan-ubsan)
function(openlmm_add_execution_spec_suite target selector module owner invariants)
  add_executable(${target} runtime/model/contract/execution_spec_tests.cpp)
  target_compile_definitions(${target} PRIVATE
    OPEN_LMM_EXECUTION_SPEC_SUITE=${selector})
  target_link_libraries(${target} PRIVATE
    open_lmm_runtime_model_objects open_lmm_runtime_state_objects
    open_lmm_common open_lmm_algorithm_config open_lmm_utils
    open_lmm_profiling Eigen3::Eigen PCL::PCL gtsam
    nlohmann_json::nlohmann_json)
  openlmm_set_global_target_properties(${target})
  openlmm_add_test(
    NAME ${target} TARGET ${target}
    LAYER L2 MODULE ${module} OWNER ${owner}
    INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan)
endfunction()

openlmm_add_execution_spec_suite(
  open_lmm_execution_spec_tests 1 runtime.model ExecutionSpec "INV-11")
openlmm_add_execution_spec_suite(
  open_lmm_artifact_repository_tests 2 runtime.state ArtifactRepository
  "INV-01;INV-02;INV-03;INV-11")

# Dataset-backed ordered replay verification. This is intentionally not a
# CTest because it requires the test1/test2 dataset configured by the caller.
add_executable(open_lmm_replay_verify tools/replay/replay_verify.cpp)
target_link_libraries(open_lmm_replay_verify PRIVATE open_lmm_map_server)
openlmm_set_global_target_properties(open_lmm_replay_verify)

# Case-driven replay runner. It is compiled against the public RuntimeClient
# façade; dataset-backed CTest registration is added only when a locked replay
# bundle is supplied by the Goal 03 data/CI phase.
add_executable(open_lmm_replay_runner tools/replay/replay_runner.cpp)
target_link_libraries(open_lmm_replay_runner PRIVATE
  open_lmm_client
  open_lmm_replay_contract
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_replay_runner)

add_executable(open_lmm_controller_concurrency_tests
  runtime/control/fault_concurrency/controller_concurrency_tests.cpp
)
target_link_libraries(open_lmm_controller_concurrency_tests PRIVATE
  open_lmm_map_server
)
openlmm_set_global_target_properties(open_lmm_controller_concurrency_tests)
openlmm_add_test(
  NAME open_lmm_controller_concurrency_tests
  TARGET open_lmm_controller_concurrency_tests
  LAYER L5 MODULE runtime.control OWNER PipelineController
  INVARIANTS INV-05 INV-06 INV-17 INV-18
  LANES pr SANITIZERS asan-ubsan tsan)

add_executable(open_lmm_map_alignment_coordinator_tests
  domain/loop_detection/fault_concurrency/map_alignment_coordinator_tests.cpp
)
target_link_libraries(open_lmm_map_alignment_coordinator_tests PRIVATE
  open_lmm_loop_detector
)
openlmm_set_global_target_properties(open_lmm_map_alignment_coordinator_tests)
openlmm_add_test(
  NAME open_lmm_map_alignment_coordinator_tests
  TARGET open_lmm_map_alignment_coordinator_tests
  LAYER L5 MODULE domain.loop_detection OWNER MapAlignmentCoordinator
  INVARIANTS INV-12 INV-16 INV-17
  LANES pr SANITIZERS asan-ubsan tsan)
