add_library(open_lmm_benchmark_support STATIC
  support/benchmark/benchmark_bundle.cpp
  support/benchmark/benchmark_pair.cpp
  support/benchmark/benchmark_options.cpp
  support/benchmark/benchmark_report.cpp
  support/benchmark/benchmark_statistics.cpp
  support/benchmark/fixture_manifest.cpp
  support/benchmark/process_window_sampler.cpp)
target_include_directories(open_lmm_benchmark_support PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_benchmark_support PUBLIC
  Threads::Threads nlohmann_json::nlohmann_json open_lmm_replay_contract)
openlmm_set_global_target_properties(open_lmm_benchmark_support)

add_library(open_lmm_benchmark_fixture_support STATIC
  support/benchmark/fixture_generator.cpp)
target_include_directories(open_lmm_benchmark_fixture_support PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_benchmark_fixture_support PUBLIC
  open_lmm_benchmark_support open_lmm_replay_contract PCL::PCL)
openlmm_set_global_target_properties(open_lmm_benchmark_fixture_support)

add_library(open_lmm_benchmark_runtime_support STATIC
  support/benchmark/stage_event_recorder.cpp)
target_include_directories(open_lmm_benchmark_runtime_support PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(open_lmm_benchmark_runtime_support PUBLIC
  open_lmm_benchmark_support open_lmm_contracts)
openlmm_set_global_target_properties(open_lmm_benchmark_runtime_support)

add_executable(open_lmm_benchmark_generate_fixture
  benchmark/fixtures/generate_fixture.cpp)
target_link_libraries(open_lmm_benchmark_generate_fixture PRIVATE
  open_lmm_benchmark_fixture_support)
openlmm_set_global_target_properties(open_lmm_benchmark_generate_fixture)

add_executable(open_lmm_benchmark_aggregate
  benchmark/benchmark_aggregate.cpp)
target_link_libraries(open_lmm_benchmark_aggregate PRIVATE
  open_lmm_benchmark_support)
openlmm_set_global_target_properties(open_lmm_benchmark_aggregate)

add_executable(open_lmm_benchmark_pair benchmark/benchmark_pair.cpp)
target_link_libraries(open_lmm_benchmark_pair PRIVATE
  open_lmm_benchmark_support)
openlmm_set_global_target_properties(open_lmm_benchmark_pair)

if(OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT AND
   OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM AND
   OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR)
  add_executable(open_lmm_benchmark_runner
    benchmark/workflow/benchmark_runner.cpp)
  target_link_libraries(open_lmm_benchmark_runner PRIVATE
    open_lmm_benchmark_fixture_support open_lmm_benchmark_runtime_support
    open_lmm_client)
  add_dependencies(open_lmm_benchmark_runner
    create_scan_context create_free_dom create_erasor)
  openlmm_set_global_target_properties(open_lmm_benchmark_runner)
endif()

if(OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT AND
   OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM AND
   OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR)
  add_executable(open_lmm_benchmark_resource_owner_runner
    benchmark/owners/resource_owner_runner.cpp)
  target_link_libraries(open_lmm_benchmark_resource_owner_runner PRIVATE
    open_lmm_benchmark_fixture_support open_lmm_map_server)
  add_dependencies(open_lmm_benchmark_resource_owner_runner
    create_scan_context create_free_dom create_erasor)
  openlmm_set_global_target_properties(
    open_lmm_benchmark_resource_owner_runner)
endif()

add_executable(open_lmm_benchmark_statistics_tests
  benchmark/contract/statistics_tests.cpp)
target_link_libraries(open_lmm_benchmark_statistics_tests PRIVATE
  open_lmm_benchmark_support)
openlmm_set_global_target_properties(open_lmm_benchmark_statistics_tests)
openlmm_add_test(
  NAME open_lmm_benchmark_statistics_tests
  TARGET open_lmm_benchmark_statistics_tests
  LAYER L1 MODULE workflows.benchmark OWNER BenchmarkStatistics
  INVARIANTS INV-16 INV-18 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_benchmark_options_tests
  benchmark/contract/options_tests.cpp)
target_link_libraries(open_lmm_benchmark_options_tests PRIVATE
  open_lmm_benchmark_support)
openlmm_set_global_target_properties(open_lmm_benchmark_options_tests)
openlmm_add_test(
  NAME open_lmm_benchmark_options_tests
  TARGET open_lmm_benchmark_options_tests
  LAYER L2 MODULE workflows.benchmark OWNER BenchmarkRunnerOptions
  INVARIANTS INV-16 INV-18 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_benchmark_stage_event_recorder_tests
  benchmark/contract/stage_event_recorder_tests.cpp)
target_link_libraries(open_lmm_benchmark_stage_event_recorder_tests PRIVATE
  open_lmm_benchmark_runtime_support)
openlmm_set_global_target_properties(
  open_lmm_benchmark_stage_event_recorder_tests)
openlmm_add_test(
  NAME open_lmm_benchmark_stage_event_recorder_tests
  TARGET open_lmm_benchmark_stage_event_recorder_tests
  LAYER L2 MODULE workflows.benchmark OWNER StageEventRecorder
  INVARIANTS INV-02 INV-04 INV-18 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_benchmark_report_contract_tests
  benchmark/contract/report_contract_tests.cpp)
target_link_libraries(open_lmm_benchmark_report_contract_tests PRIVATE
  open_lmm_benchmark_support)
openlmm_set_global_target_properties(open_lmm_benchmark_report_contract_tests)
openlmm_add_test(
  NAME open_lmm_benchmark_report_contract_tests
  TARGET open_lmm_benchmark_report_contract_tests
  LAYER L2 MODULE workflows.benchmark OWNER BenchmarkReport
  INVARIANTS INV-16 INV-18 LANES pr SANITIZERS asan-ubsan)

if(OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT AND
   OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM)
  add_executable(open_lmm_benchmark_small_smoke_tests
    benchmark/workflow/small_smoke_tests.cpp)
  target_link_libraries(open_lmm_benchmark_small_smoke_tests PRIVATE
    open_lmm_benchmark_fixture_support open_lmm_benchmark_runtime_support
    open_lmm_client)
  add_dependencies(open_lmm_benchmark_small_smoke_tests
    create_scan_context create_free_dom)
  openlmm_set_global_target_properties(open_lmm_benchmark_small_smoke_tests)
  openlmm_add_test(
    NAME open_lmm_benchmark_small_smoke_tests
    TARGET open_lmm_benchmark_small_smoke_tests
    LAYER L6 MODULE workflows.benchmark OWNER RuntimeWorkflow
    INVARIANTS INV-01 INV-02 INV-03 INV-04 INV-09 INV-16 INV-18
    LANES pr SANITIZERS asan-ubsan TIMEOUT 120)
endif()

add_executable(open_lmm_benchmark_fixture_policy_tests
  benchmark/contract/fixture_policy_tests.cpp)
target_link_libraries(open_lmm_benchmark_fixture_policy_tests PRIVATE
  open_lmm_benchmark_fixture_support)
openlmm_set_global_target_properties(open_lmm_benchmark_fixture_policy_tests)
openlmm_add_test(
  NAME open_lmm_benchmark_fixture_policy_tests
  TARGET open_lmm_benchmark_fixture_policy_tests
  LAYER L2 MODULE workflows.benchmark OWNER BenchmarkFixture
  INVARIANTS INV-03 INV-16 INV-18 LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_benchmark_process_sampler_tests
  benchmark/contract/process_sampler_tests.cpp)
target_link_libraries(open_lmm_benchmark_process_sampler_tests PRIVATE
  open_lmm_benchmark_support)
openlmm_set_global_target_properties(open_lmm_benchmark_process_sampler_tests)
openlmm_add_test(
  NAME open_lmm_benchmark_process_sampler_tests
  TARGET open_lmm_benchmark_process_sampler_tests
  LAYER L2 MODULE workflows.benchmark OWNER ProcessWindowSampler
  INVARIANTS INV-16 INV-18 LANES pr SANITIZERS asan-ubsan)
