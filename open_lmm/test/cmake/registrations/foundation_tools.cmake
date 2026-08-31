add_executable(open_lmm_agent_id_tests
  foundation/contracts/unit/agent_id_tests.cpp)
target_link_libraries(open_lmm_agent_id_tests PRIVATE
  open_lmm_common
  open_lmm_utils
  gtsam)
openlmm_set_global_target_properties(open_lmm_agent_id_tests)
openlmm_add_test(
  NAME open_lmm_agent_id_tests TARGET open_lmm_agent_id_tests
  LAYER L1 MODULE foundation.contracts OWNER AgentId
  LANES pr)
add_executable(open_lmm_artifact_compare tools/replay/artifact_compare.cpp)
target_link_libraries(open_lmm_artifact_compare PRIVATE
  PCL::PCL
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_artifact_compare)

add_library(open_lmm_replay_contract STATIC
  tools/replay/replay_contract.cpp
  tools/replay/replay_input_lock.cpp
  tools/replay/replay_sha256.cpp)
target_include_directories(open_lmm_replay_contract PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/tools/replay)
target_link_libraries(open_lmm_replay_contract PUBLIC
  nlohmann_json::nlohmann_json)
openlmm_set_global_target_properties(open_lmm_replay_contract)

add_executable(open_lmm_replay_compare tools/replay/replay_compare.cpp)
target_link_libraries(open_lmm_replay_compare PRIVATE open_lmm_replay_contract)
openlmm_set_global_target_properties(open_lmm_replay_compare)
openlmm_add_test(
  NAME open_lmm_replay_compare_tests TARGET open_lmm_replay_compare
  COMMAND_ARGS
    --baseline
    ${CMAKE_CURRENT_SOURCE_DIR}/replay/fixtures/tiny_fixture_baseline_v1.json
    --report
    ${CMAKE_CURRENT_SOURCE_DIR}/replay/fixtures/tiny_fixture_report_v1.json
  LAYER L2 MODULE workflows.replay OWNER ReplayComparator
  INVARIANTS INV-02 INV-03 INV-04
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_replay_validate tools/replay/replay_validate.cpp)
target_link_libraries(open_lmm_replay_validate PRIVATE open_lmm_replay_contract)
openlmm_set_global_target_properties(open_lmm_replay_validate)
openlmm_add_test(
  NAME open_lmm_replay_validate_tests TARGET open_lmm_replay_validate
  COMMAND_ARGS
    --kind case
    --input ${CMAKE_CURRENT_SOURCE_DIR}/replay/fixtures/tiny_fixture_case_v1.json
  LAYER L2 MODULE workflows.replay OWNER ReplayContract
  INVARIANTS INV-02 INV-03 INV-04
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_replay_contract_tests
  replay/contract/replay_contract_tests.cpp)
target_link_libraries(open_lmm_replay_contract_tests PRIVATE
  open_lmm_replay_contract)
openlmm_set_global_target_properties(open_lmm_replay_contract_tests)
openlmm_add_test(
  NAME open_lmm_replay_contract_tests TARGET open_lmm_replay_contract_tests
  LAYER L2 MODULE workflows.replay OWNER ReplayContract
  INVARIANTS INV-02 INV-03 INV-04
  LANES pr SANITIZERS asan-ubsan)

add_executable(open_lmm_profiling_macro_tests
  foundation/profiling/unit/profiling_macro_tests.cpp
)
target_link_libraries(open_lmm_profiling_macro_tests PRIVATE
  open_lmm_profiling
)
openlmm_set_global_target_properties(open_lmm_profiling_macro_tests)
openlmm_add_test(
  NAME open_lmm_profiling_macro_tests TARGET open_lmm_profiling_macro_tests
  LAYER L1 MODULE foundation.profiling OWNER ProfilingMacros
  LANES pr)

add_executable(open_lmm_logging_tests foundation/logging/unit/logging_tests.cpp)
target_link_libraries(open_lmm_logging_tests PRIVATE open_lmm_utils)
openlmm_set_global_target_properties(open_lmm_logging_tests)
openlmm_add_test(
  NAME open_lmm_logging_tests TARGET open_lmm_logging_tests
  LAYER L1 MODULE foundation.logging OWNER LoggingFacade
  LANES pr)
