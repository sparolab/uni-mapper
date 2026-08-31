if(OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT AND
   OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM)
  function(openlmm_add_e2e_suite target selector layer module invariants)
    add_executable(${target} workflows/e2e/self_contained_e2e_tests.cpp)
    target_compile_definitions(${target} PRIVATE OPEN_LMM_E2E_SUITE=${selector})
    target_link_libraries(${target} PRIVATE open_lmm_map_server)
    add_dependencies(${target} create_scan_context create_free_dom)
    openlmm_set_global_target_properties(${target})
    openlmm_add_test(
      NAME ${target} TARGET ${target}
      LAYER ${layer} MODULE ${module} OWNER RuntimeWorkflow
      INVARIANTS ${invariants} LANES pr SANITIZERS asan-ubsan
      TIMEOUT 120)
  endfunction()
  openlmm_add_e2e_suite(
    open_lmm_self_contained_e2e_tests 1 L4 workflows.e2e
    "INV-01;INV-02;INV-03;INV-04;INV-09")
  openlmm_add_e2e_suite(
    open_lmm_self_contained_reconfiguration_tests 2 L3
    workflows.reconfiguration "INV-02;INV-03;INV-04;INV-09;INV-10")
  openlmm_add_e2e_suite(
    open_lmm_self_contained_recovery_tests 3 L5 workflows.fault_concurrency
    "INV-03;INV-04;INV-07;INV-09")
  openlmm_add_e2e_suite(
    open_lmm_self_contained_resource_tests 4 L5 workflows.fault_concurrency
    "INV-02;INV-04;INV-16")

  add_executable(open_lmm_replay_runner_e2e_tests
    replay/e2e/replay_runner_e2e_tests.cpp)
  target_compile_definitions(open_lmm_replay_runner_e2e_tests PRIVATE
    OPEN_LMM_REPLAY_RUNNER="$<TARGET_FILE:open_lmm_replay_runner>")
  target_link_libraries(open_lmm_replay_runner_e2e_tests PRIVATE
    open_lmm_replay_contract
    nlohmann_json::nlohmann_json)
  add_dependencies(open_lmm_replay_runner_e2e_tests
    open_lmm_replay_runner create_scan_context create_free_dom)
  openlmm_set_global_target_properties(open_lmm_replay_runner_e2e_tests)
  openlmm_add_test(
    NAME open_lmm_replay_runner_e2e_tests
    TARGET open_lmm_replay_runner_e2e_tests
    LAYER L4 MODULE workflows.replay OWNER ReplayRunner
    INVARIANTS INV-01 INV-02 INV-03 INV-04 INV-09 INV-14
    LANES pr SANITIZERS asan-ubsan
    TIMEOUT 120)
endif()
