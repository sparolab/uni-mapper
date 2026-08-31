cmake_minimum_required(VERSION 3.25)
project(open_lmm_distribution_contract VERSION 3.0.0 LANGUAGES NONE)

include(CTest)
option(OPEN_LMM_ENABLE_BENCHMARK_WORKFLOW_TEST
  "Register the repository benchmark workflow contract" OFF)

get_filename_component(OPEN_LMM_REPOSITORY_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/.." ABSOLUTE)

add_test(NAME open_lmm_repository_architecture_tests
  COMMAND "${CMAKE_COMMAND}"
    "-DOPEN_LMM_SOURCE_DIR=${OPEN_LMM_REPOSITORY_ROOT}/open_lmm"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/architecture/repository_architecture_tests.cmake")
add_test(NAME open_lmm_application_boundary_inventory_tests
  COMMAND "${CMAKE_COMMAND}"
    "-DOPEN_LMM_REPOSITORY_ROOT=${OPEN_LMM_REPOSITORY_ROOT}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/architecture/application_boundary_inventory_tests.cmake")
add_test(NAME open_lmm_release_policy_tests
  COMMAND "${CMAKE_COMMAND}"
    "-DOPEN_LMM_REPOSITORY_ROOT=${OPEN_LMM_REPOSITORY_ROOT}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/test/release/release_policy_tests.cmake")
add_test(NAME open_lmm_replay_fetch_tests
  COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/replay/replay_fetch_tests.sh"
    "${OPEN_LMM_REPOSITORY_ROOT}/scripts/replay/fetch_replay_data.sh")
add_test(NAME open_lmm_replay_subset_tests
  COMMAND bash "${CMAKE_CURRENT_SOURCE_DIR}/test/replay/replay_subset_tests.sh"
    "${OPEN_LMM_REPOSITORY_ROOT}/scripts/replay/build_replay_subset.py")
add_test(NAME open_lmm_critical_coverage_contract_tests
  COMMAND /usr/bin/python3
    "${CMAKE_CURRENT_SOURCE_DIR}/test/quality/critical_coverage_contract_tests.py"
    "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/critical_coverage.py")

foreach(required_build IN ITEMS CORE CLI GUI ROS)
  if(NOT DEFINED OPEN_LMM_DISTRIBUTION_${required_build}_BUILD_DIR)
    set(_open_lmm_distribution_builds_complete FALSE)
  endif()
endforeach()
if(NOT DEFINED _open_lmm_distribution_builds_complete)
  set(_open_lmm_distribution_builds_complete TRUE)
endif()

if(_open_lmm_distribution_builds_complete)
  add_test(NAME open_lmm_combined_distribution_tests
    COMMAND "${CMAKE_COMMAND}"
      "-DOPEN_LMM_REPOSITORY_ROOT=${OPEN_LMM_REPOSITORY_ROOT}"
      "-DOPEN_LMM_CORE_BUILD_DIR=${OPEN_LMM_DISTRIBUTION_CORE_BUILD_DIR}"
      "-DOPEN_LMM_CLI_BUILD_DIR=${OPEN_LMM_DISTRIBUTION_CLI_BUILD_DIR}"
      "-DOPEN_LMM_GUI_BUILD_DIR=${OPEN_LMM_DISTRIBUTION_GUI_BUILD_DIR}"
      "-DOPEN_LMM_ROS_BUILD_DIR=${OPEN_LMM_DISTRIBUTION_ROS_BUILD_DIR}"
      "-DOPEN_LMM_DISTRIBUTION_TEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/package-test"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/package/combined_distribution_tests.cmake")
  set_tests_properties(open_lmm_combined_distribution_tests PROPERTIES
    RUN_SERIAL TRUE TIMEOUT 300)
endif()

if(OPEN_LMM_ENABLE_BENCHMARK_WORKFLOW_TEST)
  if(NOT DEFINED OPEN_LMM_DISTRIBUTION_CORE_BUILD_DIR)
    message(FATAL_ERROR
      "OPEN_LMM_DISTRIBUTION_CORE_BUILD_DIR is required for benchmark workflow testing")
  endif()
  add_test(NAME open_lmm_benchmark_orchestrator_tests
    COMMAND "${CMAKE_COMMAND}"
      "-DOPEN_LMM_REPOSITORY_ROOT=${OPEN_LMM_REPOSITORY_ROOT}"
      "-DOPEN_LMM_BUILD_DIR=${OPEN_LMM_DISTRIBUTION_CORE_BUILD_DIR}"
      "-DOPEN_LMM_BENCHMARK_TEST_ROOT=${CMAKE_CURRENT_BINARY_DIR}/benchmark-contract-test"
      -P "${CMAKE_CURRENT_SOURCE_DIR}/test/benchmark/orchestrator_tests.cmake")
  set_tests_properties(open_lmm_benchmark_orchestrator_tests PROPERTIES
    RUN_SERIAL TRUE TIMEOUT 180)
endif()
