if(NOT OPEN_LMM_REPOSITORY_ROOT)
  message(FATAL_ERROR "OPEN_LMM_REPOSITORY_ROOT is required")
endif()

function(assert_file_contains path)
  if(NOT EXISTS "${path}")
    message(FATAL_ERROR "required release file is missing: ${path}")
  endif()
  file(READ "${path}" contents)
  foreach(pattern IN LISTS ARGN)
    string(FIND "${contents}" "${pattern}" found)
    if(found EQUAL -1)
      message(FATAL_ERROR "${path} must contain: ${pattern}")
    endif()
  endforeach()
endfunction()

set(core_dir "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm")
set(workflow
  "${OPEN_LMM_REPOSITORY_ROOT}/.github/workflows/compiler-matrix.yml")
set(nightly_benchmark_workflow
  "${OPEN_LMM_REPOSITORY_ROOT}/.github/workflows/nightly-benchmark.yml")
set(quality_workflow
  "${OPEN_LMM_REPOSITORY_ROOT}/.github/workflows/quality-gates.yml")

assert_file_contains(
  "${core_dir}/CMakeLists.txt"
  "project(open_lmm VERSION 3.0.0"
  "COMPATIBILITY SameMajorVersion"
  "COMPONENT Runtime"
  "COMPONENT Development"
  "COMPONENT Plugins"
  "COMPONENT Tools")
assert_file_contains(
  "${core_dir}/src/adapters/python/pyproject.toml"
  "name = \"open-lmm\""
  "version = \"3.0.0\""
  "requires-python = \">=3.10,<3.11\""
  "open-lmm-experiment = \"open_lmm.experiments._cli:main\"")
assert_file_contains(
  "${core_dir}/src/adapters/python/CMakeLists.txt"
  "OPEN_LMM_PYTHON_VERSION=\"\${PROJECT_VERSION}\""
  "target_link_libraries(open_lmm_python_native PRIVATE open_lmm_client)"
  "PATTERN \"*.json\""
  "COMPONENT Python")
assert_file_contains(
  "${core_dir}/cmake/options.cmake"
  "option(OPEN_LMM_BUILD_PYTHON \"Build the optional CPython binding\" OFF)")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/ros/CMakeLists.txt"
  "project(open_lmm_ros VERSION 3.0.0"
  "VERSION \${PROJECT_VERSION}"
  "SOVERSION \${PROJECT_VERSION_MAJOR}")
assert_file_contains(
  "${core_dir}/cmake/CompilerOptions.cmake"
  "openlmm_target_type STREQUAL \"SHARED_LIBRARY\""
  "VERSION \${PROJECT_VERSION}"
  "SOVERSION \${PROJECT_VERSION_MAJOR}")
assert_file_contains(
  "${core_dir}/src/adapters/gui/gui_plugin_host.cpp"
  "gui:services-v3")
assert_file_contains(
  "${core_dir}/src/adapters/gui/iridescence/iridescence_gui.cpp"
  "gui:services-v3")
assert_file_contains(
  "${core_dir}/include/open_lmm/common/plugin_api.h"
  "kind-specific capability contract")

foreach(package_manifest IN ITEMS
    "${core_dir}/package.xml"
    "${OPEN_LMM_REPOSITORY_ROOT}/ros/package.xml")
  assert_file_contains(
    "${package_manifest}"
    "<version>3.0.0</version>"
    "<license>GPL-3.0-only</license>")
endforeach()

assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/RELEASE_POLICY.md"
  "SOVERSION=3"
  "gui:services-v3"
  "Runtime"
  "Development"
  "Plugins"
  "Tools")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/THIRD_PARTY_NOTICES.md"
  "GPL-3.0-only"
  "open_lmm/thirdparty/<dependency>/LICENSE")
assert_file_contains(
  "${core_dir}/cmake/open_lmm-install-components.txt"
  "version 3.0.0"
  "Runtime"
  "Development"
  "Plugins"
  "Tools")
if(NOT EXISTS "${OPEN_LMM_REPOSITORY_ROOT}/LICENCE")
  message(FATAL_ERROR "GPL-3.0 license text is missing")
endif()

assert_file_contains(
  "${workflow}"
  "pull_request:"
  "merge_group:"
  "name: build / \${{ matrix.name }}"
  "name: sanitizer / \${{ matrix.name }}"
  "name: policy / architecture-boundary"
  "name: gcc12-gui-off"
  "name: gcc12-gui-on"
  "name: gcc13-gui-off"
  "name: clang15-gui-off"
  "name: asan-ubsan"
  "name: tsan")
assert_file_contains(
  "${quality_workflow}"
  "pull_request:"
  "merge_group:"
  "name: quality / static-high-confidence"
  "name: quality / fuzz-smoke"
  "name: quality / critical-coverage"
  "name: quality / static-broad"
  "name: quality / fuzz-nightly"
  "scripts/ci/run_static_analysis.sh required"
  "scripts/ci/build_fuzz_tests.sh quality-fuzz-smoke"
  "scripts/ci/run_critical_coverage.sh quality-critical-coverage"
  "scripts/ci/run_mutation_feasibility.sh quality-mutation-pilot")
assert_file_contains(
  "${nightly_benchmark_workflow}"
  "OPEN_LMM_BENCHMARK_IMAGE"
  "@sha256:[0-9a-f]{64}"
  "docker pull \"$BENCHMARK_IMAGE\""
  "image_digest=\${BENCHMARK_IMAGE##*@}"
  "\"$BENCHMARK_IMAGE\"")
file(READ "${nightly_benchmark_workflow}" nightly_benchmark_contents)
string(FIND "${nightly_benchmark_contents}" "docker build"
  nightly_mutable_image_build)
if(NOT nightly_mutable_image_build EQUAL -1)
  message(FATAL_ERROR
    "nightly benchmark must consume one reviewed immutable image digest")
endif()
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/build_and_test.sh"
  "-DOPEN_LMM_ENABLE_STRICT_WARNINGS=ON"
  "scripts/ci/inspect_symbol_visibility.sh"
  "--output-junit"
  "ctest-open_lmm.xml"
  "ctest-open_lmm-ros.xml")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/build_sanitizer_tests.sh"
  "--output-junit"
  "ctest.xml")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/check_architecture_policy.sh"
  "architecture_boundary_tests.cmake"
  "release_policy_tests.cmake")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/run_benchmark_tests.sh"
  "OPEN_LMM_PERFORMANCE_BASELINE"
  "docs/post_freeze_results/performance_baseline.json"
  "baseline_args=(--baseline")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/benchmark/run_benchmarks.sh"
  "--baseline REVIEWED_CATALOG"
  "aggregate_args+=(--baseline \"$baseline\")"
  "owner_aggregate_args+=(--baseline \"$baseline\")")
assert_file_contains(
  "${workflow}"
  "scripts/ci/check_architecture_policy.sh")
assert_file_contains(
  "${core_dir}/cmake/options.cmake"
  "option(OPEN_LMM_ENABLE_CLANG_TIDY"
  "option(OPEN_LMM_ENABLE_STRICT_WARNINGS"
  "option(OPEN_LMM_ENABLE_COVERAGE"
  "option(OPEN_LMM_ENABLE_FUZZING")
foreach(quality_script IN ITEMS
    run_static_analysis.sh
    build_fuzz_tests.sh
    run_critical_coverage.sh
    inspect_symbol_visibility.sh
    run_mutation_feasibility.sh
    critical_coverage.py
    mutation_pilot.py)
  if(NOT EXISTS
      "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/${quality_script}")
    message(FATAL_ERROR "Goal 08 script is missing: ${quality_script}")
  endif()
endforeach()

set(critical_sources
  "${core_dir}/test/quality/coverage/critical_sources.tsv")
set(critical_tests
  "${core_dir}/test/quality/coverage/critical_tests.tsv")
set(critical_baseline
  "${core_dir}/test/quality/coverage/critical_branch_coverage_baseline.json")
assert_file_contains("${critical_sources}"
  "RuntimeStateStore"
  "PipelineController"
  "RuntimeService"
  "FileSetTransaction"
  "StageCoordinator")
assert_file_contains("${critical_tests}"
  "open_lmm_runtime_state_store_tests"
  "open_lmm_pipeline_controller_tests"
  "open_lmm_runtime_service_tests"
  "open_lmm_storage_file_set_tests"
  "open_lmm_stage_coordinator_tests")
if(NOT EXISTS "${critical_baseline}")
  message(FATAL_ERROR "reviewed Goal 08 coverage baseline is missing")
endif()
file(READ "${critical_baseline}" critical_baseline_contents)
string(JSON critical_baseline_schema GET "${critical_baseline_contents}"
  schema_version)
string(JSON critical_baseline_status GET "${critical_baseline_contents}"
  status)
if(NOT critical_baseline_schema EQUAL 1 OR
   NOT critical_baseline_status STREQUAL "reviewed")
  message(FATAL_ERROR
    "Goal 08 coverage baseline must be schema 1 and reviewed")
endif()
foreach(critical_owner IN ITEMS
    RuntimeStateStore PipelineController RuntimeService FileSetTransaction
    StageCoordinator)
  string(JSON owner_branch_count ERROR_VARIABLE owner_json_error
    GET "${critical_baseline_contents}" owners ${critical_owner} branches count)
  if(owner_json_error OR owner_branch_count LESS 1)
    message(FATAL_ERROR
      "Goal 08 coverage baseline owner is missing/nonzero: ${critical_owner}")
  endif()
endforeach()

file(READ "${workflow}" workflow_contents)
string(FIND "${workflow_contents}" "paths:" paths_filter)
if(NOT paths_filter EQUAL -1)
  message(FATAL_ERROR "required workflow must not use a paths filter")
endif()
string(REGEX MATCHALL "uses:[^\r\n]+" action_lines "${workflow_contents}")
list(LENGTH action_lines action_count)
if(action_count LESS 1)
  message(FATAL_ERROR "workflow must use at least one pinned GitHub Action")
endif()
foreach(action_line IN LISTS action_lines)
  string(REGEX REPLACE "^.*@([0-9a-f]+).*$" "\\1" action_ref
    "${action_line}")
  string(LENGTH "${action_ref}" action_ref_length)
  if(NOT action_ref MATCHES "^[0-9a-f]+$" OR
     NOT action_ref_length EQUAL 40)
    message(FATAL_ERROR
      "GitHub Action is not pinned to an immutable 40-hex SHA: ${action_line}")
  endif()
endforeach()

file(READ "${quality_workflow}" quality_workflow_contents)
string(FIND "${quality_workflow_contents}" "paths:" quality_paths_filter)
if(NOT quality_paths_filter EQUAL -1)
  message(FATAL_ERROR "quality workflow must not use a paths filter")
endif()
string(FIND "${quality_workflow_contents}" " calibrate"
  quality_baseline_rewrite)
if(NOT quality_baseline_rewrite EQUAL -1)
  message(FATAL_ERROR
    "quality workflow must never auto-calibrate/rewrite the reviewed baseline")
endif()
string(REGEX MATCHALL "uses:[^\r\n]+" quality_action_lines
  "${quality_workflow_contents}")
list(LENGTH quality_action_lines quality_action_count)
if(quality_action_count LESS 1)
  message(FATAL_ERROR "quality workflow must use pinned GitHub Actions")
endif()
foreach(action_line IN LISTS quality_action_lines)
  string(REGEX REPLACE "^.*@([0-9a-f]+).*$" "\\1" action_ref
    "${action_line}")
  string(LENGTH "${action_ref}" action_ref_length)
  if(NOT action_ref MATCHES "^[0-9a-f]+$" OR
     NOT action_ref_length EQUAL 40)
    message(FATAL_ERROR
      "Quality GitHub Action is not pinned to 40-hex SHA: ${action_line}")
  endif()
endforeach()
