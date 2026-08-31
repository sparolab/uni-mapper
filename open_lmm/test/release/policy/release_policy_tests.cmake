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

assert_file_contains(
  "${core_dir}/CMakeLists.txt"
  "project(open_lmm VERSION 3.0.0"
  "COMPATIBILITY SameMajorVersion"
  "COMPONENT Runtime"
  "COMPONENT Development"
  "COMPONENT Plugins"
  "COMPONENT Tools")
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
