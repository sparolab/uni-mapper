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

function(assert_actions_pinned path label)
  file(READ "${path}" contents)
  string(REGEX MATCHALL "uses:[^\r\n]+" action_lines "${contents}")
  list(LENGTH action_lines action_count)
  if(action_count LESS 1)
    message(FATAL_ERROR "${label} workflow must use pinned GitHub Actions")
  endif()
  foreach(action_line IN LISTS action_lines)
    string(REGEX REPLACE "^.*@([0-9a-f]+).*$" "\\1" action_ref
      "${action_line}")
    string(LENGTH "${action_ref}" action_ref_length)
    if(NOT action_ref MATCHES "^[0-9a-f]+$" OR
       NOT action_ref_length EQUAL 40)
      message(FATAL_ERROR
        "${label} GitHub Action is not pinned to 40-hex SHA: ${action_line}")
    endif()
  endforeach()
endfunction()

set(core_dir "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm")
set(cli_dir "${OPEN_LMM_REPOSITORY_ROOT}/applications/cli")
set(gui_dir "${OPEN_LMM_REPOSITORY_ROOT}/applications/gui")
set(python_dir "${OPEN_LMM_REPOSITORY_ROOT}/bindings/python")
set(experiment_dir
  "${OPEN_LMM_REPOSITORY_ROOT}/applications/python/experiment")
set(viser_dir
  "${OPEN_LMM_REPOSITORY_ROOT}/applications/python/viser")
set(iridescence_python_dir
  "${OPEN_LMM_REPOSITORY_ROOT}/applications/python/iridescence")
set(workflow
  "${OPEN_LMM_REPOSITORY_ROOT}/.github/workflows/compiler-matrix.yml")
set(nightly_benchmark_workflow
  "${OPEN_LMM_REPOSITORY_ROOT}/.github/workflows/nightly-benchmark.yml")
set(quality_workflow
  "${OPEN_LMM_REPOSITORY_ROOT}/.github/workflows/quality-gates.yml")
set(release_candidate_workflow
  "${OPEN_LMM_REPOSITORY_ROOT}/.github/workflows/release-candidate.yml")
set(release_promote_workflow
  "${OPEN_LMM_REPOSITORY_ROOT}/.github/workflows/release-promote.yml")

assert_file_contains(
  "${core_dir}/CMakeLists.txt"
  "project(open_lmm VERSION 3.0.0"
  "COMPATIBILITY SameMajorVersion"
  "COMPONENT Runtime"
  "COMPONENT Development"
  "COMPONENT Plugins")
assert_file_contains(
  "${cli_dir}/CMakeLists.txt"
  "project(open_lmm_cli VERSION 3.0.0"
  "find_package(open_lmm \${PROJECT_VERSION} EXACT CONFIG REQUIRED"
  "COMPONENTS client"
  "COMPONENT Tools")
assert_file_contains(
  "${python_dir}/pyproject.toml"
  "name = \"open-lmm\""
  "version = \"3.0.0\""
  "requires-python = \">=3.10,<3.11\"")
file(READ "${python_dir}/pyproject.toml" python_pyproject)
string(FIND "${python_pyproject}" "open-lmm-experiment =" old_experiment_entry)
if(NOT old_experiment_entry EQUAL -1)
  message(FATAL_ERROR "the SDK wheel must not own the experiment command")
endif()
assert_file_contains(
  "${experiment_dir}/pyproject.toml"
  "name = \"open-lmm-experiment\""
  "version = \"3.0.0\""
  "requires-python = \">=3.10,<3.11\""
  "dependencies = [\"open-lmm==3.0.0\"]"
  "open-lmm-experiment = \"open_lmm_experiment.cli:main\"")
assert_file_contains(
  "${viser_dir}/pyproject.toml"
  "name = \"open-lmm-viser\""
  "version = \"3.0.0\""
  "requires-python = \">=3.10,<3.11\""
  "\"open-lmm==3.0.0\""
  "\"viser>=1.1,<2\""
  "open-lmm-viser = \"open_lmm_viser.cli:main\"")
assert_file_contains(
  "${iridescence_python_dir}/pyproject.toml"
  "name = \"open-lmm-iridescence\""
  "version = \"3.0.0\""
  "requires-python = \">=3.10,<3.11\""
  "\"open-lmm==3.0.0\""
  "\"pyridescence==1.0.3\""
  "open-lmm-iridescence = \"open_lmm_iridescence.cli:main\"")
assert_file_contains(
  "${experiment_dir}/packaging/RELEASE_POLICY.md"
  "open-lmm-experiment` 3.0.0"
  "open-lmm==3.0.0")
assert_file_contains(
  "${viser_dir}/packaging/RELEASE_POLICY.md"
  "open-lmm-viser` 3.0.0"
  "open-lmm==3.0.0")
assert_file_contains(
  "${viser_dir}/packaging/THIRD_PARTY_NOTICES.md"
  "viser"
  "Apache-2.0"
  "dist-info/licenses/LICENSE")
assert_file_contains(
  "${iridescence_python_dir}/packaging/RELEASE_POLICY.md"
  "open-lmm-iridescence"
  "pyridescence==1.0.3"
  "Xvfb")
assert_file_contains(
  "${iridescence_python_dir}/packaging/THIRD_PARTY_NOTICES.md"
  "pyridescence 1.0.3"
  "MIT"
  "license metadata")
assert_file_contains(
  "${python_dir}/CMakeLists.txt"
  "project(open_lmm_python VERSION 3.0.0"
  "find_package(open_lmm \${PROJECT_VERSION} EXACT CONFIG REQUIRED"
  "COMPONENTS client"
  "OPEN_LMM_PYTHON_VERSION=\"\${PROJECT_VERSION}\""
  "target_link_libraries(open_lmm_python_native PRIVATE open_lmm::client)"
  "PATTERN \"*.json\""
  "COMPONENT Python")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/ros/CMakeLists.txt"
  "project(open_lmm_ros VERSION 3.0.0"
  "find_package(open_lmm \${PROJECT_VERSION} EXACT CONFIG REQUIRED COMPONENTS client)"
  "VERSION \${PROJECT_VERSION}"
  "SOVERSION \${PROJECT_VERSION_MAJOR}"
  "ros_visualization_bridge.cpp")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/ros/package.xml"
  "<depend version_eq=\"3.0.0\">open_lmm</depend>")
assert_file_contains(
  "${core_dir}/cmake/CompilerOptions.cmake"
  "openlmm_target_type STREQUAL \"SHARED_LIBRARY\""
  "VERSION \${PROJECT_VERSION}"
  "SOVERSION \${PROJECT_VERSION_MAJOR}")
assert_file_contains(
  "${gui_dir}/src/host/gui_plugin_module.cpp"
  "gui:services-v3")
assert_file_contains(
  "${gui_dir}/src/iridescence/iridescence_gui.cpp"
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
  "PluginSDK"
  "Plugins"
  "open-lmm-cli"
  "open-lmm Python"
  "empty intersection"
  "exact-version"
  "distribution/"
  "core -> CLI -> GUI"
  "ROS -> GUI -> CLI -> core"
  "baseline `cdda354`"
  "ghcr.io/sparolab/uni-mapper"
  "Headless Core + CLI"
  "do not claim crash- or power-loss durability"
  "blocks HIGH and CRITICAL"
  "never publishes a mutable"
  "exact candidate image digest")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/THIRD_PARTY_NOTICES.md"
  "GPL-3.0-only"
  "open_lmm/thirdparty/<dependency>/LICENSE")
assert_file_contains(
  "${core_dir}/cmake/open_lmm-install-components.txt"
  "version 3.0.0"
  "Runtime"
  "Development"
  "PluginSDK"
  "Plugins")
assert_file_contains(
  "${gui_dir}/CMakeLists.txt"
  "project(open_lmm_gui VERSION 3.0.0"
  "find_package(open_lmm \${PROJECT_VERSION} EXACT CONFIG REQUIRED"
  "add_library(open_lmm_gui::gui ALIAS open_lmm_gui_core)"
  "SOVERSION \${PROJECT_VERSION_MAJOR}"
  "COMPONENT GuiRuntime"
  "COMPONENT GuiDevelopment"
  "COMPONENT GuiPlugins"
  "COMPONENT GuiApplication")
assert_file_contains(
  "${gui_dir}/cmake/open_lmm-gui-install-components.txt"
  "v3.0.0"
  "GuiRuntime"
  "GuiDevelopment"
  "GuiPlugins"
  "GuiApplication")
assert_file_contains(
  "${cli_dir}/cmake/open_lmm-cli-install-components.txt"
  "version 3.0.0"
  "Tools"
  "open_lmm_batch")
if(NOT EXISTS "${OPEN_LMM_REPOSITORY_ROOT}/LICENCE")
  message(FATAL_ERROR "GPL-3.0 license text is missing")
endif()
foreach(package_metadata IN ITEMS LICENCE RELEASE_POLICY.md THIRD_PARTY_NOTICES.md)
  foreach(snapshot_root IN ITEMS
      "${core_dir}/package" "${cli_dir}/packaging" "${gui_dir}/packaging")
    execute_process(COMMAND "${CMAKE_COMMAND}" -E compare_files
      "${OPEN_LMM_REPOSITORY_ROOT}/${package_metadata}"
      "${snapshot_root}/${package_metadata}"
      RESULT_VARIABLE metadata_snapshot_result)
    if(NOT metadata_snapshot_result EQUAL 0)
      message(FATAL_ERROR
        "package metadata snapshot differs from repository owner: "
        "${snapshot_root}/${package_metadata}")
    endif()
  endforeach()
endforeach()

assert_file_contains(
  "${workflow}"
  "pull_request:"
  "merge_group:"
  "name: build / \${{ matrix.name }}"
  "name: sanitizer / \${{ matrix.name }}"
  "name: package / python310-wheel"
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
  "scripts/ci/run_critical_coverage.sh quality-critical-coverage")
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
  "${OPEN_LMM_REPOSITORY_ROOT}/docker/open_lmm.release.Dockerfile"
  "ubuntu@sha256:79676deb51ebb02885b0b9d33788e78a37cf1045ad79d1bb04c6a222c3556b3d"
  "snapshot.ubuntu.com"
  "--require-hashes"
  "--component Runtime"
  "--component Plugins"
  "USER 65532:65532"
  "ENTRYPOINT [\"/opt/open_lmm/bin/open_lmm_batch\"]")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/bindings/python/build-constraints.txt"
  "pip==24.0 --hash=sha256:"
  "scikit-build-core==0.10.7 --hash=sha256:"
  "pybind11==2.13.6 --hash=sha256:"
  "numpy==1.21.5 --hash=sha256:")
assert_file_contains(
  "${core_dir}/thirdparty/kiss_matcher/robin.patch"
  "a2dfd612a501bca83c47206255dbbff619481f97"
  "1c449ae953ce2a440b0d16c5ed1181d2754860ab")
if(EXISTS "${core_dir}/thirdparty/pcl/pcl.cmake")
  message(FATAL_ERROR "the invalid bundled PCL fallback must not return")
endif()
assert_file_contains(
  "${release_candidate_workflow}"
  "ghcr.io/sparolab/uni-mapper"
  "Require completed verification evidence"
  "nightly / gcc12-headless"
  "Build identical Headless images twice"
  "Validate expiring vulnerability exceptions"
  "severity: HIGH,CRITICAL"
  "format: cyclonedx"
  "release-manifest-v1.json"
  "actions/attest@"
  "--prerelease")
assert_file_contains(
  "${release_promote_workflow}"
  "Retag the exact RC image digest"
  "[[ \"$before\" == \"$after\" ]]"
  "Publish stable release without rebuilding")
file(READ "${release_promote_workflow}" release_promote_contents)
string(FIND "${release_promote_contents}" "docker build"
  stable_rebuild)
string(FIND "${release_promote_contents}" ":latest"
  mutable_latest)
if(NOT stable_rebuild EQUAL -1 OR NOT mutable_latest EQUAL -1)
  message(FATAL_ERROR "stable promotion must not rebuild or publish latest")
endif()
foreach(legacy_launcher IN ITEMS run_docker.sh docker/docker-compose.yml)
  if(EXISTS "${OPEN_LMM_REPOSITORY_ROOT}/${legacy_launcher}")
    message(FATAL_ERROR "legacy privileged launcher remains: ${legacy_launcher}")
  endif()
endforeach()
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/build_and_test.sh"
  "-DOPEN_LMM_ENABLE_STRICT_WARNINGS=ON"
  "-DCMAKE_BUILD_TYPE=Release"
  "applications/cli/test/cli_package_tests.cmake"
  "gui_source_root"
  "bindings/python/test/package/python_package_tests.cmake"
  "scripts/ci/inspect_symbol_visibility.sh"
  "--output-junit"
  "ctest-open_lmm.xml"
  "ctest-open_lmm-gui.xml"
  "ctest-open_lmm-ros.xml"
  "distribution-build"
  "ctest-distribution.xml")
assert_file_contains(
  "${gui_dir}/test/CMakeLists.txt"
  "NAME open_lmm_gui_package_tests"
  "package/gui_package_tests.cmake")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/run_static_analysis.sh"
  "applications/cli/test/quality/production_sources.tsv"
  "applications/cli"
  "cli-compiled-sources.txt"
  "-DOPEN_LMM_CLI_WARNINGS_AS_ERRORS=ON"
  "applications/gui/test/quality/production_sources.tsv"
  "applications/gui"
  "gui-compiled-sources.txt"
  "-DOPEN_LMM_GUI_WARNINGS_AS_ERRORS=ON"
  "bindings/python/test/quality/production_sources.tsv"
  "bindings/python"
  "python-compiled-sources.txt"
  "-DOPEN_LMM_PYTHON_WARNINGS_AS_ERRORS=ON")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/build_sanitizer_tests.sh"
  "applications/gui"
  "OPEN_LMM_GUI_BUILD_IRIDESCENCE=OFF"
  "ctest-gui.xml"
  "bindings/python"
  "OPEN_LMM_PYTHON_CORE_CONTRACT_ONLY=ON"
  "ctest-python.xml"
  "--output-junit"
  "ctest.xml")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/build_python_wheel_and_test.sh"
  "OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT=ON"
  "OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM=ON"
  "bindings/python/build_local_wheel.sh"
  "OPEN_LMM_PYTHON_WHEEL_ONLY=ON"
  "python-owner-inventory.tsv"
  "runtime-requirements.txt"
  "--require-hashes"
  "wheel.sha256")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/check_architecture_policy.sh"
  "architecture_boundary_tests.cmake"
  "distribution/test/architecture/repository_architecture_tests.cmake"
  "release_policy_tests.cmake")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/distribution/CMakeLists.txt"
  "open_lmm_combined_distribution_tests"
  "open_lmm_developer_entrypoint_tests"
  "OPEN_LMM_DISTRIBUTION_CORE_BUILD_DIR"
  "OPEN_LMM_DISTRIBUTION_ROS_BUILD_DIR")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/Makefile"
  "core-build:"
  "gui-build:"
  "gui-run:"
  "gui-clean:"
  "cli-build:"
  "cli-run:"
  "cli-clean:"
  "python-build:"
  "python-install:"
  "python-run:"
  "python-clean:"
  "ros-build:"
  "ros-run:"
  "ros-clean:"
  "dev-clean:"
  "-DOPEN_LMM_GUI_BUILD_IRIDESCENCE=ON"
  "-DOPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT=ON"
  "-DOPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM=ON"
  "$(PYTHON_SOURCE)/build_local_wheel.sh"
  "PYTHON_VENV := $(DEV_PREFIX)/python-venv"
  "-DCMAKE_PREFIX_PATH=\"$(DEV_PREFIX)\""
  "--base-paths \"$(ROS_SOURCE)\""
  "ROS_USE_RVIZ"
  "env -u LD_LIBRARY_PATH")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/dev/remove_install_manifest.sh"
  "mapfile -t owned_paths"
  "unsafe"
  "rm -f --")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/distribution/manifests/artifact-set-v3.tsv"
  "schema_version\tartifact_id\tversion\tnamespace\tinstall_mode\tprofiles\texact_dependencies\towner_root"
  "python\t3.0.0\tpython-venv\twheel\tpython-sdk"
  "experiment\t3.0.0\tpython-venv\twheel\texperiment-cli\tpython=3.0.0\tapplications/python/experiment"
  "viser\t3.0.0\tpython-venv\twheel\tviser-application\tpython=3.0.0\tapplications/python/viser"
  "iridescence-python\t3.0.0\tpython-venv\twheel\tiridescence-application\tpython=3.0.0\tapplications/python/iridescence"
  "ros\t3.0.0\tros-overlay\tament-install")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/distribution/manifests/legacy-owner-transfers-v3.tsv"
  "baseline_commit\told_owner\tnamespace\trelative_path\tdisposition\tnew_owner"
  "cdda354\tcore\tnative-prefix\tbin/open_lmm_batch\ttransfer\tcli")
assert_file_contains(
  "${OPEN_LMM_REPOSITORY_ROOT}/scripts/ci/run_benchmark_tests.sh"
  "OPEN_LMM_PERFORMANCE_BASELINE"
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
    critical_coverage.py)
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
assert_actions_pinned("${workflow}" "Required")

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
assert_actions_pinned("${quality_workflow}" "Quality")
assert_actions_pinned("${release_candidate_workflow}" "Release candidate")
assert_actions_pinned("${release_promote_workflow}" "Release promotion")
