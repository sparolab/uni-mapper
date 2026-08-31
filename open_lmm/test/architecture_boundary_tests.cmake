if(NOT OPEN_LMM_SOURCE_DIR)
  message(FATAL_ERROR "OPEN_LMM_SOURCE_DIR is required")
endif()

function(assert_file_excludes relative_path)
  file(READ "${OPEN_LMM_SOURCE_DIR}/${relative_path}" contents)
  foreach(pattern IN LISTS ARGN)
    string(FIND "${contents}" "${pattern}" found)
    if(NOT found EQUAL -1)
      message(FATAL_ERROR
        "${relative_path} must not expose implementation dependency: ${pattern}")
    endif()
  endforeach()
endfunction()

function(assert_exact_matches description root pattern)
  set(expected ${ARGN})
  file(GLOB_RECURSE candidates
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.cpp"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.hpp"
    "${OPEN_LMM_SOURCE_DIR}/${root}/*.h")
  set(actual)
  foreach(candidate IN LISTS candidates)
    file(READ "${candidate}" contents)
    string(FIND "${contents}" "${pattern}" found)
    if(NOT found EQUAL -1)
      file(RELATIVE_PATH relative_candidate
        "${OPEN_LMM_SOURCE_DIR}" "${candidate}")
      list(APPEND actual "${relative_candidate}")
    endif()
  endforeach()
  list(SORT actual)
  list(SORT expected)
  if(NOT "${actual}" STREQUAL "${expected}")
    message(FATAL_ERROR
      "${description} changed. expected=[${expected}] actual=[${actual}]")
  endif()
endfunction()

# Goal 4 starts by freezing the exact known reverse edges. Later dependency
# correction commits shrink these lists to zero; new callers fail immediately.
assert_exact_matches(
  "GUI controller-internal include baseline"
  "gui" "runtime/control/pipeline_controller.hpp")
assert_exact_matches(
  "execution concrete AlgorithmFactory include baseline"
  "src/runtime/execution" "plugins/host/algorithm_factory.hpp")
assert_exact_matches(
  "reconfigure concrete AlgorithmFactory include baseline"
  "src/config/application" "plugins/host/algorithm_factory.hpp")
assert_exact_matches(
  "projection whole RuntimeState dependency baseline"
  "src/visualization/projection" "runtime/state/runtime_state.hpp")

# Production sources compiled directly into a test executable bypass canonical
# target ownership. Keep the current exception list exact until Goal 5 assigns
# every production translation unit one owner.
file(READ "${OPEN_LMM_SOURCE_DIR}/test/CMakeLists.txt" test_cmake)
set(known_test_compiled_production_sources)
string(REGEX MATCHALL "\\.\\./[A-Za-z0-9_./-]+\\.cpp"
  test_compiled_production_sources "${test_cmake}")
list(REMOVE_DUPLICATES test_compiled_production_sources)
list(SORT test_compiled_production_sources)
list(SORT known_test_compiled_production_sources)
if(NOT "${test_compiled_production_sources}" STREQUAL
       "${known_test_compiled_production_sources}")
  message(FATAL_ERROR
    "test-owned production source baseline changed. expected="
    "[${known_test_compiled_production_sources}] actual="
    "[${test_compiled_production_sources}]")
endif()

file(READ "${OPEN_LMM_SOURCE_DIR}/src/adapters/batch/compat/CMakeLists.txt"
  batch_compat_cmake)
string(FIND "${batch_compat_cmake}"
  "add_library(open_lmm_batch_compat_objects OBJECT" batch_compat_owner)
if(batch_compat_owner EQUAL -1)
  message(FATAL_ERROR
    "canonical batch compatibility owner is missing: open_lmm_batch_compat_objects")
endif()

file(READ "${OPEN_LMM_SOURCE_DIR}/src/plugins/host/CMakeLists.txt"
  plugin_host_cmake)
string(FIND "${plugin_host_cmake}"
  "add_library(open_lmm_plugin_host_objects OBJECT" plugin_host_owner)
if(plugin_host_owner EQUAL -1)
  message(FATAL_ERROR
    "canonical plugin host owner is missing: open_lmm_plugin_host_objects")
endif()

file(READ "${OPEN_LMM_SOURCE_DIR}/src/visualization/CMakeLists.txt"
  visualization_cmake)
string(FIND "${visualization_cmake}"
  "add_library(open_lmm_visualization_projection_objects OBJECT"
  visualization_owner)
if(visualization_owner EQUAL -1)
  message(FATAL_ERROR
    "canonical visualization owner is missing")
endif()

foreach(runtime_owner IN ITEMS
    open_lmm_runtime_state_objects
    open_lmm_runtime_resources_objects
    open_lmm_runtime_execution_objects
    open_lmm_runtime_control_objects
    open_lmm_runtime_service_objects
    open_lmm_runtime_composition_objects)
  file(GLOB_RECURSE runtime_cmake_files
    "${OPEN_LMM_SOURCE_DIR}/src/runtime/*/CMakeLists.txt")
  set(runtime_owner_found FALSE)
  foreach(runtime_cmake_path IN LISTS runtime_cmake_files)
    file(READ "${runtime_cmake_path}" runtime_cmake)
    string(FIND "${runtime_cmake}"
      "add_library(${runtime_owner} OBJECT" found)
    if(NOT found EQUAL -1)
      set(runtime_owner_found TRUE)
    endif()
  endforeach()
  if(NOT runtime_owner_found)
    message(FATAL_ERROR "canonical runtime owner is missing: ${runtime_owner}")
  endif()
endforeach()

file(READ "${OPEN_LMM_SOURCE_DIR}/src/storage/CMakeLists.txt" storage_cmake)
string(FIND "${storage_cmake}"
  "add_library(open_lmm_storage_objects OBJECT" storage_owner)
if(storage_owner EQUAL -1)
  message(FATAL_ERROR
    "canonical storage owner is missing: open_lmm_storage_objects")
endif()

foreach(owner IN ITEMS
    open_lmm_config_document_objects
    open_lmm_config_schema_objects
    open_lmm_config_domain_objects
    open_lmm_config_application_objects)
  string(FIND "${test_cmake}" "${owner}" unexpected_test_owner)
  if(NOT unexpected_test_owner EQUAL -1)
    message(FATAL_ERROR "tests must not own production target: ${owner}")
  endif()
  file(READ "${OPEN_LMM_SOURCE_DIR}/src/config/CMakeLists.txt" config_cmake)
  string(FIND "${config_cmake}" "add_library(${owner} OBJECT" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "canonical config owner is missing: ${owner}")
  endif()
endforeach()
file(READ "${OPEN_LMM_SOURCE_DIR}/src/foundation/CMakeLists.txt"
  foundation_cmake)
foreach(owner IN ITEMS
    open_lmm_foundation_concurrency_objects
    open_lmm_foundation_logging_objects)
  string(FIND "${foundation_cmake}" "add_library(${owner} OBJECT" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "canonical foundation owner is missing: ${owner}")
  endif()
endforeach()
file(READ "${OPEN_LMM_SOURCE_DIR}/src/domain/support/CMakeLists.txt"
  domain_support_cmake)
string(FIND "${domain_support_cmake}"
  "add_library(open_lmm_domain_support_objects OBJECT" found_domain_support)
if(found_domain_support EQUAL -1)
  message(FATAL_ERROR "canonical domain support owner is missing")
endif()
file(READ "${OPEN_LMM_SOURCE_DIR}/src/runtime/client/CMakeLists.txt"
  runtime_client_cmake)
string(FIND "${runtime_client_cmake}"
  "add_library(open_lmm_runtime_client_objects OBJECT" found_runtime_client)
if(found_runtime_client EQUAL -1)
  message(FATAL_ERROR "canonical runtime client owner is missing")
endif()
file(READ "${OPEN_LMM_SOURCE_DIR}/src/adapters/gui/CMakeLists.txt" gui_cmake)
foreach(owner IN ITEMS
    open_lmm_gui_model_objects
    open_lmm_gui_adapter_objects
    open_lmm_gui_presentation_objects)
  string(FIND "${gui_cmake}"
    "openlmm_add_gui_object_owner(${owner}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "canonical GUI owner is missing: ${owner}")
  endif()
endforeach()

# A production translation unit may be absent when its feature is disabled,
# but no configured build may compile the same source into multiple owners.
# Tests have their own sources and consume production code only through the
# canonical targets checked above.
if(OPEN_LMM_BUILD_DIR AND
   EXISTS "${OPEN_LMM_BUILD_DIR}/compile_commands.json")
  file(READ "${OPEN_LMM_BUILD_DIR}/compile_commands.json" compile_commands)
  foreach(root IN ITEMS common core gui server utils src)
    file(GLOB_RECURSE production_sources
      "${OPEN_LMM_SOURCE_DIR}/${root}/*.cpp")
    foreach(source IN LISTS production_sources)
      string(REPLACE "\\" "\\\\" source_pattern "${source}")
      string(REPLACE "." "\\." source_pattern "${source_pattern}")
      string(REGEX MATCHALL
        "\"file\"[ \t]*:[ \t]*\"${source_pattern}\""
        source_compile_entries "${compile_commands}")
      list(LENGTH source_compile_entries source_compile_count)
      if(source_compile_count GREATER 1)
        file(RELATIVE_PATH relative_source
          "${OPEN_LMM_SOURCE_DIR}" "${source}")
        message(FATAL_ERROR
          "production source has multiple compile owners: "
          "${relative_source} (${source_compile_count})")
      endif()
    endforeach()
  endforeach()
endif()

# The contracts/common layer may depend on third-party value types, but must
# never reach upward into a concrete core implementation.
file(GLOB_RECURSE common_sources
  "${OPEN_LMM_SOURCE_DIR}/include/open_lmm/common/*.hpp"
  "${OPEN_LMM_SOURCE_DIR}/include/open_lmm/common/*.h"
  "${OPEN_LMM_SOURCE_DIR}/src/foundation/contracts/*.cpp")
foreach(source IN LISTS common_sources)
  file(READ "${source}" contents)
  string(FIND "${contents}" "open_lmm/core/" found_core_include)
  if(NOT found_core_include EQUAL -1)
    file(RELATIVE_PATH relative_source "${OPEN_LMM_SOURCE_DIR}" "${source}")
    message(FATAL_ERROR
      "${relative_source} reaches upward into an open_lmm/core implementation")
  endif()
endforeach()

# Domain support still consumes the foundation logger through its private
# canonical include path; the façade DSO dependency remains explicit.
assert_exact_matches(
  "legacy pipeline to foundation logging include baseline"
  "src/runtime/execution/legacy_pipeline" "foundation/logging/logging.hpp"
  "src/runtime/execution/legacy_pipeline/pipeline.cpp")
assert_exact_matches(
  "domain support to foundation logging include baseline"
  "src/domain/support" "foundation/logging/logging.hpp"
  "src/domain/support/registration_log.cpp")
assert_exact_matches(
  "plugin host public-contract include baseline"
  "src/plugins/host" "open_lmm/common/"
  "src/plugins/host/algorithm_provider.hpp"
  "src/plugins/host/load_module.hpp"
  "src/plugins/host/plugin_support.hpp")
file(READ "${OPEN_LMM_SOURCE_DIR}/src/domain/support/CMakeLists.txt"
  common_target_cmake)
string(FIND "${common_target_cmake}" "open_lmm_utils" common_logging_link)
if(common_logging_link EQUAL -1)
  message(FATAL_ERROR
    "open_lmm_common must declare its foundation logging dependency")
endif()

# Core algorithms and built-in plugins consume validated snapshots only.
file(GLOB_RECURSE core_sources
  "${OPEN_LMM_SOURCE_DIR}/src/domain/*.cpp"
  "${OPEN_LMM_SOURCE_DIR}/src/domain/*.hpp"
  "${OPEN_LMM_SOURCE_DIR}/src/domain/*.h"
  "${OPEN_LMM_SOURCE_DIR}/src/plugins/*.cpp"
  "${OPEN_LMM_SOURCE_DIR}/src/plugins/*.hpp"
  "${OPEN_LMM_SOURCE_DIR}/src/plugins/*.h")
foreach(source IN LISTS core_sources)
  file(READ "${source}" contents)
  foreach(pattern
      "GlobalConfig"
      "load_module_from_so"
      "create_descriptor_kdtree_module"
      "create_dynamic_remover_module"
      "createInstance("
      "SetCancellationToken("
      "LoopDetectorInput"
      "loadRawScanData")
    string(FIND "${contents}" "${pattern}" found)
    if(NOT found EQUAL -1)
      file(RELATIVE_PATH relative_source "${OPEN_LMM_SOURCE_DIR}" "${source}")
      message(FATAL_ERROR
        "${relative_source} bypasses typed config/plugin ABI v1 with ${pattern}")
    endif()
  endforeach()
endforeach()

assert_file_excludes(
  "src/runtime/execution/legacy_pipeline/pipeline.hpp"
  "spdlog/"
  "fmt/"
  "common/profiling.hpp"
  "loop_detector_base.hpp"
  "kiss_matcher/"
)

assert_file_excludes(
  "src/runtime/execution/nodes/map_update_node.hpp"
  "spdlog/"
  "fmt/"
  "pcl/io/"
  "common/profiling.hpp"
  "dynamic_remover_base.hpp"
)

assert_file_excludes(
  "include/open_lmm/common/data_types.hpp"
  "scan_context.h"
)

assert_file_excludes(
  "src/domain/support/registration.cpp"
  "spdlog/"
  "fmt/"
  "pcl/common/transforms.h"
  "pcl/io/"
)

# Keep template-heavy fmt/spdlog implementation details behind the lightweight
# foundation logging API. A direct include in a large translation unit has
# previously triggered GCC optimizer ICEs at Release optimization levels.
file(GLOB_RECURSE open_lmm_sources
  "${OPEN_LMM_SOURCE_DIR}/*.cpp"
  "${OPEN_LMM_SOURCE_DIR}/*.hpp"
  "${OPEN_LMM_SOURCE_DIR}/*.h")
foreach(source IN LISTS open_lmm_sources)
  if(source STREQUAL
     "${OPEN_LMM_SOURCE_DIR}/src/foundation/logging/logging.cpp")
    continue()
  endif()
  file(READ "${source}" contents)
  foreach(pattern "spdlog/" "spdlog::" "fmt/" "fmt::")
    string(FIND "${contents}" "${pattern}" found)
    if(NOT found EQUAL -1)
      file(RELATIVE_PATH relative_source "${OPEN_LMM_SOURCE_DIR}" "${source}")
      message(FATAL_ERROR
        "${relative_source} bypasses utils/logging.hpp with ${pattern}")
    endif()
  endforeach()
endforeach()

assert_file_excludes(
  "CMakeLists.txt"
  "add_compile_options($<$<CONFIG:Release>:-O3>)"
  "install(DIRECTORY \${PROJECT_SOURCE_DIR}/\${public_header_dir}/")

file(READ "${OPEN_LMM_SOURCE_DIR}/src/foundation/contracts/CMakeLists.txt"
  common_cmake)
string(REGEX MATCH
  "add_library\\(open_lmm_contracts SHARED[^)]*\\)" contracts_sources
  "${common_cmake}")
foreach(forbidden "CMAKE_DL_LIBS")
  string(FIND "${contracts_sources}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "contracts target contains runtime plugin-host dependency: ${forbidden}")
  endif()
endforeach()
string(FIND "${common_cmake}"
  "target_link_libraries(open_lmm_contracts" contracts_link_dependencies)
if(NOT contracts_link_dependencies EQUAL -1)
  message(FATAL_ERROR
    "contracts target must not publish or link runtime dependencies")
endif()

file(READ
  "${OPEN_LMM_SOURCE_DIR}/test/package_consumer/public_header_allowlist.txt"
  public_header_allowlist)
foreach(forbidden
    "plugin_support.hpp"
    "stage_executor.hpp"
    "runtime_state.hpp"
    "server/execution/"
    "runtime/execution/")
  string(FIND "${public_header_allowlist}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "public header allowlist exposes implementation detail: ${forbidden}")
  endif()
endforeach()

# MapServer is the stable command/query port façade only; mutable
# session/runtime responsibilities belong to StageExecutor and RuntimeStateStore.
file(READ "${OPEN_LMM_SOURCE_DIR}/src/adapters/batch/compat/map_server.hpp"
  map_server_header)
foreach(expected
    "std::unique_ptr<StageExecutor> executor_"
    "class MapServer final : public StageRuntimePort")
  string(FIND "${map_server_header}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "MapServer façade must contain: ${expected}")
  endif()
endforeach()
foreach(forbidden "RuntimeStateStore" "OutputRepository" "state_mutex_")
  string(FIND "${map_server_header}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "MapServer façade must not own runtime detail: ${forbidden}")
  endif()
endforeach()

# Session execution owns an injected immutable bootstrap configuration. No
# process-global configuration singleton is allowed in production code.
assert_file_excludes(
  "src/config/document/config.hpp"
  "GlobalConfig")
assert_file_excludes(
  "src/config/document/config.cpp"
  "GlobalConfig")
assert_file_excludes(
  "src/runtime/execution/stage_executor.hpp"
  "GlobalConfig")
assert_file_excludes(
  "src/runtime/execution/stage_executor.cpp"
  "GlobalConfig")
assert_file_excludes(
  "src/runtime/service/runtime_service.hpp"
  "GlobalConfig"
  "open_lmm/core/")
assert_file_excludes(
  "src/runtime/service/runtime_service.cpp"
  "GlobalConfig"
  "open_lmm/core/")
file(READ "${OPEN_LMM_SOURCE_DIR}/src/runtime/service/runtime_service.hpp"
  runtime_service_header)
foreach(expected
    "class RuntimeService"
    "CloseMode"
    "std::shared_ptr<RuntimeInstance> active_"
    "LifecycleState"
    "transition_generation_")
  string(FIND "${runtime_service_header}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "RuntimeService contract must contain: ${expected}")
  endif()
endforeach()

foreach(removed_internal_path
    "server/session_state.hpp"
    "server/session_manager.hpp"
    "server/runtime_session_client.hpp")
  if(EXISTS "${OPEN_LMM_SOURCE_DIR}/${removed_internal_path}")
    message(FATAL_ERROR
      "single-runtime migration must remove ${removed_internal_path}")
  endif()
endforeach()
file(READ "${OPEN_LMM_SOURCE_DIR}/src/runtime/state/runtime_state_store.hpp"
  runtime_state_store_header)
foreach(expected "class RuntimeStateStore" "std::shared_ptr<const RuntimeState>")
  string(FIND "${runtime_state_store_header}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "RuntimeStateStore contract must contain: ${expected}")
  endif()
endforeach()

file(READ "${OPEN_LMM_SOURCE_DIR}/include/open_lmm/common/runtime_contracts.hpp"
  runtime_contracts_header)
string(FIND "${runtime_contracts_header}" "SessionId" found_session_id)
if(NOT found_session_id EQUAL -1)
  message(FATAL_ERROR "Single-runtime contracts must not expose SessionId")
endif()
string(FIND "${runtime_contracts_header}" "struct JobHandle" found_job_handle)
if(found_job_handle EQUAL -1)
  message(FATAL_ERROR "Single-runtime contracts must expose JobHandle")
endif()
foreach(lightweight_header IN ITEMS
    include/open_lmm/common/agent_id.hpp
    include/open_lmm/common/result.hpp
    include/open_lmm/common/runtime_contracts.hpp
    include/open_lmm/server/runtime_client.hpp)
  assert_file_excludes("${lightweight_header}"
    "Eigen/" "pcl/" "gtsam/" "rclcpp/" "open_lmm/core/")
endforeach()

file(READ "${OPEN_LMM_SOURCE_DIR}/src/runtime/resources/resource_governor.hpp"
  resource_governor_header)
foreach(expected
    "struct ResourceBudget"
    "max_agent_tasks"
    "max_cpu_threads"
    "soft_memory_bytes"
    "BoundedExecutor agent_executor_"
    "AcquireHeavyMemoryPhase")
  string(FIND "${resource_governor_header}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "ResourceGovernor contract must contain: ${expected}")
  endif()
endforeach()
assert_file_excludes("src/runtime/resources/resource_governor.hpp"
  "max_active_sessions" "TryAcquireSession" "ReleaseSession")
assert_file_excludes(
  "src/foundation/concurrency/bounded_executor.cpp"
  "std::async"
  ".detach(")

# Normal stage transitions must go through the presentation generation owner.
# A GUI event handler may request a new snapshot, but must not globally clear the
# last valid presentation before the replacement is ready.
assert_file_excludes(
  "src/adapters/gui/iridescence/iridescence_gui.cpp"
  "ClearVisualizationLayers")
file(READ "${OPEN_LMM_SOURCE_DIR}/src/runtime/execution/stage_executor.cpp"
  stage_executor_source)
file(READ "${OPEN_LMM_SOURCE_DIR}/src/runtime/execution/stages/data_load_executor.cpp"
  data_load_executor_source)
file(READ "${OPEN_LMM_SOURCE_DIR}/src/runtime/execution/stages/map_update_executor.cpp"
  map_update_executor_source)
foreach(expected "AgentExecutor" "MemoryClass::kResidentPayload")
  string(FIND "${data_load_executor_source}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "bounded DataLoad executor must contain: ${expected}")
  endif()
endforeach()
foreach(expected "AgentExecutor" "internal_cpu_threads"
                 "kInstanceIsolatedParallel" "AcquireHeavyMemoryPhase")
  string(FIND "${map_update_executor_source}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "bounded MapUpdate executor must contain: ${expected}")
  endif()
endforeach()
foreach(forbidden "runParallelDataLoad" "runParallelMapUpdate"
                  "root_config_" "config_map_server_" "output_save_dir_")
  string(FIND "${stage_executor_source}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "thin StageExecutor must not retain responsibility: ${forbidden}")
  endif()
endforeach()
file(READ "${OPEN_LMM_SOURCE_DIR}/src/domain/data_loader/data_loader_base.hpp"
  data_loader_header)
string(FIND "${data_loader_header}" "VisitRawScanData" raw_scan_streaming)
if(raw_scan_streaming EQUAL -1)
  message(FATAL_ERROR "MapUpdate raw scans must expose bounded streaming")
endif()
file(READ "${OPEN_LMM_SOURCE_DIR}/src/runtime/execution/stages/alignment_executor.cpp"
  alignment_executor_source)
string(FIND "${alignment_executor_source}" "AgentExecutor" parallel_alignment)
if(NOT parallel_alignment EQUAL -1)
  message(FATAL_ERROR "Alignment must remain sequential")
endif()

file(READ "${OPEN_LMM_SOURCE_DIR}/cmake/options.cmake" build_options)
foreach(expected
    "OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT"
    "OPEN_LMM_BUILD_DESCRIPTOR_SOLID"
    "OPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS"
    "OPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP"
    "OPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD"
    "OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM"
    "OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR"
    "OPEN_LMM_ENABLE_ASAN_UBSAN"
    "OPEN_LMM_ENABLE_TSAN")
  string(FIND "${build_options}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "required build option is missing: ${expected}")
  endif()
endforeach()

file(READ "${OPEN_LMM_SOURCE_DIR}/../ros/CMakeLists.txt" ros_cmake)
string(FIND "${ros_cmake}"
  "find_package(open_lmm CONFIG REQUIRED COMPONENTS client)"
  found_open_lmm_package)
if(found_open_lmm_package EQUAL -1)
  message(FATAL_ERROR "ROS must consume the installed open_lmm package")
endif()
string(FIND "${ros_cmake}" "open_lmm::utils" found_ros_utils)
if(NOT found_ros_utils EQUAL -1)
  message(FATAL_ERROR "ROS adapter must not link the unused utils façade")
endif()
string(FIND "${ros_cmake}" "add_subdirectory(" found_ros_subdirectory)
if(NOT found_ros_subdirectory EQUAL -1)
  message(FATAL_ERROR "ROS must not embed open_lmm with add_subdirectory")
endif()
string(FIND "${ros_cmake}"
  "set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} -O3\")" found_ros_o3)
if(NOT found_ros_o3 EQUAL -1)
  message(FATAL_ERROR "ROS CMake must not append a global -O3 flag")
endif()

# A component is a typed public RuntimeClient adapter, not a hidden batch
# launcher or a consumer of internal RuntimeService headers.
# The standalone open_lmm_batch executable owns the direct synchronous path.
assert_file_excludes(
  "../ros/ros2/open_lmm_ros/open_lmm_ros.cpp"
  "map_server->process()"
  "gui_auto_run"
  "MapServer"
  "PipelineController"
  "ReplacePorts"
  "std_srvs"
  "Trigger"
  "std_msgs::msg::String")
file(READ "${OPEN_LMM_SOURCE_DIR}/../ros/ros2/open_lmm_ros/open_lmm_ros.cpp"
  ros_adapter)
foreach(expected
    "RuntimeClient"
    "create_server<ExecutePipeline>"
    "\"~/execute\""
    "\"~/status\""
    "\"~/events\""
    "runtime_->Submit("
    "runtime_->Cancel(")
  string(FIND "${ros_adapter}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "ROS command adapter must contain: ${expected}")
  endif()
endforeach()
assert_file_excludes(
  "../ros/ros2/open_lmm_ros/open_lmm_ros.cpp"
  "GuiRuntimeHost"
  "gui_plugin_path"
  "open_lmm/gui/")
file(READ
  "${OPEN_LMM_SOURCE_DIR}/../ros/ros2/open_lmm_ros/open_lmm_ros_gui.cpp"
  ros_gui_composition)
foreach(expected "GuiRuntimeHost" "open_lmm/gui/gui_runtime_host.hpp")
  string(FIND "${ros_gui_composition}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR
      "optional ROS GUI composition must contain: ${expected}")
  endif()
endforeach()
assert_file_excludes(
  "../ros/ros2/open_lmm_ros/open_lmm_ros.hpp"
  "server/runtime_service.hpp"
  "runtime/service/runtime_service.hpp"
  "gui_controller_bridge.hpp"
  "gui_plugin_host.hpp"
  "gui_runtime_host.hpp"
  "RuntimeSessionClient")
file(READ "${OPEN_LMM_SOURCE_DIR}/src/adapters/batch/CMakeLists.txt"
  batch_cmake)
string(FIND "${batch_cmake}" "add_executable(open_lmm_batch main.cpp)"
  found_batch_launcher)
if(found_batch_launcher EQUAL -1)
  message(FATAL_ERROR "standalone open_lmm_batch launcher must be built")
endif()

file(READ "${OPEN_LMM_SOURCE_DIR}/../docker/open_lmm.Dockerfile" dockerfile)
foreach(expected
    "update-alternatives --set gcc /usr/bin/gcc-12"
    "update-alternatives --set g++ /usr/bin/g++-12"
    "ENV CC=/usr/bin/gcc-12"
    "ENV CXX=/usr/bin/g++-12")
  string(FIND "${dockerfile}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "Docker compiler default must contain: ${expected}")
  endif()
endforeach()
