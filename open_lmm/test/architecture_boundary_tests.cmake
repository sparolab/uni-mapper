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

# The contracts/common layer may depend on third-party value types, but must
# never reach upward into a concrete core implementation.
file(GLOB_RECURSE common_sources
  "${OPEN_LMM_SOURCE_DIR}/common/*.cpp"
  "${OPEN_LMM_SOURCE_DIR}/common/*.hpp"
  "${OPEN_LMM_SOURCE_DIR}/common/*.h")
foreach(source IN LISTS common_sources)
  file(READ "${source}" contents)
  string(FIND "${contents}" "open_lmm/core/" found_core_include)
  if(NOT found_core_include EQUAL -1)
    file(RELATIVE_PATH relative_source "${OPEN_LMM_SOURCE_DIR}" "${source}")
    message(FATAL_ERROR
      "${relative_source} reaches upward into an open_lmm/core implementation")
  endif()
endforeach()

# Core algorithms and built-in plugins consume validated snapshots only.
file(GLOB_RECURSE core_sources
  "${OPEN_LMM_SOURCE_DIR}/core/*.cpp"
  "${OPEN_LMM_SOURCE_DIR}/core/*.hpp"
  "${OPEN_LMM_SOURCE_DIR}/core/*.h")
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
  "common/pipeline.hpp"
  "spdlog/"
  "fmt/"
  "common/profiling.hpp"
  "loop_detector_base.hpp"
  "kiss_matcher/"
)

assert_file_excludes(
  "server/nodes/map_update_node.hpp"
  "spdlog/"
  "fmt/"
  "pcl/io/"
  "common/profiling.hpp"
  "dynamic_remover_base.hpp"
)

assert_file_excludes(
  "common/data_types.hpp"
  "scan_context.h"
)

assert_file_excludes(
  "common/registration.cpp"
  "spdlog/"
  "fmt/"
  "pcl/common/transforms.h"
  "pcl/io/"
)

# Keep template-heavy fmt/spdlog implementation details behind the lightweight
# utils/logging.hpp API. A direct include in a large translation unit has
# previously triggered GCC optimizer ICEs at Release optimization levels.
file(GLOB_RECURSE open_lmm_sources
  "${OPEN_LMM_SOURCE_DIR}/*.cpp"
  "${OPEN_LMM_SOURCE_DIR}/*.hpp"
  "${OPEN_LMM_SOURCE_DIR}/*.h")
foreach(source IN LISTS open_lmm_sources)
  if(source STREQUAL "${OPEN_LMM_SOURCE_DIR}/utils/logging.cpp")
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

file(READ "${OPEN_LMM_SOURCE_DIR}/common/CMakeLists.txt" common_cmake)
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
    "server/execution/")
  string(FIND "${public_header_allowlist}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "public header allowlist exposes implementation detail: ${forbidden}")
  endif()
endforeach()

# MapServer is the stable command/query port façade only; mutable
# session/runtime responsibilities belong to StageExecutor and RuntimeStateStore.
file(READ "${OPEN_LMM_SOURCE_DIR}/server/map_server.hpp" map_server_header)
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
  "utils/config.hpp"
  "GlobalConfig")
assert_file_excludes(
  "utils/config.cpp"
  "GlobalConfig")
assert_file_excludes(
  "server/stage_executor.hpp"
  "GlobalConfig")
assert_file_excludes(
  "server/stage_executor.cpp"
  "GlobalConfig")
assert_file_excludes(
  "server/runtime_service.hpp"
  "GlobalConfig"
  "open_lmm/core/")
assert_file_excludes(
  "server/runtime_service.cpp"
  "GlobalConfig"
  "open_lmm/core/")
file(READ "${OPEN_LMM_SOURCE_DIR}/server/runtime_service.hpp"
  runtime_service_header)
foreach(expected
    "class RuntimeService"
    "CloseMode"
    "std::shared_ptr<RuntimeInstance> active_"
    "replacement_in_progress_")
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
file(READ "${OPEN_LMM_SOURCE_DIR}/server/runtime_state_store.hpp"
  runtime_state_store_header)
foreach(expected "class RuntimeStateStore" "std::shared_ptr<const RuntimeState>")
  string(FIND "${runtime_state_store_header}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "RuntimeStateStore contract must contain: ${expected}")
  endif()
endforeach()

file(READ "${OPEN_LMM_SOURCE_DIR}/common/runtime_contracts.hpp"
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
    common/agent_id.hpp common/result.hpp common/runtime_contracts.hpp
    server/runtime_client.hpp)
  assert_file_excludes("${lightweight_header}"
    "Eigen/" "pcl/" "gtsam/" "rclcpp/" "open_lmm/core/")
endforeach()

file(READ "${OPEN_LMM_SOURCE_DIR}/server/resource_governor.hpp"
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
assert_file_excludes("server/resource_governor.hpp"
  "max_active_sessions" "TryAcquireSession" "ReleaseSession")
assert_file_excludes(
  "utils/bounded_executor.cpp"
  "std::async"
  ".detach(")
file(READ "${OPEN_LMM_SOURCE_DIR}/server/stage_executor.cpp"
  stage_executor_source)
file(READ "${OPEN_LMM_SOURCE_DIR}/server/execution/data_load_executor.cpp"
  data_load_executor_source)
file(READ "${OPEN_LMM_SOURCE_DIR}/server/execution/map_update_executor.cpp"
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
file(READ "${OPEN_LMM_SOURCE_DIR}/core/data_loader/data_loader_base.hpp"
  data_loader_header)
string(FIND "${data_loader_header}" "VisitRawScanData" raw_scan_streaming)
if(raw_scan_streaming EQUAL -1)
  message(FATAL_ERROR "MapUpdate raw scans must expose bounded streaming")
endif()
file(READ "${OPEN_LMM_SOURCE_DIR}/server/execution/alignment_executor.cpp"
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
  "find_package(open_lmm CONFIG REQUIRED COMPONENTS client gui)"
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
    "GuiRuntimeHost"
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
  "../ros/ros2/open_lmm_ros/open_lmm_ros.hpp"
  "server/runtime_service.hpp"
  "gui_controller_bridge.hpp"
  "gui_plugin_host.hpp"
  "RuntimeSessionClient")
file(READ "${OPEN_LMM_SOURCE_DIR}/CMakeLists.txt" core_cmake)
string(FIND "${core_cmake}" "add_executable(open_lmm_batch main.cpp)"
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
