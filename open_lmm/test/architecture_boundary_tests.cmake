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

# Core algorithms and built-in plugins consume validated snapshots only. The
# mutable global configuration remains a runtime bootstrap concern.
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
      "create_dynamic_remover_module")
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
)

# MapServer is the stable StageRunner API only; mutable session/runtime
# responsibilities belong to the internal StageExecutor and SessionManager.
file(READ "${OPEN_LMM_SOURCE_DIR}/server/map_server.hpp" map_server_header)
foreach(expected
    "std::unique_ptr<StageExecutor> executor_"
    "class MapServer final : public StageRunner")
  string(FIND "${map_server_header}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "MapServer façade must contain: ${expected}")
  endif()
endforeach()
foreach(forbidden "SessionManager" "OutputRepository" "state_mutex_")
  string(FIND "${map_server_header}" "${forbidden}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "MapServer façade must not own runtime detail: ${forbidden}")
  endif()
endforeach()

# Session execution owns immutable config state. GlobalConfig is permitted only
# in the legacy MapServer entry constructor for config-directory discovery.
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
    "std::map<SessionId, std::shared_ptr<RuntimeSession>> sessions_")
  string(FIND "${runtime_service_header}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "RuntimeService contract must contain: ${expected}")
  endif()
endforeach()

file(READ "${OPEN_LMM_SOURCE_DIR}/common/runtime_contracts.hpp"
  runtime_contracts_header)
string(FIND "${runtime_contracts_header}" "class SessionId" found_session_id)
if(found_session_id EQUAL -1)
  message(FATAL_ERROR "Lightweight runtime contracts must contain SessionId")
endif()
foreach(lightweight_header IN ITEMS
    common/agent_id.hpp common/result.hpp common/runtime_contracts.hpp
    common/plugin_api_v2.h common/plugin_host_v2.hpp server/runtime_client.hpp)
  assert_file_excludes("${lightweight_header}"
    "Eigen/" "pcl/" "gtsam/" "rclcpp/" "open_lmm/core/")
endforeach()

file(READ "${OPEN_LMM_SOURCE_DIR}/server/resource_governor.hpp"
  resource_governor_header)
foreach(expected
    "struct ResourceBudget"
    "max_active_sessions"
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
assert_file_excludes(
  "utils/bounded_executor.cpp"
  "std::async"
  ".detach(")
file(READ "${OPEN_LMM_SOURCE_DIR}/server/stage_executor.cpp"
  stage_executor_source)
foreach(expected
    "runParallelDataLoad"
    "runParallelMapUpdate"
    "internal_cpu_threads"
    "parallel MapUpdate requires allowlisted remover 'erasor'")
  string(FIND "${stage_executor_source}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "bounded stage execution must contain: ${expected}")
  endif()
endforeach()
file(READ "${OPEN_LMM_SOURCE_DIR}/core/data_loader/data_loader_base.hpp"
  data_loader_header)
string(FIND "${data_loader_header}" "VisitRawScanData" raw_scan_streaming)
if(raw_scan_streaming EQUAL -1)
  message(FATAL_ERROR "MapUpdate raw scans must expose bounded streaming")
endif()
string(FIND "${stage_executor_source}"
  "Result<void> StageExecutor::runAlignmentStage()" alignment_start)
string(FIND "${stage_executor_source}"
  "Result<void> StageExecutor::prepareAlignmentArtifacts" alignment_end)
if(alignment_start EQUAL -1 OR alignment_end EQUAL -1 OR
   alignment_end LESS_EQUAL alignment_start)
  message(FATAL_ERROR "could not locate the ordered Alignment stage boundary")
endif()
math(EXPR alignment_length "${alignment_end} - ${alignment_start}")
string(SUBSTRING "${stage_executor_source}" ${alignment_start}
  ${alignment_length} alignment_stage)
string(FIND "${alignment_stage}" "AgentExecutor" parallel_alignment)
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
string(FIND "${ros_cmake}" "find_package(open_lmm CONFIG REQUIRED)"
  found_open_lmm_package)
if(found_open_lmm_package EQUAL -1)
  message(FATAL_ERROR "ROS must consume the installed open_lmm package")
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

# A component is a command adapter, not a hidden batch launcher. The
# standalone open_lmm_batch executable owns the direct synchronous path.
assert_file_excludes(
  "../ros/ros2/open_lmm_ros/open_lmm_ros.cpp"
  "map_server->process()"
  "gui_auto_run")
file(READ "${OPEN_LMM_SOURCE_DIR}/../ros/ros2/open_lmm_ros/open_lmm_ros.cpp"
  ros_adapter)
foreach(expected
    "\"~/start\""
    "\"~/cancel\""
    "\"~/status\""
    "\"~/progress\""
    "\"~/result\""
    "controller_->SubmitRunAll()"
    "controller_->Cancel(")
  string(FIND "${ros_adapter}" "${expected}" found)
  if(found EQUAL -1)
    message(FATAL_ERROR "ROS command adapter must contain: ${expected}")
  endif()
endforeach()
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
