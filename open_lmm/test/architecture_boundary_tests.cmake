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

file(READ "${OPEN_LMM_SOURCE_DIR}/../ros/CMakeLists.txt" ros_cmake)
string(FIND "${ros_cmake}"
  "set(CMAKE_CXX_FLAGS \"\${CMAKE_CXX_FLAGS} -O3\")" found_ros_o3)
if(NOT found_ros_o3 EQUAL -1)
  message(FATAL_ERROR "ROS CMake must not append a global -O3 flag")
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
