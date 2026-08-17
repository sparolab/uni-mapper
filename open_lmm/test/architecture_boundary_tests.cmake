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
