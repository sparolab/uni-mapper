cmake_minimum_required(VERSION 3.5)
project(open_lmm_ros VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
# set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_LIST_DIR}/cmake")

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(BUILD_WITH_MARCH_NATIVE)
  add_definitions(-march=native)
  add_definitions(-DBUILD_WITH_MARCH_NATIVE)
  set(CMAKE_C_FLAGS "-march=native ${CMAKE_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "-march=native ${CMAKE_CXX_FLAGS}")
endif()

### ROS version check ###
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    message(STATUS "Current ROS version : ROS2")
    set(ros_version "ros2")
  else()
    # ROS1 : not supported
    # message(STATUS "Current ROS version : ROS1")
    # set(ros_version "ros1")
  endif()
endif()

### ROS2 ###
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

### open_lmm ###
if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/../open_lmm/ AND NOT IS_SYMLINK ${CMAKE_CURRENT_SOURCE_DIR})
  add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../open_lmm ${CMAKE_CURRENT_BINARY_DIR}/open_lmm)
endif()

### open_lmm_ros ###
ament_auto_add_library(open_lmm_ros SHARED
  ${ros_version}/open_lmm_ros/open_lmm_ros.cpp
)
target_link_libraries(open_lmm_ros
  open_lmm_utils
  open_lmm_map_server
)
rclcpp_components_register_nodes(open_lmm_ros "open_lmm::OpenLMMROS")

### open_lmm_rosnode ###
ament_auto_add_executable(open_lmm_rosnode
  ${ros_version}/open_lmm_ros/open_lmm_rosnode.cpp
)
target_link_libraries(open_lmm_rosnode
  open_lmm_ros
)

# Install launch files.
install(DIRECTORY
  # ${ros_version}/launch
  # ${ros_version}/rviz
  DESTINATION share/${PROJECT_NAME}/
)
ament_auto_package()
