cmake_minimum_required(VERSION 3.5)
project(open_lmm_ros VERSION 2.0.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
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
find_package(open_lmm CONFIG REQUIRED COMPONENTS client gui)
ament_auto_find_build_dependencies()
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExecutionEvent.msg"
  "srv/GetRuntimeStatus.srv"
  "action/ExecutePipeline.action"
)

### open_lmm_ros ###
ament_auto_add_library(open_lmm_ros_component SHARED
  ${ros_version}/open_lmm_ros/open_lmm_ros.cpp
)
set_target_properties(open_lmm_ros_component PROPERTIES
  OUTPUT_NAME open_lmm_ros
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)
target_link_libraries(open_lmm_ros_component
  open_lmm::client
  open_lmm::gui
)
rosidl_get_typesupport_target(open_lmm_ros_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(open_lmm_ros_component "${open_lmm_ros_typesupport_target}")
rclcpp_components_register_nodes(open_lmm_ros_component "open_lmm::OpenLMMROS")

if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  add_executable(open_lmm_ros_goal_admission_tests
    test/goal_admission_tests.cpp)
  target_link_libraries(open_lmm_ros_goal_admission_tests PRIVATE Threads::Threads)
  add_test(NAME open_lmm_ros_goal_admission_tests
    COMMAND open_lmm_ros_goal_admission_tests)
endif()

### open_lmm_rosnode ###
ament_auto_add_executable(open_lmm_rosnode
  ${ros_version}/open_lmm_ros/open_lmm_rosnode.cpp
)
target_link_libraries(open_lmm_rosnode
  open_lmm_ros_component
)

# Install launch files.
install(DIRECTORY
  # ${ros_version}/launch
  # ${ros_version}/rviz
  DESTINATION share/${PROJECT_NAME}/
)
ament_auto_package()
