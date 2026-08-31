cmake_minimum_required(VERSION 3.5)
project(open_lmm_ros VERSION 3.0.0 LANGUAGES C CXX)

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
find_package(open_lmm ${PROJECT_VERSION} EXACT CONFIG REQUIRED COMPONENTS client)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(rosidl_default_generators REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ExecutionEvent.msg"
  "srv/GetRuntimeStatus.srv"
  "action/ExecutePipeline.action"
)

### open_lmm_ros ###
ament_auto_add_library(open_lmm_ros_component SHARED
  ${ros_version}/open_lmm_ros/runtime_adapter/open_lmm_ros.cpp
  ${ros_version}/open_lmm_ros/runtime_adapter/ros_visualization_bridge.cpp
)
set_target_properties(open_lmm_ros_component PROPERTIES
  OUTPUT_NAME open_lmm_ros
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
)
target_link_libraries(open_lmm_ros_component
  open_lmm::client
)
rosidl_get_typesupport_target(open_lmm_ros_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(open_lmm_ros_component "${open_lmm_ros_typesupport_target}")
rclcpp_components_register_nodes(open_lmm_ros_component "open_lmm::OpenLMMROS")

ament_auto_add_library(open_lmm_rviz_control_panel SHARED
  ${ros_version}/open_lmm_ros/rviz_control/open_lmm_control_panel.cpp
)
target_link_libraries(open_lmm_rviz_control_panel
  Qt5::Widgets
  "${open_lmm_ros_typesupport_target}"
)
pluginlib_export_plugin_description_file(rviz_common rviz_plugin_description.xml)

if(BUILD_TESTING)
  find_package(Threads REQUIRED)
  include(${CMAKE_CURRENT_SOURCE_DIR}/../open_lmm/test/cmake/OpenLmmTest.cmake)
  ament_auto_add_executable(open_lmm_ros_visualization_bridge_tests
    test/contract/visualization_bridge_tests.cpp)
  target_link_libraries(open_lmm_ros_visualization_bridge_tests
    open_lmm_ros_component)
  openlmm_add_test(
    NAME open_lmm_ros_visualization_bridge_tests
    TARGET open_lmm_ros_visualization_bridge_tests
    LAYER L5 MODULE adapters.ros OWNER RosVisualizationBridge
    INVARIANTS INV-06 INV-07 INV-09 INV-13 LANES pr)

  add_executable(open_lmm_ros_goal_admission_tests
    test/contract/goal_admission_tests.cpp)
  ament_target_dependencies(open_lmm_ros_goal_admission_tests rclcpp_action)
  target_link_libraries(open_lmm_ros_goal_admission_tests
    Threads::Threads
    open_lmm::client)
  openlmm_add_test(
    NAME open_lmm_ros_goal_admission_tests
    TARGET open_lmm_ros_goal_admission_tests
    LAYER L5 MODULE adapters.ros OWNER RosGoalCoordinator
    INVARIANTS INV-13 INV-17 LANES pr SANITIZERS tsan)

  ament_auto_add_executable(open_lmm_ros_control_panel_model_tests
    test/contract/control_panel_model_tests.cpp)
  target_link_libraries(open_lmm_ros_control_panel_model_tests
    "${open_lmm_ros_typesupport_target}")
  openlmm_add_test(
    NAME open_lmm_ros_control_panel_model_tests
    TARGET open_lmm_ros_control_panel_model_tests
    LAYER L5 MODULE adapters.ros OWNER OpenLmmControlPanel
    INVARIANTS INV-07 INV-13 INV-17 LANES pr)

  ament_auto_add_executable(open_lmm_ros_control_panel_plugin_tests
    test/contract/control_panel_plugin_tests.cpp)
  target_link_libraries(open_lmm_ros_control_panel_plugin_tests
    open_lmm_rviz_control_panel
    Qt5::Widgets)
  openlmm_add_test(
    NAME open_lmm_ros_control_panel_plugin_tests
    TARGET open_lmm_ros_control_panel_plugin_tests
    LAYER L5 MODULE adapters.ros OWNER OpenLmmControlPanel
    INVARIANTS INV-07 INV-13 LANES pr)

  ament_auto_add_executable(open_lmm_ros_runtime_adapter_graph_tests
    test/integration/runtime_adapter_graph_tests.cpp)
  target_include_directories(open_lmm_ros_runtime_adapter_graph_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/../open_lmm/test)
  target_link_libraries(open_lmm_ros_runtime_adapter_graph_tests
    open_lmm_ros_component
    open_lmm::client
    "${open_lmm_ros_typesupport_target}")
  openlmm_add_test(
    NAME open_lmm_ros_runtime_adapter_graph_tests
    TARGET open_lmm_ros_runtime_adapter_graph_tests
    LAYER L3 MODULE adapters.ros OWNER OpenLMMROS
    INVARIANTS INV-05 INV-07 INV-08 INV-13 INV-17 INV-18
    LANES pr TIMEOUT 60)
  file(MAKE_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/ros-test-log)
  openlmm_apply_test_environment(
    "LD_LIBRARY_PATH=$<TARGET_FILE_DIR:open_lmm_ros_component>:$<TARGET_FILE_DIR:open_lmm::client>:$ENV{LD_LIBRARY_PATH};ROS_LOG_DIR=${CMAKE_CURRENT_BINARY_DIR}/ros-test-log;QT_QPA_PLATFORM=offscreen")
  openlmm_finalize_test_manifest()
endif()

### open_lmm_rosnode ###
ament_auto_add_executable(open_lmm_rosnode
  ${ros_version}/open_lmm_ros/runtime_adapter/open_lmm_rosnode.cpp
)
target_link_libraries(open_lmm_rosnode
  open_lmm_ros_component
)

# Install launch files.
install(DIRECTORY
  ${ros_version}/launch
  ${ros_version}/rviz
  DESTINATION share/${PROJECT_NAME}/
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
ament_auto_package()
