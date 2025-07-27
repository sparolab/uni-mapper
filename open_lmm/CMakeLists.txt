cmake_minimum_required(VERSION 3.5)
project(open_lmm VERSION 1.0.0 LANGUAGES C CXX)

#############
## Options ##
#############
# Ccache option
option(USE_CCACHE "Build using Ccache if found on the path" ON)
# Mandantory : use below options to build the project
option(USE_SYSTEM_EIGEN3 "Use system pre-installed Eigen" ON)
option(USE_SYSTEM_TBB "Use system pre-installed oneAPI/tbb" ON)
option(USE_SYSTEM_PCL "Use system pre-installed PCL" ON)
option(USE_SYSTEM_GTSAM "Use system pre-installed GTSAM" ON)  # tested on v4.2a9
option(USE_SYSTEM_NANOFLANN "Use system pre-installed nanoflann" OFF)  # fetched from v1.5.5
option(USE_SYSTEM_TQDMCPP "Use system pre-installed tqdmcpp" OFF)  # fetched from custom tqdmcpp
# Non-mandantory
# - [OFF] will fetch the dependencies (recommended)
# - [ON] make sure you have the below dependencies installed to your system
option(USE_SYSTEM_SMALL_GICP "Use system pre-installed small_gicp" OFF)
option(USE_SYSTEM_KISS_MATCHER "Use system pre-installed kiss_matcher" OFF)
option(USE_SYSTEM_NLOHMANN_JSON "Use system pre-installed nlohmann_json" OFF)

# ccache setup
if(USE_CCACHE)
  find_program(CCACHE_PATH ccache)
  if(CCACHE_PATH)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE ccache)
    set_property(GLOBAL PROPERTY RULE_LAUNCH_LINK ccache)
    message(STATUS "Using ccache: ${CCACHE_PATH}")
  endif()
endif()

# Set build
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set open_lmm dynamic loading create libraries list
set(OPEN_LMM_CREATE_LIBRARIES "" CACHE INTERNAL "List of all OpenLMM create libraries")

# Custom cmake settings and find dependencies
include(cmake/CompilerOptions.cmake)
include(thirdparty/find_dependencies.cmake)


# cmake policy
# cmake_policy(SET CMP0076 NEW)

add_subdirectory(utils)
add_subdirectory(common)
add_subdirectory(core)
add_subdirectory(server)

# TODO(gil) : make it can build only selected models
#! install create_*.so libraries for ROS build
foreach(create_target ${OPEN_LMM_CREATE_LIBRARIES})
  install(TARGETS ${create_target}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )
endforeach()

### if ROS-related ###
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # ROS2
    find_package(ament_cmake REQUIRED)
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
    ament_package()
  elseif($ENV{ROS_VERSION} EQUAL 1)
    # ROS1 : not supported
    # find_package(catkin REQUIRED)
    # install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif()
endif()


#! for cpp only
# find_package(spdlog REQUIRED)
# find_package(fmt REQUIRED)

# add_executable(main main.cpp)
# target_link_libraries(main PRIVATE
#   open_lmm_utils
#   open_lmm_map_server
#   spdlog::spdlog
#   fmt::fmt
#   tqdmcpp::tqdmcpp
# )