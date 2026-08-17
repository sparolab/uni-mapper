cmake_minimum_required(VERSION 3.5)
project(open_lmm VERSION 1.0.0 LANGUAGES C CXX)

# 빌드 설정
set(CMAKE_CXX_STANDARD 20)
add_compile_options($<$<CONFIG:Release>:-O3>)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Set open_lmm dynamic loading create libraries list
set(OPEN_LMM_CREATE_LIBRARIES "" CACHE INTERNAL "List of all OpenLMM create libraries")

# CMake 레이어 — options → externals → internals 순서로 로드
include(cmake/options.cmake)
include(cmake/externals.cmake)
include(cmake/internals.cmake)


# cmake policy
# cmake_policy(SET CMP0076 NEW)

add_subdirectory(utils)
add_subdirectory(common)
add_subdirectory(core)
add_subdirectory(server)
add_subdirectory(gui)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

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
