cmake_minimum_required(VERSION 3.5)
project(open_lmm VERSION 1.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# 빌드 설정
set(CMAKE_CXX_STANDARD 20)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_INSTALL_RPATH "$ORIGIN")

# Set open_lmm dynamic loading create libraries list
set(OPEN_LMM_CREATE_LIBRARIES "" CACHE INTERNAL
  "List of all OpenLMM create libraries" FORCE)

# CMake 레이어 — options → externals → internals 순서로 로드
include(cmake/options.cmake)
include(cmake/externals.cmake)
include(cmake/internals.cmake)


# cmake policy
# cmake_policy(SET CMP0076 NEW)

add_subdirectory(utils)
add_subdirectory(common)

add_library(open_lmm_algorithm_config SHARED core/algorithm_config.cpp)
target_link_libraries(open_lmm_algorithm_config PUBLIC
  open_lmm_common
  open_lmm_utils
  Eigen3::Eigen
)
target_compile_definitions(open_lmm_algorithm_config PRIVATE
  OPEN_LMM_HAVE_DESCRIPTOR_SCAN_CONTEXT=$<BOOL:${OPEN_LMM_BUILD_DESCRIPTOR_SCAN_CONTEXT}>
  OPEN_LMM_HAVE_DESCRIPTOR_SOLID=$<BOOL:${OPEN_LMM_BUILD_DESCRIPTOR_SOLID}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_HMM_MOS=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_HMM_MOS}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_DUFOMAP=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_DUFOMAP}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_OTD=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_OTD}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_FREE_DOM=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_FREE_DOM}>
  OPEN_LMM_HAVE_DYNAMIC_REMOVER_ERASOR=$<BOOL:${OPEN_LMM_BUILD_DYNAMIC_REMOVER_ERASOR}>
)
openlmm_set_global_target_properties(open_lmm_algorithm_config)
set_target_properties(open_lmm_algorithm_config PROPERTIES EXPORT_NAME config)

add_subdirectory(core)
add_subdirectory(server)
add_subdirectory(gui)

# Stable package-facing target names. Implementation-only static libraries and
# fetched dependency targets stay behind these shared-library boundaries.
add_library(open_lmm_core INTERFACE)
target_link_libraries(open_lmm_core INTERFACE
  open_lmm_algorithm_config
  open_lmm_data_loader
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
)
set_target_properties(open_lmm_core PROPERTIES EXPORT_NAME core)

set_target_properties(open_lmm_common PROPERTIES EXPORT_NAME contracts)
set_target_properties(open_lmm_utils PROPERTIES EXPORT_NAME utils)
set_target_properties(open_lmm_map_server PROPERTIES EXPORT_NAME runtime)
set_target_properties(open_lmm_gui_core PROPERTIES EXPORT_NAME gui)

add_library(open_lmm::contracts ALIAS open_lmm_common)
add_library(open_lmm::config ALIAS open_lmm_algorithm_config)
add_library(open_lmm::core ALIAS open_lmm_core)
add_library(open_lmm::runtime ALIAS open_lmm_map_server)
add_library(open_lmm::gui ALIAS open_lmm_gui_core)
add_library(open_lmm::utils ALIAS open_lmm_utils)

# The batch launcher is deliberately a core-package executable. ROS component
# construction never reuses it and never starts a batch pipeline implicitly.
add_executable(open_lmm_batch main.cpp)
target_link_libraries(open_lmm_batch PRIVATE
  open_lmm_map_server
  open_lmm_utils
)
openlmm_set_global_target_properties(open_lmm_batch)
set_target_properties(open_lmm_batch PROPERTIES
  INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}"
)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# Install only the selected ABI entry libraries.
foreach(create_target ${OPEN_LMM_CREATE_LIBRARIES})
  install(TARGETS ${create_target}
    LIBRARY DESTINATION lib
    ARCHIVE DESTINATION lib
    RUNTIME DESTINATION bin
  )
endforeach()

install(TARGETS
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_data_loader
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
  open_lmm_map_server
  open_lmm_gui_core
  open_lmm_core
  EXPORT open_lmmTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(TARGETS open_lmm_batch
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

# Runtime-loaded remover plugins depend on their support DSOs. Descriptor
# implementations are static inside their create_*.so entry libraries.
set(_open_lmm_plugin_support_targets)
foreach(plugin_support_target IN ITEMS free_dom hmm_mos dufomap otd erasor)
  if(TARGET ${plugin_support_target})
    list(APPEND _open_lmm_plugin_support_targets ${plugin_support_target})
  endif()
endforeach()
if(_open_lmm_plugin_support_targets)
  install(TARGETS ${_open_lmm_plugin_support_targets}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  )
endif()

foreach(public_header_dir common core gui server utils)
  install(DIRECTORY ${PROJECT_SOURCE_DIR}/${public_header_dir}/
    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/open_lmm/${public_header_dir}
    FILES_MATCHING
      PATTERN "*.hpp"
      PATTERN "*.h"
      PATTERN "stage_executor.hpp" EXCLUDE
  )
endforeach()
install(DIRECTORY ${nlohmann_json_SOURCE_DIR}/include/nlohmann
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

install(EXPORT open_lmmTargets
  FILE open_lmmTargets.cmake
  NAMESPACE open_lmm::
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake
)

write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/open_lmmConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)

if(NOT (DEFINED ENV{ROS_VERSION} AND "$ENV{ROS_VERSION}" EQUAL "2"))
  configure_package_config_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/open_lmm-config.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/open_lmmConfig.cmake
    INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake
  )
  install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/open_lmmConfig.cmake
    ${CMAKE_CURRENT_BINARY_DIR}/open_lmmConfigVersion.cmake
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake
  )
endif()

### if ROS-related ###
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # ROS2
    find_package(ament_cmake REQUIRED)
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
    ament_export_targets(open_lmmTargets HAS_LIBRARY_TARGET)
    ament_package(CONFIG_EXTRAS cmake/open_lmm_dependencies.cmake)
  elseif($ENV{ROS_VERSION} EQUAL 1)
    # ROS1 : not supported
    # find_package(catkin REQUIRED)
    # install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif()
endif()


#! for cpp only
# find_package(spdlog REQUIRED)
# find_package(fmt REQUIRED)
