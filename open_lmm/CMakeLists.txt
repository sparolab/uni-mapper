cmake_minimum_required(VERSION 3.5)
project(open_lmm VERSION 3.0.0 LANGUAGES C CXX)

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

add_subdirectory(src/foundation)
add_subdirectory(src/config)
add_subdirectory(src/domain/support)
add_subdirectory(src/domain/data_loader)
add_subdirectory(src/domain/descriptor)
add_subdirectory(src/domain/alignment)
add_subdirectory(src/plugins/descriptor)
add_subdirectory(src/domain/loop_detection)
add_subdirectory(src/domain/optimization)
add_subdirectory(src/plugins/remover)
add_subdirectory(src/domain/dynamic_removal)
add_subdirectory(src/plugins/host)
add_subdirectory(src/storage)
add_subdirectory(src/runtime)
add_subdirectory(src/visualization)

add_library(open_lmm_client SHARED
  $<TARGET_OBJECTS:open_lmm_runtime_client_objects>)
add_dependencies(open_lmm_client open_lmm_map_server)
target_link_libraries(open_lmm_client PUBLIC open_lmm_contracts Eigen3::Eigen PRIVATE
  "$<BUILD_INTERFACE:$<TARGET_FILE:open_lmm_map_server>>"
  nlohmann_json::nlohmann_json)
target_include_directories(open_lmm_client PRIVATE
  "$<TARGET_PROPERTY:Eigen3::Eigen,INTERFACE_INCLUDE_DIRECTORIES>"
  "$<TARGET_PROPERTY:PCL::PCL,INTERFACE_INCLUDE_DIRECTORIES>"
  "$<TARGET_PROPERTY:gtsam,INTERFACE_INCLUDE_DIRECTORIES>")
target_link_options(open_lmm_client INTERFACE
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_map_server>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_utils>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_common>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_algorithm_config>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_backend_optimizer>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_loop_detector>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_data_loader>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_dynamic_remover>>"
  "$<BUILD_INTERFACE:LINKER:-rpath-link,$<TARGET_FILE_DIR:open_lmm_descriptor>>")
openlmm_set_global_target_properties(open_lmm_client)
set_target_properties(open_lmm_client PROPERTIES EXPORT_NAME client)

# Stable package-facing target names. Implementation-only static libraries and
# fetched dependency targets stay behind these shared-library boundaries.
add_library(open_lmm_core INTERFACE)
target_link_libraries(open_lmm_core INTERFACE
  open_lmm_algorithm_config
  open_lmm_descriptor
  open_lmm_alignment
)
set_target_properties(open_lmm_core PROPERTIES EXPORT_NAME core)

set_target_properties(open_lmm_contracts PROPERTIES EXPORT_NAME contracts)
set_target_properties(open_lmm_common PROPERTIES EXPORT_NAME common)
set_target_properties(open_lmm_utils PROPERTIES EXPORT_NAME utils)
set_target_properties(open_lmm_map_server PROPERTIES EXPORT_NAME runtime)
add_library(open_lmm_plugin_sdk INTERFACE)
target_include_directories(open_lmm_plugin_sdk INTERFACE
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
set_target_properties(open_lmm_plugin_sdk PROPERTIES EXPORT_NAME plugin_sdk)

add_library(open_lmm::contracts ALIAS open_lmm_contracts)
add_library(open_lmm::client ALIAS open_lmm_client)
add_library(open_lmm::common ALIAS open_lmm_common)
add_library(open_lmm::plugin_sdk ALIAS open_lmm_plugin_sdk)
add_library(open_lmm::config ALIAS open_lmm_algorithm_config)
add_library(open_lmm::descriptor ALIAS open_lmm_descriptor)
add_library(open_lmm::alignment ALIAS open_lmm_alignment)
add_library(open_lmm::core ALIAS open_lmm_core)
add_library(open_lmm::runtime ALIAS open_lmm_map_server)
add_library(open_lmm::utils ALIAS open_lmm_utils)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()

# Install only the selected ABI entry libraries.
foreach(create_target ${OPEN_LMM_CREATE_LIBRARIES})
  install(TARGETS ${create_target}
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Plugins
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Plugins
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Plugins
  )
endforeach()

install(TARGETS
  open_lmm_contracts
  open_lmm_client
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_descriptor
  open_lmm_alignment
  open_lmm_map_server
  open_lmm_core
  open_lmm_plugin_sdk
  EXPORT open_lmmTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Runtime
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Runtime
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)

# Runtime implementation DSOs are installed for DT_NEEDED resolution but are
# not CMake package targets. Their headers and link contracts remain internal.
install(TARGETS
  open_lmm_data_loader
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Runtime
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Development
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Runtime
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
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Plugins
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT Plugins
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Plugins
  )
endif()

# Package API allowlist.  Do not replace this with install(DIRECTORY): headers
# under runtime/execution, plugin support/host, and concrete
# algorithms are implementation details and must not become accidental ABI.
set(_open_lmm_public_headers
  common/agent_context.hpp
  common/agent_data.hpp
  common/agent_id.hpp
  common/algorithm_execution_context.hpp
  common/algorithm_progress.hpp
  common/alignment_feedback.hpp
  common/alignment_types.hpp
  common/cancellation.hpp
  common/config_transaction.hpp
  common/data_types.hpp
  common/descriptor_index.hpp
  common/result.hpp
  common/rigid_transform.hpp
  common/runtime_contracts.hpp
  common/runtime_api.hpp
  common/visualization_snapshot.hpp
  core/alignment/alignment_decision_policy.hpp
  core/alignment/alignment_proposer.hpp
  core/alignment/loop_constraint_builder.hpp
  core/descriptor/descriptor_artifact.hpp
  core/descriptor/descriptor_engine.hpp
  server/runtime_client.hpp
  utils/config_schema.hpp
)
foreach(public_header IN LISTS _open_lmm_public_headers)
  get_filename_component(public_header_directory "${public_header}" DIRECTORY)
  set(public_header_source
    "${PROJECT_SOURCE_DIR}/include/open_lmm/${public_header}")
  install(FILES "${public_header_source}"
    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/open_lmm/${public_header_directory}"
    COMPONENT Development)
endforeach()

# Remove only paths owned by the former recursive OpenLMM header install. This
# makes an in-place upgrade converge to the allowlist without touching unknown
# files that a user may keep below include/open_lmm.
include("${PROJECT_SOURCE_DIR}/cmake/open_lmm-legacy-installed-headers-1.x.cmake")
include("${PROJECT_SOURCE_DIR}/cmake/open_lmm-legacy-installed-headers-pre-src-layout.cmake")
set(_open_lmm_owned_stale_headers
  ${_open_lmm_legacy_installed_headers_1_x}
  ${_open_lmm_legacy_installed_headers_pre_src_layout})
list(REMOVE_ITEM _open_lmm_owned_stale_headers
  ${_open_lmm_public_headers}
  common/plugin_api.h
  gui/gui_plugin.hpp
  gui/gui_runtime_host.hpp
  )
list(REMOVE_DUPLICATES _open_lmm_owned_stale_headers)
configure_file(
  "${PROJECT_SOURCE_DIR}/cmake/open_lmm-clean-stale-headers.cmake.in"
  "${PROJECT_BINARY_DIR}/open_lmm-clean-stale-headers.cmake" @ONLY)
install(SCRIPT "${PROJECT_BINARY_DIR}/open_lmm-clean-stale-headers.cmake"
  COMPONENT Development)

string(REPLACE ";" "\n" _open_lmm_public_header_manifest
  "${_open_lmm_public_headers};common/plugin_api.h")
file(WRITE "${PROJECT_BINARY_DIR}/open_lmm-public-headers.txt"
  "${_open_lmm_public_header_manifest}\n")
install(FILES "${PROJECT_BINARY_DIR}/open_lmm-public-headers.txt"
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
  COMPONENT Development)
install(FILES
  ${PROJECT_SOURCE_DIR}/include/open_lmm/common/plugin_api.h
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/open_lmm/common
  COMPONENT PluginSDK)
install(DIRECTORY ${nlohmann_json_SOURCE_DIR}/include/nlohmann
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
  COMPONENT Development
)

install(EXPORT open_lmmTargets
  FILE open_lmmTargets.cmake
  NAMESPACE open_lmm::
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake
  COMPONENT Development
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
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/open_lmm-gui-compat.cmake
    DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/cmake
    COMPONENT Development
  )
endif()

install(FILES
  ${PROJECT_SOURCE_DIR}/package/LICENCE
  ${PROJECT_SOURCE_DIR}/package/RELEASE_POLICY.md
  ${PROJECT_SOURCE_DIR}/package/THIRD_PARTY_NOTICES.md
  ${PROJECT_SOURCE_DIR}/cmake/open_lmm-install-components.txt
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}
  COMPONENT Runtime
)
install(DIRECTORY ${PROJECT_SOURCE_DIR}/thirdparty/
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/licenses/thirdparty
  COMPONENT Runtime
  FILES_MATCHING PATTERN "LICENSE"
)
install(FILES
  ${PROJECT_SOURCE_DIR}/src/plugins/remover/erasor/LICENCE
  DESTINATION ${CMAKE_INSTALL_DATADIR}/${PROJECT_NAME}/licenses/erasor
  COMPONENT Runtime
)

### if ROS-related ###
if(DEFINED ENV{ROS_VERSION})
  if($ENV{ROS_VERSION} EQUAL 2)
    # ROS2
    find_package(ament_cmake REQUIRED)
    install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/
      COMPONENT Runtime)
    ament_export_targets(open_lmmTargets HAS_LIBRARY_TARGET)
    ament_package(
      CONFIG_EXTRAS cmake/open_lmm_dependencies.cmake
      CONFIG_EXTRAS_POST cmake/open_lmm-gui-compat.cmake)
  elseif($ENV{ROS_VERSION} EQUAL 1)
    # ROS1 : not supported
    # find_package(catkin REQUIRED)
    # install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
  endif()
endif()


#! for cpp only
# find_package(spdlog REQUIRED)
# find_package(fmt REQUIRED)
