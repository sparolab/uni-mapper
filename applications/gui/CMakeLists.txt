cmake_minimum_required(VERSION 3.25)
project(open_lmm_gui VERSION 3.0.0 LANGUAGES C CXX)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

option(OPEN_LMM_GUI_BUILD_IRIDESCENCE
  "Build the optional Iridescence GUI plugin" ON)
option(OPEN_LMM_GUI_USE_SYSTEM_IRIDESCENCE
  "Use a system-installed Iridescence package" ON)
option(OPEN_LMM_GUI_WARNINGS_AS_ERRORS
  "Treat warnings in OpenLMM GUI sources as errors" OFF)

if(DEFINED OPEN_LMM_BUILD_IRIDESCENCE_GUI)
  message(DEPRECATION
    "OPEN_LMM_BUILD_IRIDESCENCE_GUI moved to the standalone GUI package; "
    "use OPEN_LMM_GUI_BUILD_IRIDESCENCE")
  set(OPEN_LMM_GUI_BUILD_IRIDESCENCE
    "${OPEN_LMM_BUILD_IRIDESCENCE_GUI}" CACHE BOOL
    "Build the optional Iridescence GUI plugin" FORCE)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH OFF)

find_package(open_lmm ${PROJECT_VERSION} EXACT CONFIG REQUIRED
  COMPONENTS client plugin_sdk utils)
find_package(Threads REQUIRED)

include(cmake/OpenLmmGuiWarnings.cmake)

add_library(open_lmm_gui_core SHARED
  src/config_editor.cpp
  src/host/gui_controller_bridge.cpp
  src/host/gui_plugin_host.cpp
  src/host/gui_plugin_module.cpp
  src/host/gui_runtime_host.cpp
  src/model/gui_event_queue.cpp
  src/model/gui_model.cpp
  src/presentation/map_presentation_state.cpp
  src/presentation/visualization_repository.cpp
  src/presentation/visualization_snapshot_worker.cpp)
target_include_directories(open_lmm_gui_core
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
  PRIVATE
    ${PROJECT_SOURCE_DIR}/src)
target_link_libraries(open_lmm_gui_core
  PUBLIC open_lmm::client
  PRIVATE open_lmm::plugin_sdk open_lmm::utils ${CMAKE_DL_LIBS}
          Threads::Threads)
target_compile_features(open_lmm_gui_core PUBLIC cxx_std_20)
openlmm_gui_enable_warnings(open_lmm_gui_core)
set_target_properties(open_lmm_gui_core PROPERTIES
  EXPORT_NAME gui
  VERSION ${PROJECT_VERSION}
  SOVERSION ${PROJECT_VERSION_MAJOR}
  INSTALL_RPATH "$ORIGIN")
add_library(open_lmm_gui::gui ALIAS open_lmm_gui_core)

if(OPEN_LMM_GUI_BUILD_IRIDESCENCE)
  if(OPEN_LMM_GUI_USE_SYSTEM_IRIDESCENCE)
    find_package(Iridescence CONFIG REQUIRED NAMES iridescence Iridescence)
  else()
    include(thirdparty/iridescence/iridescence.cmake)
  endif()
  find_package(spdlog REQUIRED)

  add_library(open_lmm_iridescence_gui SHARED
    src/iridescence/application_log.cpp
    src/iridescence/iridescence_gui.cpp)
  target_include_directories(open_lmm_iridescence_gui PRIVATE
    ${PROJECT_SOURCE_DIR}/src)
  target_link_libraries(open_lmm_iridescence_gui PRIVATE
    open_lmm_gui_core Iridescence::Iridescence spdlog::spdlog)
  target_compile_features(open_lmm_iridescence_gui PRIVATE cxx_std_20)
  openlmm_gui_enable_warnings(open_lmm_iridescence_gui)
  set_target_properties(open_lmm_iridescence_gui PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION ${PROJECT_VERSION_MAJOR}
    INSTALL_RPATH "$ORIGIN")
endif()

add_executable(open_lmm_gui_app src/main.cpp)
target_compile_features(open_lmm_gui_app PRIVATE cxx_std_20)
target_compile_definitions(open_lmm_gui_app PRIVATE
  OPEN_LMM_GUI_INSTALL_LIBDIR="${CMAKE_INSTALL_LIBDIR}")
target_link_libraries(open_lmm_gui_app PRIVATE open_lmm_gui_core)
openlmm_gui_enable_warnings(open_lmm_gui_app)
set_target_properties(open_lmm_gui_app PROPERTIES
  OUTPUT_NAME open_lmm_gui
  INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")

install(TARGETS open_lmm_gui_core
  EXPORT OpenLmmGuiTargets
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT GuiRuntime
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT GuiDevelopment
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT GuiRuntime
  INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
if(TARGET open_lmm_iridescence_gui)
  install(TARGETS open_lmm_iridescence_gui
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT GuiPlugins
    ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR} COMPONENT GuiPlugins
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT GuiPlugins)
endif()
install(TARGETS open_lmm_gui_app
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT GuiApplication)

install(FILES
  include/open_lmm/gui/gui_plugin.hpp
  include/open_lmm/gui/gui_runtime_host.hpp
  DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/open_lmm/gui
  COMPONENT GuiDevelopment)

configure_package_config_file(
  cmake/OpenLmmGuiConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/OpenLmmGuiConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_DATADIR}/OpenLmmGui/cmake)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/OpenLmmGuiConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(EXPORT OpenLmmGuiTargets
  FILE OpenLmmGuiTargets.cmake
  NAMESPACE open_lmm_gui::
  DESTINATION ${CMAKE_INSTALL_DATADIR}/OpenLmmGui/cmake
  COMPONENT GuiDevelopment)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/OpenLmmGuiConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/OpenLmmGuiConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_DATADIR}/OpenLmmGui/cmake
  COMPONENT GuiDevelopment)
install(FILES
  packaging/LICENCE
  packaging/RELEASE_POLICY.md
  packaging/THIRD_PARTY_NOTICES.md
  cmake/open_lmm-gui-install-components.txt
  DESTINATION ${CMAKE_INSTALL_DATADIR}/open_lmm_gui
  COMPONENT GuiRuntime)
install(FILES README.md
  DESTINATION ${CMAKE_INSTALL_DATADIR}/open_lmm_gui
  COMPONENT GuiApplication)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
