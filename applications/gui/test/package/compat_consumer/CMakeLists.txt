cmake_minimum_required(VERSION 3.25)
project(open_lmm_gui_boundary LANGUAGES CXX)

find_package(open_lmm 3.0 CONFIG REQUIRED COMPONENTS gui)

if(NOT TARGET open_lmm::gui OR NOT TARGET open_lmm_gui::gui)
  message(FATAL_ERROR "Compat-A targets are missing")
endif()
get_target_property(gui_type open_lmm::gui TYPE)
if(NOT gui_type STREQUAL "SHARED_LIBRARY")
  message(FATAL_ERROR "Compat-A must preserve SHARED_LIBRARY type: ${gui_type}")
endif()

add_library(gui_boundary_plugin MODULE plugin.cpp)
target_compile_features(gui_boundary_plugin PRIVATE cxx_std_20)
target_link_libraries(gui_boundary_plugin PRIVATE
  open_lmm::gui open_lmm::plugin_sdk)

add_executable(gui_boundary main.cpp)
target_compile_features(gui_boundary PRIVATE cxx_std_20)
target_link_libraries(gui_boundary PRIVATE open_lmm::gui)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/gui-target-file.txt"
  CONTENT "$<TARGET_FILE:open_lmm::gui>\n")
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/plugin-target-file.txt"
  CONTENT "$<TARGET_FILE:gui_boundary_plugin>\n")
