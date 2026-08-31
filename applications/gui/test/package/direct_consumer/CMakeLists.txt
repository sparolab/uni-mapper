cmake_minimum_required(VERSION 3.25)
project(open_lmm_gui_direct_consumer LANGUAGES C CXX)

find_package(OpenLmmGui 3.0.0 EXACT CONFIG REQUIRED COMPONENTS gui)

if(NOT TARGET open_lmm_gui::gui)
  message(FATAL_ERROR "canonical GUI target is missing")
endif()
get_target_property(gui_type open_lmm_gui::gui TYPE)
if(NOT gui_type STREQUAL "SHARED_LIBRARY")
  message(FATAL_ERROR "canonical GUI target must be SHARED_LIBRARY: ${gui_type}")
endif()

add_executable(gui_direct main.cpp)
target_compile_features(gui_direct PRIVATE cxx_std_20)
target_link_libraries(gui_direct PRIVATE open_lmm_gui::gui)
file(GENERATE OUTPUT "${CMAKE_CURRENT_BINARY_DIR}/gui-target-file.txt"
  CONTENT "$<TARGET_FILE:open_lmm_gui::gui>\n")
