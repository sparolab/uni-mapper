cmake_minimum_required(VERSION 3.25)
project(open_lmm_gui_unqualified_consumer LANGUAGES C CXX)

find_package(open_lmm 3.0.0 EXACT CONFIG REQUIRED)
if(NOT TARGET open_lmm::gui OR NOT TARGET open_lmm_gui::gui)
  message(FATAL_ERROR
    "combined unqualified discovery must preserve the v3 GUI target")
endif()
