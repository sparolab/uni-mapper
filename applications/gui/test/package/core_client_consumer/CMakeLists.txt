cmake_minimum_required(VERSION 3.25)
project(open_lmm_core_client_consumer LANGUAGES CXX)

find_package(open_lmm 3.0.0 EXACT CONFIG REQUIRED COMPONENTS client)
if(TARGET open_lmm::gui)
  message(FATAL_ERROR "client-only discovery unexpectedly imported GUI")
endif()
