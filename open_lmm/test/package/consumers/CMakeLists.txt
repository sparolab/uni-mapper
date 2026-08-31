cmake_minimum_required(VERSION 3.16)
project(open_lmm_package_consumer LANGUAGES C CXX)

find_package(open_lmm 3.0 CONFIG REQUIRED)

foreach(expected_target contracts client plugin_sdk common config descriptor alignment core runtime utils)
  if(NOT TARGET open_lmm::${expected_target})
    message(FATAL_ERROR
      "Installed package does not export open_lmm::${expected_target}")
  endif()
endforeach()

add_executable(open_lmm_package_consumer main.cpp)
target_compile_features(open_lmm_package_consumer PRIVATE cxx_std_20)
target_link_libraries(open_lmm_package_consumer PRIVATE
  open_lmm::descriptor
  open_lmm::alignment
  open_lmm::runtime
)

add_executable(open_lmm_contracts_consumer contracts_main.cpp)
target_compile_features(open_lmm_contracts_consumer PRIVATE cxx_std_20)
target_link_libraries(open_lmm_contracts_consumer PRIVATE open_lmm::contracts)

add_executable(open_lmm_client_consumer client_main.cpp)
target_compile_features(open_lmm_client_consumer PRIVATE cxx_std_20)
target_link_libraries(open_lmm_client_consumer PRIVATE open_lmm::client)

add_executable(open_lmm_plugin_sdk_consumer plugin_sdk_main.c)
target_link_libraries(open_lmm_plugin_sdk_consumer PRIVATE open_lmm::plugin_sdk)
