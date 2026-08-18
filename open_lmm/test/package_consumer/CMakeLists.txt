cmake_minimum_required(VERSION 3.16)
project(open_lmm_package_consumer LANGUAGES C CXX)

find_package(open_lmm CONFIG REQUIRED)

foreach(expected_target contracts config core runtime gui utils)
  if(NOT TARGET open_lmm::${expected_target})
    message(FATAL_ERROR
      "Installed package does not export open_lmm::${expected_target}")
  endif()
endforeach()

add_executable(open_lmm_package_consumer main.cpp)
target_compile_features(open_lmm_package_consumer PRIVATE cxx_std_20)
target_link_libraries(open_lmm_package_consumer PRIVATE
  open_lmm::runtime
  open_lmm::gui
)
