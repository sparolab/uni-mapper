cmake_minimum_required(VERSION 3.25)
project(open_lmm_cli VERSION 3.0.0 LANGUAGES CXX)

include(GNUInstallDirs)

option(OPEN_LMM_CLI_WARNINGS_AS_ERRORS
  "Treat warnings in the OpenLMM CLI translation unit as errors" OFF)

set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH OFF)

find_package(open_lmm ${PROJECT_VERSION} EXACT CONFIG REQUIRED
  COMPONENTS client)

include(cmake/OpenLmmCliWarnings.cmake)

add_executable(open_lmm_batch src/main.cpp)
target_compile_features(open_lmm_batch PRIVATE cxx_std_20)
target_link_libraries(open_lmm_batch PRIVATE open_lmm::client)
openlmm_cli_enable_warnings(open_lmm_batch)
set_target_properties(open_lmm_batch PROPERTIES
  INSTALL_RPATH "$ORIGIN/../${CMAKE_INSTALL_LIBDIR}")

install(TARGETS open_lmm_batch
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR} COMPONENT Tools)
install(FILES
  README.md
  packaging/LICENCE
  packaging/RELEASE_POLICY.md
  packaging/THIRD_PARTY_NOTICES.md
  cmake/open_lmm-cli-install-components.txt
  DESTINATION ${CMAKE_INSTALL_DATADIR}/open_lmm-cli
  COMPONENT Tools)

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
