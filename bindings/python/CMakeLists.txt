cmake_minimum_required(VERSION 3.25)
project(open_lmm_python VERSION 3.0.0 LANGUAGES CXX)

include(GNUInstallDirs)

option(OPEN_LMM_PYTHON_WARNINGS_AS_ERRORS
  "Treat warnings in the OpenLMM Python native bridge as errors" OFF)
option(OPEN_LMM_PYTHON_STAGE_LOCAL_WHEEL_RUNTIME
  "Stage the reviewed same-image OpenLMM runtime closure" OFF)
option(OPEN_LMM_PYTHON_WHEEL_ONLY
  "Register only the installed-wheel contract test" OFF)
option(OPEN_LMM_PYTHON_CORE_CONTRACT_ONLY
  "Register only the seven native bridge sanitizer contracts" OFF)
set(OPEN_LMM_CORE_PREFIX "" CACHE PATH
  "Exact installed OpenLMM core prefix consumed by this binding")
set(OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON "" CACHE FILEPATH
  "Python executable from a venv containing the local OpenLMM wheel")
set(OPEN_LMM_PYTHON_WHEEL_FILE "" CACHE FILEPATH
  "Built local wheel consumed by the closure contract test")
set(OPEN_LMM_REPOSITORY_ROOT "" CACHE PATH
  "Repository root used only by replay/benchmark integration tests")
set(OPEN_LMM_CORE_TEST_BUILD_ROOT "" CACHE PATH
  "Core build root used only by replay/benchmark integration tests")
set(OPEN_LMM_REPLAY_COMPARE "" CACHE FILEPATH
  "Goal 03 replay comparator used only by integration tests")
set(OPEN_LMM_PYTHON_TEST_LD_PRELOAD "" CACHE FILEPATH
  "Sanitizer runtime preloaded only into source-test Python processes")
set(OPEN_LMM_PYTHON_TEST_LIBRARY_PATH "" CACHE PATH
  "Installed runtime library directory exposed only to source tests")

set(CMAKE_BUILD_RPATH_USE_ORIGIN ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH OFF)

find_package(Python3 3.10 EXACT
  COMPONENTS Interpreter Development.Module NumPy REQUIRED)
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 2.9 CONFIG REQUIRED)
find_package(open_lmm ${PROJECT_VERSION} EXACT CONFIG REQUIRED
  COMPONENTS client)

get_filename_component(_open_lmm_package_share "${open_lmm_DIR}/.." ABSOLUTE)
get_filename_component(_open_lmm_share "${_open_lmm_package_share}/.." ABSOLUTE)
get_filename_component(_open_lmm_inferred_prefix "${_open_lmm_share}/.." ABSOLUTE)
if(OPEN_LMM_CORE_PREFIX)
  file(REAL_PATH "${OPEN_LMM_CORE_PREFIX}" _open_lmm_requested_prefix)
  file(REAL_PATH "${_open_lmm_inferred_prefix}" _open_lmm_resolved_prefix)
  if(NOT _open_lmm_requested_prefix STREQUAL _open_lmm_resolved_prefix)
    message(FATAL_ERROR
      "OPEN_LMM_CORE_PREFIX does not own the discovered package: "
      "${_open_lmm_requested_prefix} != ${_open_lmm_resolved_prefix}")
  endif()
else()
  set(OPEN_LMM_CORE_PREFIX "${_open_lmm_inferred_prefix}")
endif()

set(_open_lmm_imported_client_location "")
foreach(_open_lmm_imported_config IN ITEMS
    RELEASE RELWITHDEBINFO DEBUG MINSIZEREL NOCONFIG)
  get_target_property(_open_lmm_candidate_client_location
    open_lmm::client "IMPORTED_LOCATION_${_open_lmm_imported_config}")
  if(_open_lmm_candidate_client_location AND
     EXISTS "${_open_lmm_candidate_client_location}")
    set(_open_lmm_imported_client_location
      "${_open_lmm_candidate_client_location}")
    break()
  endif()
endforeach()
if(NOT _open_lmm_imported_client_location OR
   NOT EXISTS "${_open_lmm_imported_client_location}")
  message(FATAL_ERROR
    "installed open_lmm::client has no imported artifact: "
    "${_open_lmm_imported_client_location}")
endif()
file(REAL_PATH "${OPEN_LMM_CORE_PREFIX}" _open_lmm_prefix_real)
file(REAL_PATH "${_open_lmm_imported_client_location}"
  _open_lmm_imported_client_real)
string(FIND "${_open_lmm_imported_client_real}" "${_open_lmm_prefix_real}/"
  _open_lmm_imported_client_prefix_index)
if(NOT _open_lmm_imported_client_prefix_index EQUAL 0)
  message(FATAL_ERROR
    "open_lmm::client resolves outside OPEN_LMM_CORE_PREFIX: "
    "${_open_lmm_imported_client_real}")
endif()

include(cmake/OpenLmmPythonWarnings.cmake)

pybind11_add_module(open_lmm_python_native MODULE
  native/module.cpp
  native/model_conversion.cpp
  native/numpy_conversion.cpp
  native/runtime_binding.cpp)
target_compile_features(open_lmm_python_native PRIVATE cxx_std_20)
target_link_libraries(open_lmm_python_native PRIVATE open_lmm::client)
target_include_directories(open_lmm_python_native PRIVATE
  ${Python3_NumPy_INCLUDE_DIRS})
target_compile_definitions(open_lmm_python_native PRIVATE
  OPEN_LMM_PYTHON_VERSION="${PROJECT_VERSION}")
openlmm_python_enable_warnings(open_lmm_python_native)
if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  target_compile_options(open_lmm_python_native PRIVATE
    "-ffile-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=."
    "-fmacro-prefix-map=${CMAKE_CURRENT_SOURCE_DIR}=.")
endif()
set_target_properties(open_lmm_python_native PROPERTIES
  OUTPUT_NAME _native
  LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/python/open_lmm"
  BUILD_RPATH "$ORIGIN;$<TARGET_FILE_DIR:open_lmm::client>"
  INSTALL_RPATH "$ORIGIN/.libs")

set(OPEN_LMM_PYTHON_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}")
set(OPEN_LMM_PYTHON_STAGE_DIR "${PROJECT_BINARY_DIR}/python")
set(OPEN_LMM_PYTHON_EXECUTABLE "${Python3_EXECUTABLE}")

file(GLOB_RECURSE _open_lmm_python_package_files CONFIGURE_DEPENDS
  "${OPEN_LMM_PYTHON_SOURCE_DIR}/package/open_lmm/*.py"
  "${OPEN_LMM_PYTHON_SOURCE_DIR}/package/open_lmm/*.json"
  "${OPEN_LMM_PYTHON_SOURCE_DIR}/package/open_lmm/py.typed")
add_custom_target(open_lmm_python_package ALL
  COMMAND ${CMAKE_COMMAND} -E make_directory
    "${OPEN_LMM_PYTHON_STAGE_DIR}/open_lmm"
  COMMAND ${CMAKE_COMMAND} -E copy_directory
    "${OPEN_LMM_PYTHON_SOURCE_DIR}/package/open_lmm"
    "${OPEN_LMM_PYTHON_STAGE_DIR}/open_lmm"
  DEPENDS ${_open_lmm_python_package_files}
  COMMENT "Staging the OpenLMM Python package"
  VERBATIM)
add_dependencies(open_lmm_python_native open_lmm_python_package)

install(TARGETS open_lmm_python_native
  LIBRARY DESTINATION open_lmm COMPONENT Python)
install(DIRECTORY "${OPEN_LMM_PYTHON_SOURCE_DIR}/package/open_lmm/"
  DESTINATION open_lmm COMPONENT Python
  FILES_MATCHING
    PATTERN "*.py"
    PATTERN "*.json"
    PATTERN "py.typed")

foreach(_open_lmm_release_file IN ITEMS
    LICENCE RELEASE_POLICY.md THIRD_PARTY_NOTICES.md)
  set(_open_lmm_release_path
    "${OPEN_LMM_CORE_PREFIX}/${CMAKE_INSTALL_DATADIR}/open_lmm/${_open_lmm_release_file}")
  if(NOT EXISTS "${_open_lmm_release_path}")
    message(FATAL_ERROR
      "installed core release metadata is missing: ${_open_lmm_release_path}")
  endif()
  install(FILES "${_open_lmm_release_path}"
    DESTINATION open_lmm COMPONENT Python)
endforeach()
install(FILES
  packaging/LOCAL_WHEEL_POLICY.md
  packaging/open_lmm-python-runtime-closure.tsv
  DESTINATION open_lmm COMPONENT Python)

if(OPEN_LMM_PYTHON_STAGE_LOCAL_WHEEL_RUNTIME)
  include(cmake/OpenLmmPythonRuntimeClosure.cmake)
  openlmm_python_install_runtime_closure(
    "${OPEN_LMM_CORE_PREFIX}"
    "${CMAKE_CURRENT_SOURCE_DIR}/packaging/open_lmm-python-runtime-closure.tsv")
endif()

include(CTest)
if(BUILD_TESTING)
  add_subdirectory(test)
endif()
