find_package(Python3 3.10 EXACT COMPONENTS Interpreter Development NumPy REQUIRED)

# pybind11 2.9 supports FindPython mode and is available in the supported
# Ubuntu 22.04 development image. Wheel builds provide the same config target
# through their isolated build environment.
set(PYBIND11_FINDPYTHON ON)
find_package(pybind11 2.9 CONFIG REQUIRED)

set(OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON "" CACHE FILEPATH
  "Python executable from a venv containing the local OpenLMM wheel")
set(OPEN_LMM_PYTHON_EXECUTABLE "${Python3_EXECUTABLE}" CACHE INTERNAL
  "CPython executable used by OpenLMM Python adapter tests" FORCE)

pybind11_add_module(open_lmm_python_native MODULE
  module.cpp
  model_conversion.cpp
  numpy_conversion.cpp
  runtime_binding.cpp)
target_link_libraries(open_lmm_python_native PRIVATE open_lmm_client)
target_include_directories(open_lmm_python_native PRIVATE
  ${Python3_NumPy_INCLUDE_DIRS})
target_compile_definitions(open_lmm_python_native PRIVATE
  OPEN_LMM_PYTHON_VERSION="${PROJECT_VERSION}")
openlmm_set_global_target_properties(open_lmm_python_native)
set_target_properties(open_lmm_python_native PROPERTIES
  OUTPUT_NAME _native
  LIBRARY_OUTPUT_DIRECTORY "${PROJECT_BINARY_DIR}/python/open_lmm"
  BUILD_RPATH "$ORIGIN;$<TARGET_FILE_DIR:open_lmm_client>"
  INSTALL_RPATH "$ORIGIN/.libs")

foreach(python_plugin IN ITEMS create_scan_context create_free_dom)
  if(TARGET ${python_plugin})
    add_dependencies(open_lmm_python_native ${python_plugin})
  endif()
endforeach()

set(OPEN_LMM_PYTHON_SOURCE_DIR "${CMAKE_CURRENT_SOURCE_DIR}"
  CACHE INTERNAL "OpenLMM Python package source directory" FORCE)
set(OPEN_LMM_PYTHON_STAGE_DIR "${PROJECT_BINARY_DIR}/python"
  CACHE INTERNAL "OpenLMM Python package build-stage directory" FORCE)

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
install(FILES
  "${PROJECT_SOURCE_DIR}/../LICENCE"
  "${PROJECT_SOURCE_DIR}/../THIRD_PARTY_NOTICES.md"
  DESTINATION open_lmm COMPONENT Python)
