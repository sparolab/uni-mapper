foreach(required_variable
    OPEN_LMM_BUILD_DIR OPEN_LMM_SOURCE_DIR OPEN_LMM_PACKAGE_TEST_ROOT)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

set(install_prefix "${OPEN_LMM_PACKAGE_TEST_ROOT}/install")
set(consumer_source "${OPEN_LMM_PACKAGE_TEST_ROOT}/consumer-source")
set(consumer_build "${OPEN_LMM_PACKAGE_TEST_ROOT}/consumer-build")
set(consumer_compiler_args)
if(DEFINED OPEN_LMM_CONSUMER_C_COMPILER)
  list(APPEND consumer_compiler_args
    "-DCMAKE_C_COMPILER=${OPEN_LMM_CONSUMER_C_COMPILER}")
endif()
if(DEFINED OPEN_LMM_CONSUMER_CXX_COMPILER)
  list(APPEND consumer_compiler_args
    "-DCMAKE_CXX_COMPILER=${OPEN_LMM_CONSUMER_CXX_COMPILER}")
endif()
if(DEFINED OPEN_LMM_CONSUMER_CXX_FLAGS)
  list(APPEND consumer_compiler_args
    "-DCMAKE_CXX_FLAGS=${OPEN_LMM_CONSUMER_CXX_FLAGS}")
endif()
file(REMOVE_RECURSE "${OPEN_LMM_PACKAGE_TEST_ROOT}")
file(MAKE_DIRECTORY "${consumer_source}")
file(COPY "${OPEN_LMM_SOURCE_DIR}/test/package_consumer/"
     DESTINATION "${consumer_source}")

execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${OPEN_LMM_BUILD_DIR}"
          --prefix "${install_prefix}"
  RESULT_VARIABLE install_result)
if(NOT install_result EQUAL 0)
  message(FATAL_ERROR "open_lmm package install failed: ${install_result}")
endif()

set(targets_file
  "${install_prefix}/share/open_lmm/cmake/open_lmmTargets.cmake")
if(NOT EXISTS "${targets_file}")
  message(FATAL_ERROR "installed target export is missing: ${targets_file}")
endif()
set(batch_launcher "${install_prefix}/bin/open_lmm_batch")
if(NOT EXISTS "${batch_launcher}")
  message(FATAL_ERROR "installed batch launcher is missing: ${batch_launcher}")
endif()

foreach(release_file IN ITEMS
    LICENCE RELEASE_POLICY.md THIRD_PARTY_NOTICES.md
    open_lmm-install-components.txt)
  if(NOT EXISTS "${install_prefix}/share/open_lmm/${release_file}")
    message(FATAL_ERROR
      "installed release metadata is missing: ${release_file}")
  endif()
endforeach()
file(READ
  "${install_prefix}/share/open_lmm/open_lmm-install-components.txt"
  install_components)
foreach(component IN ITEMS Runtime Development Plugins Tools)
  string(FIND "${install_components}" "${component}" component_found)
  if(component_found EQUAL -1)
    message(FATAL_ERROR
      "installed component manifest is missing: ${component}")
  endif()
endforeach()
foreach(third_party_license IN ITEMS
    thirdparty/eigen/LICENSE
    thirdparty/gtsam/LICENSE
    thirdparty/kiss_matcher/LICENSE
    thirdparty/nlohmann_json/LICENSE
    erasor/LICENCE)
  if(NOT EXISTS
      "${install_prefix}/share/open_lmm/licenses/${third_party_license}")
    message(FATAL_ERROR
      "installed third-party license is missing: ${third_party_license}")
  endif()
endforeach()

set(versioned_runtime_libraries
  open_lmm_common
  open_lmm_algorithm_config
  open_lmm_utils
  open_lmm_data_loader
  open_lmm_loop_detector
  open_lmm_backend_optimizer
  open_lmm_dynamic_remover
  open_lmm_map_server
  open_lmm_gui_core)
find_program(OPEN_LMM_READELF readelf REQUIRED)
foreach(library IN LISTS versioned_runtime_libraries)
  set(library_base "lib${library}.so")
  set(library_major "${install_prefix}/lib/${library_base}.1")
  set(library_full "${install_prefix}/lib/${library_base}.1.0.0")
  if(NOT EXISTS "${library_major}" OR NOT EXISTS "${library_full}")
    message(FATAL_ERROR
      "versioned runtime library is missing: ${library_base}")
  endif()
  execute_process(
    COMMAND "${OPEN_LMM_READELF}" -d "${library_full}"
    RESULT_VARIABLE readelf_result
    OUTPUT_VARIABLE dynamic_section)
  if(NOT readelf_result EQUAL 0 OR
     NOT dynamic_section MATCHES "SONAME.*\\[${library_base}\\.1\\]")
    message(FATAL_ERROR
      "runtime library has an unexpected SONAME: ${library_full}")
  endif()
endforeach()

foreach(plugin IN ITEMS create_scan_context create_free_dom)
  if(NOT EXISTS "${install_prefix}/lib/lib${plugin}.so.1.0.0")
    message(FATAL_ERROR "versioned plugin entry is missing: ${plugin}")
  endif()
endforeach()

execute_process(
  COMMAND "${batch_launcher}" --help
  RESULT_VARIABLE batch_help_result)
if(NOT batch_help_result EQUAL 0)
  message(FATAL_ERROR
    "installed batch launcher could not run: ${batch_help_result}")
endif()
file(READ "${targets_file}" targets_contents)
string(FIND "${targets_contents}" "${OPEN_LMM_SOURCE_DIR}" source_reference)
if(NOT source_reference EQUAL -1)
  message(FATAL_ERROR "installed targets retain a source-tree reference")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -S "${consumer_source}" -B "${consumer_build}"
          "-DCMAKE_PREFIX_PATH=${install_prefix}"
          ${consumer_compiler_args}
  RESULT_VARIABLE configure_result)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR "package consumer configure failed: ${configure_result}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${consumer_build}" --parallel 1
  RESULT_VARIABLE build_result)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR "package consumer build failed: ${build_result}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH
          "${consumer_build}/open_lmm_package_consumer"
  RESULT_VARIABLE run_result)
if(NOT run_result EQUAL 0)
  message(FATAL_ERROR "package consumer run failed: ${run_result}")
endif()
