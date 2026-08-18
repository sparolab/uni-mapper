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
  COMMAND "${consumer_build}/open_lmm_package_consumer"
  RESULT_VARIABLE run_result)
if(NOT run_result EQUAL 0)
  message(FATAL_ERROR "package consumer run failed: ${run_result}")
endif()
