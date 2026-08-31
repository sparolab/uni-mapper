cmake_minimum_required(VERSION 3.25)

foreach(required_variable IN ITEMS
    OPEN_LMM_REPOSITORY_ROOT
    OPEN_LMM_CORE_PREFIX
    OPEN_LMM_CORE_BUILD_DIR
    OPEN_LMM_PYTHON_TEST_ROOT)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

set(binding_source "${OPEN_LMM_PYTHON_TEST_ROOT}/source")
set(binding_build "${OPEN_LMM_PYTHON_TEST_ROOT}/build")
set(binding_install "${OPEN_LMM_PYTHON_TEST_ROOT}/install")
set(replay_compare
  "${OPEN_LMM_CORE_BUILD_DIR}/test/open_lmm_replay_compare")
if(NOT EXISTS
    "${OPEN_LMM_CORE_PREFIX}/share/open_lmm/cmake/open_lmmConfig.cmake")
  message(FATAL_ERROR
    "core prefix is not an installed OpenLMM package: ${OPEN_LMM_CORE_PREFIX}")
endif()
if(NOT EXISTS "${replay_compare}")
  message(FATAL_ERROR "replay comparator is missing: ${replay_compare}")
endif()

set(compiler_args)
if(DEFINED OPEN_LMM_PYTHON_CXX_COMPILER AND
   NOT OPEN_LMM_PYTHON_CXX_COMPILER STREQUAL "")
  list(APPEND compiler_args
    "-DCMAKE_CXX_COMPILER=${OPEN_LMM_PYTHON_CXX_COMPILER}")
endif()
if(DEFINED OPEN_LMM_PYTHON_CXX_FLAGS AND
   NOT OPEN_LMM_PYTHON_CXX_FLAGS STREQUAL "")
  list(APPEND compiler_args
    "-DCMAKE_CXX_FLAGS=${OPEN_LMM_PYTHON_CXX_FLAGS}")
endif()

file(REMOVE_RECURSE "${OPEN_LMM_PYTHON_TEST_ROOT}")
file(MAKE_DIRECTORY "${OPEN_LMM_PYTHON_TEST_ROOT}")
file(COPY "${OPEN_LMM_REPOSITORY_ROOT}/bindings/python/"
  DESTINATION "${binding_source}")

execute_process(
  COMMAND "${CMAKE_COMMAND}"
          -S "${binding_source}" -B "${binding_build}"
          "-DCMAKE_PREFIX_PATH=${OPEN_LMM_CORE_PREFIX}"
          "-DOPEN_LMM_CORE_PREFIX=${OPEN_LMM_CORE_PREFIX}"
          "-DOPEN_LMM_REPOSITORY_ROOT=${OPEN_LMM_REPOSITORY_ROOT}"
          "-DOPEN_LMM_CORE_TEST_BUILD_ROOT=${OPEN_LMM_CORE_BUILD_DIR}"
          "-DOPEN_LMM_REPLAY_COMPARE=${replay_compare}"
          -DCMAKE_BUILD_TYPE=RelWithDebInfo
          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
          -DOPEN_LMM_PYTHON_WARNINGS_AS_ERRORS=ON
          -DBUILD_TESTING=ON
          ${compiler_args}
  RESULT_VARIABLE configure_result
  OUTPUT_VARIABLE configure_stdout
  ERROR_VARIABLE configure_stderr)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR
    "standalone Python configure failed: ${configure_result}\n"
    "stdout:\n${configure_stdout}\nstderr:\n${configure_stderr}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${binding_build}" --parallel 1
  RESULT_VARIABLE build_result
  OUTPUT_VARIABLE build_stdout
  ERROR_VARIABLE build_stderr)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR
    "standalone Python build failed: ${build_result}\n"
    "stdout:\n${build_stdout}\nstderr:\n${build_stderr}")
endif()

set(compile_database "${binding_build}/compile_commands.json")
if(NOT EXISTS "${compile_database}")
  message(FATAL_ERROR "standalone Python compile database is missing")
endif()
file(READ "${compile_database}" compile_commands)
foreach(forbidden_reference IN ITEMS
    "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src"
    "${OPEN_LMM_CORE_BUILD_DIR}")
  string(FIND "${compile_commands}" "${forbidden_reference}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "Python compile database retained private path: ${forbidden_reference}")
  endif()
endforeach()
foreach(source IN ITEMS
    model_conversion.cpp module.cpp numpy_conversion.cpp runtime_binding.cpp)
  string(REGEX MATCHALL
    "\"file\"[ \t]*:[ \t]*\"${binding_source}/native/${source}\""
    source_entries "${compile_commands}")
  list(LENGTH source_entries source_count)
  if(NOT source_count EQUAL 1)
    message(FATAL_ERROR
      "Python source must have exactly one compile owner: ${source} (${source_count})")
  endif()
endforeach()

set(link_command
  "${binding_build}/CMakeFiles/open_lmm_python_native.dir/link.txt")
if(NOT EXISTS "${link_command}")
  message(FATAL_ERROR "standalone Python native link command is missing")
endif()
file(READ "${link_command}" link_contents)
string(FIND "${link_contents}"
  "${OPEN_LMM_CORE_PREFIX}/lib/libopen_lmm_client.so.3.0.0"
  installed_client_link)
if(installed_client_link EQUAL -1)
  message(FATAL_ERROR
    "Python native bridge did not link the installed open_lmm::client artifact")
endif()
foreach(forbidden_reference IN ITEMS
    "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src"
    "${OPEN_LMM_CORE_BUILD_DIR}")
  string(FIND "${link_contents}" "${forbidden_reference}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "Python link command retained private path: ${forbidden_reference}")
  endif()
endforeach()

execute_process(
  COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${binding_build}"
          --output-on-failure
          --output-junit "${OPEN_LMM_PYTHON_TEST_ROOT}/ctest.xml"
  RESULT_VARIABLE ctest_result
  OUTPUT_VARIABLE ctest_stdout
  ERROR_VARIABLE ctest_stderr)
if(NOT ctest_result EQUAL 0)
  message(FATAL_ERROR
    "standalone Python behavior tests failed: ${ctest_result}\n"
    "stdout:\n${ctest_stdout}\nstderr:\n${ctest_stderr}")
endif()

file(MAKE_DIRECTORY "${binding_install}/open_lmm")
set(user_file "${binding_install}/open_lmm/user-owned-file")
file(WRITE "${user_file}" "must survive Python install\n")
foreach(install_attempt RANGE 1 2)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" --install "${binding_build}"
            --prefix "${binding_install}" --component Python
    RESULT_VARIABLE install_result
    OUTPUT_VARIABLE install_stdout
    ERROR_VARIABLE install_stderr)
  if(NOT install_result EQUAL 0)
    message(FATAL_ERROR
      "standalone Python install ${install_attempt} failed: ${install_result}\n"
      "stdout:\n${install_stdout}\nstderr:\n${install_stderr}")
  endif()
  if(NOT EXISTS "${user_file}")
    message(FATAL_ERROR "Python install removed an unknown user file")
  endif()
endforeach()

file(GLOB native_modules
  "${binding_build}/python/open_lmm/_native*.so")
list(LENGTH native_modules native_module_count)
if(NOT native_module_count EQUAL 1)
  message(FATAL_ERROR
    "standalone Python build must produce one native module: ${native_modules}")
endif()
list(GET native_modules 0 native_module)
find_program(OPEN_LMM_PYTHON_READELF readelf REQUIRED)
execute_process(
  COMMAND "${OPEN_LMM_PYTHON_READELF}" -d "${native_module}"
  RESULT_VARIABLE readelf_result
  OUTPUT_VARIABLE dynamic_section
  ERROR_VARIABLE readelf_stderr)
if(NOT readelf_result EQUAL 0 OR
   NOT dynamic_section MATCHES "(RPATH|RUNPATH).*(\\$ORIGIN)")
  message(FATAL_ERROR
    "standalone Python module has an unexpected RUNPATH\n"
    "${dynamic_section}${readelf_stderr}")
endif()
foreach(forbidden_reference IN ITEMS
    "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src"
    "${OPEN_LMM_CORE_BUILD_DIR}")
  string(FIND "${dynamic_section}" "${forbidden_reference}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "Python module RUNPATH retained private path: ${forbidden_reference}")
  endif()
endforeach()

find_program(OPEN_LMM_PYTHON_LDD ldd REQUIRED)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env --unset=LD_LIBRARY_PATH --unset=PYTHONPATH
          "${OPEN_LMM_PYTHON_LDD}" "${native_module}"
  RESULT_VARIABLE ldd_result
  OUTPUT_VARIABLE ldd_stdout
  ERROR_VARIABLE ldd_stderr)
if(NOT ldd_result EQUAL 0 OR ldd_stdout MATCHES "not found")
  message(FATAL_ERROR
    "standalone Python module has unresolved dependencies\n"
    "${ldd_stdout}${ldd_stderr}")
endif()

message(STATUS "standalone Python package contract passed (18 tests)")
