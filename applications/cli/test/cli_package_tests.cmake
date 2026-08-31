cmake_minimum_required(VERSION 3.25)

foreach(required_variable IN ITEMS
    OPEN_LMM_REPOSITORY_ROOT
    OPEN_LMM_CORE_PREFIX
    OPEN_LMM_CORE_BUILD_DIR
    OPEN_LMM_CLI_TEST_ROOT)
  if(NOT DEFINED ${required_variable})
    message(FATAL_ERROR "${required_variable} is required")
  endif()
endforeach()

set(cli_source "${OPEN_LMM_CLI_TEST_ROOT}/source")
set(cli_build "${OPEN_LMM_CLI_TEST_ROOT}/build")
set(cli_fixture "${OPEN_LMM_CLI_TEST_ROOT}/installed-contract")
set(core_batch "${OPEN_LMM_CORE_PREFIX}/bin/open_lmm_batch")
set(cli_batch "${OPEN_LMM_CORE_PREFIX}/bin/open_lmm_batch")

if(NOT EXISTS
    "${OPEN_LMM_CORE_PREFIX}/share/open_lmm/cmake/open_lmmConfig.cmake")
  message(FATAL_ERROR
    "core prefix is not an installed OpenLMM package: ${OPEN_LMM_CORE_PREFIX}")
endif()
if(EXISTS "${core_batch}")
  message(FATAL_ERROR "core package still owns bin/open_lmm_batch")
endif()

set(compiler_args)
if(DEFINED OPEN_LMM_CLI_C_COMPILER AND
   NOT OPEN_LMM_CLI_C_COMPILER STREQUAL "")
  list(APPEND compiler_args
    "-DCMAKE_C_COMPILER=${OPEN_LMM_CLI_C_COMPILER}")
endif()
if(DEFINED OPEN_LMM_CLI_CXX_COMPILER AND
   NOT OPEN_LMM_CLI_CXX_COMPILER STREQUAL "")
  list(APPEND compiler_args
    "-DCMAKE_CXX_COMPILER=${OPEN_LMM_CLI_CXX_COMPILER}")
endif()
if(DEFINED OPEN_LMM_CLI_CXX_FLAGS AND
   NOT OPEN_LMM_CLI_CXX_FLAGS STREQUAL "")
  list(APPEND compiler_args
    "-DCMAKE_CXX_FLAGS=${OPEN_LMM_CLI_CXX_FLAGS}")
endif()

file(REMOVE_RECURSE "${OPEN_LMM_CLI_TEST_ROOT}")
file(MAKE_DIRECTORY "${OPEN_LMM_CLI_TEST_ROOT}")
file(COPY "${OPEN_LMM_REPOSITORY_ROOT}/applications/cli/"
  DESTINATION "${cli_source}")

execute_process(
  COMMAND "${CMAKE_COMMAND}"
          -S "${cli_source}" -B "${cli_build}"
          "-DCMAKE_PREFIX_PATH=${OPEN_LMM_CORE_PREFIX}"
          -DCMAKE_BUILD_TYPE=RelWithDebInfo
          -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
          -DOPEN_LMM_CLI_WARNINGS_AS_ERRORS=ON
          -DBUILD_TESTING=ON
          ${compiler_args}
  RESULT_VARIABLE configure_result
  OUTPUT_VARIABLE configure_stdout
  ERROR_VARIABLE configure_stderr)
if(NOT configure_result EQUAL 0)
  message(FATAL_ERROR
    "standalone CLI configure failed: ${configure_result}\n"
    "stdout:\n${configure_stdout}\nstderr:\n${configure_stderr}")
endif()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --build "${cli_build}" --parallel 1
  RESULT_VARIABLE build_result
  OUTPUT_VARIABLE build_stdout
  ERROR_VARIABLE build_stderr)
if(NOT build_result EQUAL 0)
  message(FATAL_ERROR
    "standalone CLI build failed: ${build_result}\n"
    "stdout:\n${build_stdout}\nstderr:\n${build_stderr}")
endif()

set(compile_database "${cli_build}/compile_commands.json")
if(NOT EXISTS "${compile_database}")
  message(FATAL_ERROR "standalone CLI compile database is missing")
endif()
file(READ "${compile_database}" compile_commands)
foreach(forbidden_reference IN ITEMS
    "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src"
    "${OPEN_LMM_CORE_BUILD_DIR}"
    "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/foundation"
    "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/runtime")
  string(FIND "${compile_commands}" "${forbidden_reference}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "CLI compile database retained private path: ${forbidden_reference}")
  endif()
endforeach()
string(FIND "${compile_commands}" "${cli_source}/src/main.cpp"
  staged_source_found)
if(staged_source_found EQUAL -1)
  message(FATAL_ERROR "CLI compile database does not own staged src/main.cpp")
endif()

set(link_command "${cli_build}/CMakeFiles/open_lmm_batch.dir/link.txt")
if(EXISTS "${link_command}")
  file(READ "${link_command}" link_contents)
  foreach(forbidden_reference IN ITEMS
      "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src"
      "${OPEN_LMM_CORE_BUILD_DIR}")
    string(FIND "${link_contents}" "${forbidden_reference}" found)
    if(NOT found EQUAL -1)
      message(FATAL_ERROR
        "CLI link command retained private path: ${forbidden_reference}")
    endif()
  endforeach()
endif()

execute_process(
  COMMAND "${CMAKE_CTEST_COMMAND}" --test-dir "${cli_build}"
          --output-on-failure
          --output-junit "${OPEN_LMM_CLI_TEST_ROOT}/ctest.xml"
  RESULT_VARIABLE ctest_result
  OUTPUT_VARIABLE ctest_stdout
  ERROR_VARIABLE ctest_stderr)
if(NOT ctest_result EQUAL 0)
  message(FATAL_ERROR
    "standalone CLI behavior tests failed: ${ctest_result}\n"
    "stdout:\n${ctest_stdout}\nstderr:\n${ctest_stderr}")
endif()

file(MAKE_DIRECTORY "${OPEN_LMM_CORE_PREFIX}/bin")
set(user_file "${OPEN_LMM_CORE_PREFIX}/bin/user-owned-file")
file(WRITE "${user_file}" "must survive CLI install\n")
execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${cli_build}"
          --prefix "${OPEN_LMM_CORE_PREFIX}" --component Tools
  RESULT_VARIABLE install_result
  OUTPUT_VARIABLE install_stdout
  ERROR_VARIABLE install_stderr)
if(NOT install_result EQUAL 0)
  message(FATAL_ERROR
    "standalone CLI install failed: ${install_result}\n"
    "stdout:\n${install_stdout}\nstderr:\n${install_stderr}")
endif()
if(NOT EXISTS "${cli_batch}")
  message(FATAL_ERROR "CLI install did not create bin/open_lmm_batch")
endif()
if(NOT EXISTS "${user_file}")
  message(FATAL_ERROR "CLI install removed an unknown user file")
endif()

set(cli_manifest "${cli_build}/install_manifest_Tools.txt")
if(NOT EXISTS "${cli_manifest}")
  set(cli_manifest "${cli_build}/install_manifest.txt")
endif()
set(core_manifest "${OPEN_LMM_CORE_BUILD_DIR}/install_manifest.txt")
foreach(manifest IN ITEMS "${cli_manifest}" "${core_manifest}")
  if(NOT EXISTS "${manifest}")
    message(FATAL_ERROR "install manifest is missing: ${manifest}")
  endif()
endforeach()

function(normalize_manifest manifest output_variable)
  file(STRINGS "${manifest}" manifest_paths)
  set(relative_paths)
  string(LENGTH "${OPEN_LMM_CORE_PREFIX}/" prefix_length)
  foreach(manifest_path IN LISTS manifest_paths)
    string(FIND "${manifest_path}" "${OPEN_LMM_CORE_PREFIX}/" prefix_index)
    if(NOT prefix_index EQUAL 0)
      message(FATAL_ERROR
        "manifest path is outside combined prefix: ${manifest_path}")
    endif()
    string(SUBSTRING "${manifest_path}" ${prefix_length} -1 relative_path)
    list(APPEND relative_paths "${relative_path}")
  endforeach()
  list(REMOVE_DUPLICATES relative_paths)
  list(SORT relative_paths)
  set(${output_variable} "${relative_paths}" PARENT_SCOPE)
endfunction()

normalize_manifest("${cli_manifest}" cli_owned_files)
normalize_manifest("${core_manifest}" core_owned_files)
set(expected_cli_owned_files
  bin/open_lmm_batch
  share/open_lmm-cli/LICENCE
  share/open_lmm-cli/README.md
  share/open_lmm-cli/RELEASE_POLICY.md
  share/open_lmm-cli/THIRD_PARTY_NOTICES.md
  share/open_lmm-cli/open_lmm-cli-install-components.txt)
list(SORT expected_cli_owned_files)
if(NOT cli_owned_files STREQUAL expected_cli_owned_files)
  message(FATAL_ERROR
    "CLI owned-file manifest changed\n"
    "expected: ${expected_cli_owned_files}\nactual: ${cli_owned_files}")
endif()
foreach(cli_owned_file IN LISTS cli_owned_files)
  if(cli_owned_file IN_LIST core_owned_files)
    message(FATAL_ERROR
      "core/CLI install ownership collision: ${cli_owned_file}")
  endif()
endforeach()

foreach(metadata IN ITEMS LICENCE RELEASE_POLICY.md THIRD_PARTY_NOTICES.md)
  execute_process(
    COMMAND "${CMAKE_COMMAND}" -E compare_files
      "${OPEN_LMM_REPOSITORY_ROOT}/${metadata}"
      "${OPEN_LMM_CORE_PREFIX}/share/open_lmm-cli/${metadata}"
    RESULT_VARIABLE metadata_compare_result)
  if(NOT metadata_compare_result EQUAL 0)
    message(FATAL_ERROR
      "installed CLI ${metadata} differs from canonical repository metadata")
  endif()
endforeach()

find_program(OPEN_LMM_CLI_READELF readelf REQUIRED)
execute_process(
  COMMAND "${OPEN_LMM_CLI_READELF}" -d "${cli_batch}"
  RESULT_VARIABLE readelf_result
  OUTPUT_VARIABLE dynamic_section
  ERROR_VARIABLE readelf_stderr)
if(NOT readelf_result EQUAL 0 OR
   NOT dynamic_section MATCHES
     "(RPATH|RUNPATH).*\\[\\$ORIGIN/\\.\\./(lib|lib64)\\]")
  message(FATAL_ERROR
    "installed CLI has an unexpected RUNPATH\n"
    "${dynamic_section}${readelf_stderr}")
endif()
foreach(forbidden_reference IN ITEMS
    "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src"
    "${OPEN_LMM_CORE_BUILD_DIR}")
  string(FIND "${dynamic_section}" "${forbidden_reference}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "installed CLI RUNPATH retained private path: ${forbidden_reference}")
  endif()
endforeach()

find_program(OPEN_LMM_CLI_LDD ldd REQUIRED)
execute_process(
  COMMAND "${CMAKE_COMMAND}" -E env
          --unset=LD_LIBRARY_PATH --unset=PYTHONPATH
          "${OPEN_LMM_CLI_LDD}" "${cli_batch}"
  RESULT_VARIABLE ldd_result
  OUTPUT_VARIABLE ldd_stdout
  ERROR_VARIABLE ldd_stderr)
if(NOT ldd_result EQUAL 0 OR ldd_stdout MATCHES "not found")
  message(FATAL_ERROR
    "installed CLI has unresolved dependencies\n${ldd_stdout}${ldd_stderr}")
endif()
foreach(forbidden_reference IN ITEMS
    "${OPEN_LMM_REPOSITORY_ROOT}/open_lmm/src"
    "${OPEN_LMM_CORE_BUILD_DIR}")
  string(FIND "${ldd_stdout}" "${forbidden_reference}" found)
  if(NOT found EQUAL -1)
    message(FATAL_ERROR
      "installed CLI dependency resolution retained private path: "
      "${forbidden_reference}")
  endif()
endforeach()

foreach(case_name IN ITEMS
    help invalid_noarg invalid_many bootstrap_failure run_failure success)
  execute_process(
    COMMAND "${CMAKE_COMMAND}"
      "-DOPEN_LMM_BATCH_EXECUTABLE=${cli_batch}"
      "-DOPEN_LMM_CLI_CASE=${case_name}"
      "-DOPEN_LMM_CLI_TEST_ROOT=${cli_fixture}/${case_name}"
      "-DOPEN_LMM_CLI_FIXTURE_SOURCE=${cli_source}/test/fixture"
      -P "${cli_source}/test/cli_contract_tests.cmake"
    RESULT_VARIABLE contract_result
    OUTPUT_VARIABLE contract_stdout
    ERROR_VARIABLE contract_stderr)
  if(NOT contract_result EQUAL 0)
    message(FATAL_ERROR
      "installed CLI ${case_name} contract failed: ${contract_result}\n"
      "stdout:\n${contract_stdout}\nstderr:\n${contract_stderr}")
  endif()
endforeach()

execute_process(
  COMMAND "${CMAKE_COMMAND}" --install "${cli_build}"
          --prefix "${OPEN_LMM_CORE_PREFIX}" --component Tools
  RESULT_VARIABLE reinstall_result
  OUTPUT_VARIABLE reinstall_stdout
  ERROR_VARIABLE reinstall_stderr)
if(NOT reinstall_result EQUAL 0)
  message(FATAL_ERROR
    "standalone CLI reinstall failed: ${reinstall_result}\n"
    "stdout:\n${reinstall_stdout}\nstderr:\n${reinstall_stderr}")
endif()
if(NOT EXISTS "${user_file}")
  message(FATAL_ERROR "CLI reinstall removed an unknown user file")
endif()
normalize_manifest("${cli_manifest}" reinstalled_cli_owned_files)
if(NOT reinstalled_cli_owned_files STREQUAL expected_cli_owned_files)
  message(FATAL_ERROR "CLI reinstall did not converge to the same owner manifest")
endif()
file(REMOVE "${user_file}")
