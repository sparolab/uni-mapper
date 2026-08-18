foreach(required IN ITEMS OPEN_LMM_SOURCE_ROOT OPEN_LMM_C_COMPILER_GNU
                          OPEN_LMM_CXX_COMPILER_GNU OPEN_LMM_C_COMPILER_CLANG
                          OPEN_LMM_CXX_COMPILER_CLANG OPEN_LMM_CROSS_TEST_DIR)
  if(NOT DEFINED ${required})
    message(FATAL_ERROR "${required} is required")
  endif()
endforeach()

file(REMOVE_RECURSE "${OPEN_LMM_CROSS_TEST_DIR}")
file(MAKE_DIRECTORY "${OPEN_LMM_CROSS_TEST_DIR}")
set(fixture "${OPEN_LMM_SOURCE_ROOT}/open_lmm/test/plugin_fixture_v2.c")
set(host_sources
  "${OPEN_LMM_SOURCE_ROOT}/open_lmm/common/agent_id.cpp"
  "${OPEN_LMM_SOURCE_ROOT}/open_lmm/common/runtime_contracts.cpp"
  "${OPEN_LMM_SOURCE_ROOT}/open_lmm/common/plugin_host_v2.cpp"
  "${OPEN_LMM_SOURCE_ROOT}/open_lmm/test/plugin_cross_host.cpp")

function(build_and_run label plugin_compiler host_compiler)
  set(plugin "${OPEN_LMM_CROSS_TEST_DIR}/${label}-plugin.so")
  set(host "${OPEN_LMM_CROSS_TEST_DIR}/${label}-host")
  execute_process(
    COMMAND "${plugin_compiler}" -std=c11 -fPIC -shared
            "-I${OPEN_LMM_SOURCE_ROOT}" "${fixture}" -o "${plugin}"
    RESULT_VARIABLE plugin_result)
  if(NOT plugin_result EQUAL 0)
    message(FATAL_ERROR "${label} plugin compilation failed")
  endif()
  execute_process(
    COMMAND "${host_compiler}" -std=c++20 "-I${OPEN_LMM_SOURCE_ROOT}"
            ${host_sources} -ldl -o "${host}"
    RESULT_VARIABLE host_result)
  if(NOT host_result EQUAL 0)
    message(FATAL_ERROR "${label} host compilation failed")
  endif()
  execute_process(COMMAND "${host}" "${plugin}" RESULT_VARIABLE run_result)
  if(NOT run_result EQUAL 0)
    message(FATAL_ERROR "${label} load/call/cancel/destroy failed: ${run_result}")
  endif()
endfunction()

build_and_run(gcc-host-clang-plugin "${OPEN_LMM_C_COMPILER_CLANG}"
              "${OPEN_LMM_CXX_COMPILER_GNU}")
build_and_run(clang-host-gcc-plugin "${OPEN_LMM_C_COMPILER_GNU}"
              "${OPEN_LMM_CXX_COMPILER_CLANG}")
