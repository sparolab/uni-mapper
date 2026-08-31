include(CMakeParseArguments)

set(OPEN_LMM_PYTHON_TEST_MANIFEST
  "${CMAKE_CURRENT_BINARY_DIR}/open_lmm_python_test_manifest.tsv")
file(WRITE "${OPEN_LMM_PYTHON_TEST_MANIFEST}"
  "name\tlayer\tmodule\towner\tinvariants\tlanes\n")

function(openlmm_add_python_test)
  set(options WHEEL)
  set(one_value NAME FILE LAYER MODULE OWNER)
  set(multi_value INVARIANTS)
  cmake_parse_arguments(TEST "${options}" "${one_value}" "${multi_value}"
    ${ARGN})
  foreach(required IN ITEMS NAME FILE LAYER MODULE OWNER INVARIANTS)
    if(NOT TEST_${required})
      message(FATAL_ERROR "Python test registration requires ${required}")
    endif()
  endforeach()

  set(labels
    "layer:${TEST_LAYER}"
    "module:${TEST_MODULE}"
    "owner:${TEST_OWNER}"
    "lane:pr")
  foreach(invariant IN LISTS TEST_INVARIANTS)
    list(APPEND labels "invariant:${invariant}")
  endforeach()

  if(TEST_WHEEL)
    add_test(NAME ${TEST_NAME}
      COMMAND ${CMAKE_COMMAND} -E env
        --unset=LD_LIBRARY_PATH
        --unset=PYTHONPATH
        --unset=ROS_VERSION
        --unset=AMENT_PREFIX_PATH
        --unset=COLCON_PREFIX_PATH
        ${OPEN_LMM_PYTHON_WHEEL_TEST_PYTHON}
        -I "${CMAKE_CURRENT_SOURCE_DIR}/${TEST_FILE}")
    set_tests_properties(${TEST_NAME} PROPERTIES
      WORKING_DIRECTORY /tmp
      TIMEOUT 180
      LABELS "${labels}")
  else()
    add_test(NAME ${TEST_NAME}
      COMMAND ${OPEN_LMM_PYTHON_EXECUTABLE}
        "${CMAKE_CURRENT_SOURCE_DIR}/${TEST_FILE}")
    set(source_test_environment
        "PYTHONPATH=${OPEN_LMM_PYTHON_STAGE_DIR}"
        "OPEN_LMM_PYTHON_TEST_ROOT=${PROJECT_BINARY_DIR}/python-test"
        "OPEN_LMM_BINDING_SOURCE_ROOT=${PROJECT_SOURCE_DIR}"
        "OPEN_LMM_REPOSITORY_ROOT=${OPEN_LMM_REPOSITORY_ROOT}"
        "OPEN_LMM_CORE_TEST_BUILD_ROOT=${OPEN_LMM_CORE_TEST_BUILD_ROOT}"
        "OPEN_LMM_REPLAY_COMPARE=${OPEN_LMM_REPLAY_COMPARE}")
    if(OPEN_LMM_PYTHON_TEST_LD_PRELOAD)
      list(APPEND source_test_environment
        "LD_PRELOAD=${OPEN_LMM_PYTHON_TEST_LD_PRELOAD}")
    endif()
    if(OPEN_LMM_PYTHON_TEST_LIBRARY_PATH)
      list(APPEND source_test_environment
        "LD_LIBRARY_PATH=${OPEN_LMM_PYTHON_TEST_LIBRARY_PATH}")
    endif()
    set_tests_properties(${TEST_NAME} PROPERTIES
      ENVIRONMENT "${source_test_environment}"
      TIMEOUT 180
      LABELS "${labels}")
  endif()

  string(REPLACE ";" "," invariant_field "${TEST_INVARIANTS}")
  file(APPEND "${OPEN_LMM_PYTHON_TEST_MANIFEST}"
    "${TEST_NAME}\t${TEST_LAYER}\t${TEST_MODULE}\t${TEST_OWNER}\t"
    "${invariant_field}\tpr\n")
endfunction()
