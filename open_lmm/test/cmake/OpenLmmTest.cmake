include_guard(GLOBAL)

set(OPEN_LMM_TEST_MANIFEST "${CMAKE_CURRENT_BINARY_DIR}/open_lmm_test_manifest.tsv")
file(WRITE "${OPEN_LMM_TEST_MANIFEST}"
  "name\ttarget\tlayer\tmodule\towner\tinvariants\tlanes\tsanitizers\n")

function(_openlmm_require_single_value argument value)
  if("${value}" STREQUAL "")
    message(FATAL_ERROR "openlmm_add_test requires ${argument}")
  endif()
  if("${value}" MATCHES ";")
    message(FATAL_ERROR
      "openlmm_add_test ${argument} must contain exactly one value: ${value}")
  endif()
endfunction()

function(openlmm_add_test)
  set(options RUN_SERIAL DISABLED ALLOW_AGGREGATE_FACADE)
  set(one_value_args
    NAME TARGET COMMAND LAYER MODULE OWNER TIMEOUT WORKING_DIRECTORY)
  set(multi_value_args COMMAND_ARGS INVARIANTS LANES SANITIZERS)
  cmake_parse_arguments(OPENLMM_TEST
    "${options}" "${one_value_args}" "${multi_value_args}" ${ARGN})

  if(OPENLMM_TEST_UNPARSED_ARGUMENTS)
    message(FATAL_ERROR
      "openlmm_add_test received unknown arguments: "
      "${OPENLMM_TEST_UNPARSED_ARGUMENTS}")
  endif()

  foreach(required IN ITEMS NAME LAYER MODULE OWNER)
    _openlmm_require_single_value(
      "${required}" "${OPENLMM_TEST_${required}}")
  endforeach()
  if(NOT OPENLMM_TEST_LAYER MATCHES "^L[1-6]$")
    message(FATAL_ERROR
      "unknown test layer '${OPENLMM_TEST_LAYER}' for ${OPENLMM_TEST_NAME}")
  endif()
  if(OPENLMM_TEST_TARGET AND OPENLMM_TEST_COMMAND)
    message(FATAL_ERROR
      "${OPENLMM_TEST_NAME} must provide TARGET or COMMAND, not both")
  endif()
  if(NOT OPENLMM_TEST_TARGET AND NOT OPENLMM_TEST_COMMAND)
    message(FATAL_ERROR
      "${OPENLMM_TEST_NAME} must provide TARGET or COMMAND")
  endif()
  if(OPENLMM_TEST_TARGET AND NOT TARGET "${OPENLMM_TEST_TARGET}")
    message(FATAL_ERROR "test target does not exist: ${OPENLMM_TEST_TARGET}")
  endif()
  if(NOT OPENLMM_TEST_LANES)
    message(FATAL_ERROR "openlmm_add_test requires at least one LANE")
  endif()
  foreach(lane IN LISTS OPENLMM_TEST_LANES)
    if(NOT lane MATCHES "^(pr|nightly|external|gpu)$")
      message(FATAL_ERROR
        "unknown test lane '${lane}' for ${OPENLMM_TEST_NAME}")
    endif()
  endforeach()
  foreach(sanitizer IN LISTS OPENLMM_TEST_SANITIZERS)
    if(NOT sanitizer MATCHES "^(asan-ubsan|tsan)$")
      message(FATAL_ERROR
        "unknown sanitizer '${sanitizer}' for ${OPENLMM_TEST_NAME}")
    endif()
  endforeach()

  get_property(registered GLOBAL PROPERTY OPEN_LMM_REGISTERED_TESTS)
  if(OPENLMM_TEST_NAME IN_LIST registered)
    message(FATAL_ERROR "duplicate CTest name: ${OPENLMM_TEST_NAME}")
  endif()

  if(OPENLMM_TEST_TARGET)
    get_target_property(test_sources "${OPENLMM_TEST_TARGET}" SOURCES)
    foreach(source IN LISTS test_sources)
      if(IS_ABSOLUTE "${source}")
        file(RELATIVE_PATH relative_source "${PROJECT_SOURCE_DIR}" "${source}")
      else()
        set(relative_source "test/${source}")
      endif()
      if(relative_source MATCHES "^src/.+\\.(c|cc|cpp|cxx)$")
        message(FATAL_ERROR
          "test target ${OPENLMM_TEST_TARGET} compiles production source: ${source}")
      endif()
    endforeach()

    get_target_property(link_targets "${OPENLMM_TEST_TARGET}" LINK_LIBRARIES)
    if(OPENLMM_TEST_LAYER MATCHES "^L[12]$" AND
       "open_lmm_map_server" IN_LIST link_targets AND
       NOT OPENLMM_TEST_ALLOW_AGGREGATE_FACADE)
      message(FATAL_ERROR
        "${OPENLMM_TEST_NAME} is ${OPENLMM_TEST_LAYER} but links the aggregate "
        "open_lmm_map_server; use its direct owner target")
    endif()
    set(test_command "${OPENLMM_TEST_TARGET}")
    set(manifest_target "${OPENLMM_TEST_TARGET}")
  else()
    set(test_command "${OPENLMM_TEST_COMMAND}")
    set(manifest_target "command:${OPENLMM_TEST_NAME}")
  endif()

  add_test(NAME "${OPENLMM_TEST_NAME}"
    COMMAND "${test_command}" ${OPENLMM_TEST_COMMAND_ARGS})

  set(labels
    "layer:${OPENLMM_TEST_LAYER}"
    "module:${OPENLMM_TEST_MODULE}"
    "owner:${OPENLMM_TEST_OWNER}")
  foreach(invariant IN LISTS OPENLMM_TEST_INVARIANTS)
    list(APPEND labels "invariant:${invariant}")
  endforeach()
  foreach(lane IN LISTS OPENLMM_TEST_LANES)
    list(APPEND labels "lane:${lane}")
  endforeach()
  foreach(sanitizer IN LISTS OPENLMM_TEST_SANITIZERS)
    list(APPEND labels "sanitizer:${sanitizer}")
  endforeach()

  if(NOT OPENLMM_TEST_TIMEOUT)
    set(OPENLMM_TEST_TIMEOUT 60)
  endif()
  if(NOT OPENLMM_TEST_WORKING_DIRECTORY)
    set(OPENLMM_TEST_WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
  endif()
  set_tests_properties("${OPENLMM_TEST_NAME}" PROPERTIES
    LABELS "${labels}"
    TIMEOUT "${OPENLMM_TEST_TIMEOUT}"
    WORKING_DIRECTORY "${OPENLMM_TEST_WORKING_DIRECTORY}")
  if(OPENLMM_TEST_RUN_SERIAL)
    set_tests_properties("${OPENLMM_TEST_NAME}" PROPERTIES RUN_SERIAL TRUE)
  endif()
  if(OPENLMM_TEST_DISABLED)
    set_tests_properties("${OPENLMM_TEST_NAME}" PROPERTIES DISABLED TRUE)
  endif()

  string(JOIN "," manifest_invariants ${OPENLMM_TEST_INVARIANTS})
  string(JOIN "," manifest_lanes ${OPENLMM_TEST_LANES})
  string(JOIN "," manifest_sanitizers ${OPENLMM_TEST_SANITIZERS})
  file(APPEND "${OPEN_LMM_TEST_MANIFEST}"
    "${OPENLMM_TEST_NAME}\t${manifest_target}\t${OPENLMM_TEST_LAYER}\t"
    "${OPENLMM_TEST_MODULE}\t${OPENLMM_TEST_OWNER}\t"
    "${manifest_invariants}\t${manifest_lanes}\t${manifest_sanitizers}\n")

  set_property(GLOBAL APPEND PROPERTY OPEN_LMM_REGISTERED_TESTS
    "${OPENLMM_TEST_NAME}")
  set_property(GLOBAL APPEND PROPERTY OPEN_LMM_TEST_TARGETS
    "${manifest_target}")
endfunction()

function(openlmm_apply_test_environment environment_value)
  get_property(registered GLOBAL PROPERTY OPEN_LMM_REGISTERED_TESTS)
  foreach(test_name IN LISTS registered)
    set_tests_properties("${test_name}" PROPERTIES
      ENVIRONMENT "${environment_value}")
  endforeach()
endfunction()

function(openlmm_finalize_test_manifest)
  get_property(registered GLOBAL PROPERTY OPEN_LMM_REGISTERED_TESTS)
  get_property(directory_tests DIRECTORY PROPERTY TESTS)
  list(SORT registered)
  list(SORT directory_tests)
  if(NOT registered STREQUAL directory_tests)
    message(FATAL_ERROR
      "all CTests must be registered through openlmm_add_test\n"
      "manifest: ${registered}\nCTest: ${directory_tests}")
  endif()
endfunction()
