include_guard(GLOBAL)

function(openlmm_apply_quality_options target)
  get_target_property(openlmm_quality_source_dir ${target} SOURCE_DIR)
  string(FIND "${openlmm_quality_source_dir}"
    "${PROJECT_SOURCE_DIR}/test" openlmm_test_source_position)
  set(openlmm_is_test_target FALSE)
  if(openlmm_test_source_position EQUAL 0)
    set(openlmm_is_test_target TRUE)
  endif()

  if(OPEN_LMM_ENABLE_CLANG_TIDY AND NOT openlmm_is_test_target)
    find_program(OPEN_LMM_CLANG_TIDY_EXECUTABLE
      NAMES clang-tidy-15 clang-tidy
      REQUIRED)
    execute_process(
      COMMAND "${OPEN_LMM_CLANG_TIDY_EXECUTABLE}" --version
      OUTPUT_VARIABLE openlmm_clang_tidy_version
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT openlmm_clang_tidy_version MATCHES "version 15\\.")
      message(FATAL_ERROR
        "Goal 08 requires clang-tidy 15, got: ${openlmm_clang_tidy_version}")
    endif()
    set_property(TARGET ${target} PROPERTY CXX_CLANG_TIDY
      "${OPEN_LMM_CLANG_TIDY_EXECUTABLE};--quiet")
  endif()

  if(OPEN_LMM_ENABLE_STRICT_WARNINGS)
    target_compile_options(${target} PRIVATE
      $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Wextra>
      $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Wpedantic>
      $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Werror=return-type>
      $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Werror=format>
      $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Werror=non-virtual-dtor>
      $<$<COMPILE_LANG_AND_ID:CXX,GNU,Clang,AppleClang>:-Werror=uninitialized>
      # GCC folds its optimization-sensitive maybe-uninitialized diagnostic
      # under -Wuninitialized. It is useful evidence, but not stable enough to
      # be a required error (notably for std::optional internals).
      $<$<COMPILE_LANG_AND_ID:CXX,GNU>:-Wno-error=maybe-uninitialized>)
  endif()

  if(OPEN_LMM_ENABLE_COVERAGE)
    target_compile_options(${target} PRIVATE
      $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-fprofile-instr-generate>
      $<$<COMPILE_LANG_AND_ID:CXX,Clang,AppleClang>:-fcoverage-mapping>)
    target_link_options(${target} PRIVATE
      $<$<LINK_LANG_AND_ID:CXX,Clang,AppleClang>:-fprofile-instr-generate>
      $<$<LINK_LANG_AND_ID:CXX,Clang,AppleClang>:-fcoverage-mapping>)
  endif()
endfunction()

function(openlmm_enable_fuzzing_target target)
  if(NOT OPEN_LMM_ENABLE_FUZZING)
    message(FATAL_ERROR
      "openlmm_enable_fuzzing_target requires OPEN_LMM_ENABLE_FUZZING=ON")
  endif()
  target_compile_options(${target} PRIVATE
    -fsanitize=fuzzer,address,undefined
    -fno-omit-frame-pointer)
  target_link_options(${target} PRIVATE
    -fsanitize=fuzzer,address,undefined
    -fno-omit-frame-pointer)
  if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    # Clang 15's compiler-rt predates the wide-ASLR allocator fix.  Keep fuzz
    # executables non-PIE for the same deterministic startup reason as the
    # existing ASan/UBSan test lane.
    target_compile_options(${target} PRIVATE -fno-pie)
    target_link_options(${target} PRIVATE -no-pie)
  endif()
endfunction()
