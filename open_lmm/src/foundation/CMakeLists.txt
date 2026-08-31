find_package(spdlog REQUIRED)

add_subdirectory(contracts)

add_library(open_lmm_profiling INTERFACE)
target_compile_definitions(open_lmm_profiling INTERFACE
  OPEN_LMM_ENABLE_TIMING_LOG=$<BOOL:${OPEN_LMM_ENABLE_TIMING_LOG}>)
if(OPEN_LMM_ENABLE_TRACY)
  target_compile_definitions(open_lmm_profiling INTERFACE
    OPEN_LMM_ENABLE_TRACY=1
    TRACY_ENABLE
    TRACY_ON_DEMAND)
  target_link_libraries(open_lmm_profiling INTERFACE Tracy::TracyClient)
else()
  target_compile_definitions(open_lmm_profiling INTERFACE
    OPEN_LMM_ENABLE_TRACY=0)
endif()

add_library(open_lmm_foundation_concurrency_objects OBJECT
  concurrency/bounded_executor.cpp)
openlmm_set_global_target_properties(
  open_lmm_foundation_concurrency_objects)

add_library(open_lmm_foundation_logging_objects OBJECT
  logging/logging.cpp)
target_link_libraries(open_lmm_foundation_logging_objects PRIVATE
  spdlog::spdlog)
openlmm_set_global_target_properties(open_lmm_foundation_logging_objects)
