include(FetchContent)

# Keep the optional GUI reproducible and independent from a host-side
# Iridescence installation. This is the revision used for the GUI adapter
# implementation and baseline verification.
set(BUILD_EXAMPLES OFF CACHE BOOL "Build Iridescence examples" FORCE)
set(BUILD_PYTHON_BINDINGS OFF CACHE BOOL "Build Iridescence Python bindings" FORCE)
set(BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Build Iridescence with -march=native" FORCE)
set(BUILD_EXT_TESTS OFF CACHE BOOL "Build Iridescence extension tests" FORCE)

FetchContent_Declare(
  iridescence
  GIT_REPOSITORY https://github.com/koide3/iridescence.git
  GIT_TAG        bf065dd5b9c8ecee823e076d827af474fbc00d80
  GIT_SHALLOW    FALSE
  PATCH_COMMAND
    ${CMAKE_COMMAND}
      -DIRIDESCENCE_SOURCE_DIR=<SOURCE_DIR>
      -P ${CMAKE_CURRENT_LIST_DIR}/patch_iridescence.cmake
)
FetchContent_MakeAvailable(iridescence)

# GCC 12 can hit an internal compiler error while compiling Iridescence at
# -O3. Limit the workaround to the fetched dependency; OpenLMM remains at -O3.
target_compile_options(iridescence PRIVATE $<$<CONFIG:Release>:-O1>)

# The build-tree target is unnamespaced, while the installed package exports
# Iridescence::Iridescence. Provide the same public name in both cases.
if(NOT TARGET Iridescence::Iridescence)
  add_library(Iridescence::Iridescence ALIAS iridescence)
endif()
