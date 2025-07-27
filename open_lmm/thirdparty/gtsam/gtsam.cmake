
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF CACHE BOOL "Don't build GTSAM examples")
set(GTSAM_BUILD_TESTS OFF CACHE BOOL "Don't build GTSAM tests")
set(GTSAM_WITH_TBB OFF CACHE BOOL "Don't build GTSAM with TBB")
set(GTSAM_BUILD_WITH_MARCH_NATIVE OFF CACHE BOOL "Don't build GTSAM with MARCH_NATIVE")
set(GTSAM_USE_SYSTEM_EIGEN OFF CACHE BOOL "Don't build GTSAM with SYSTEM_EIGEN3")

include(FetchContent)
FetchContent_Declare(
    gtsam
    GIT_REPOSITORY https://github.com/borglab/gtsam.git
    GIT_TAG        4.2
)
FetchContent_MakeAvailable(gtsam)
# FetchContent_GetProperties(gtsam)
# if(NOT gtsam_POPULATED)
#   FetchContent_Populate(gtsam)
#   if(${CMAKE_VERSION} GREATER_EQUAL 3.25)
#     add_subdirectory(${gtsam_SOURCE_DIR} ${gtsam_BINARY_DIR} SYSTEM EXCLUDE_FROM_ALL)
#   else()
#     # Emulate the SYSTEM flag introduced in CMake 3.25. Withouth this flag the compiler will
#     # consider this 3rdparty headers as source code and fail due the -Werror flag.
#     add_subdirectory(${gtsam_SOURCE_DIR} ${gtsam_BINARY_DIR} EXCLUDE_FROM_ALL)
#     get_target_property(gtsam_include_dirs gtsam INTERFACE_INCLUDE_DIRECTORIES)
#     set_target_properties(gtsam PROPERTIES INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${gtsam_include_dirs}")
#   endif()
# endif()