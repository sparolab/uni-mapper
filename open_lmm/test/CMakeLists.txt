include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/OpenLmmTest.cmake)
add_subdirectory(architecture/module_compile)

# Keep this file as a dispatcher. Owner-aligned registrations live beside the
# manifest helper so ownership changes remain reviewable in isolation.
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/domain.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/foundation_tools.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/runtime.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/visualization.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/config_plugins.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/e2e.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/soak.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/benchmark.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/quality.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/registrations/policy_package.cmake)
