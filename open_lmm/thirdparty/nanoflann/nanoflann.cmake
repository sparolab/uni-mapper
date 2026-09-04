
include(FetchContent)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "Do not build nanoflann examples")
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "Do not build nanoflann tests")
set(MASTER_PROJECT_HAS_TARGET_UNINSTALL ON CACHE BOOL "Set to ON to avoid conflicts with ROS2 ament_auto_package()")

FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG        923c2ac16a955317054c296d298425d33c0cb9de
)
FetchContent_MakeAvailable(nanoflann)
