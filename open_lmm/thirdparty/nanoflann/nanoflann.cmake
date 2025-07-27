
include(FetchContent)
set(NANOFLANN_BUILD_EXAMPLES OFF CACHE BOOL "Do not build nanoflann examples")
set(NANOFLANN_BUILD_TESTS OFF CACHE BOOL "Do not build nanoflann tests")
set(MASTER_PROJECT_HAS_TARGET_UNINSTALL ON CACHE BOOL "Set to ON to avoid conflicts with ROS2 ament_auto_package()")

FetchContent_Declare(
  nanoflann
  GIT_REPOSITORY https://github.com/jlblancoc/nanoflann.git
  GIT_TAG        v1.5.5
)
FetchContent_MakeAvailable(nanoflann)