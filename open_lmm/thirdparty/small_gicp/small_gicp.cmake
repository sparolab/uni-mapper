include(FetchContent)
FetchContent_Declare(
  small_gicp
  GIT_REPOSITORY https://github.com/koide3/small_gicp
  GIT_TAG master
)
FetchContent_MakeAvailable(small_gicp)