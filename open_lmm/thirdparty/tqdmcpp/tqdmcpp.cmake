include(FetchContent)
FetchContent_Declare(
  tqdmcpp
  GIT_REPOSITORY https://github.com/hwan0806/tqdm-cpp-custom.git
  GIT_TAG master
)
FetchContent_MakeAvailable(tqdmcpp)