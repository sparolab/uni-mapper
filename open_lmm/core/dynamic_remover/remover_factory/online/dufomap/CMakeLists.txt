message(STATUS "================  BUILDING DUFOMAP  ======================")

#! sudo apt install liblzf-dev
include(FetchContent)
FetchContent_Declare(
    ufomap
    GIT_REPOSITORY https://github.com/UnknownFreeOccupied/ufomap.git
    GIT_TAG        fcb20f445d257280c039c07ab756c9d5b37ebd78
)
FetchContent_MakeAvailable(ufomap)

add_library(dufomap SHARED)
target_sources(dufomap PRIVATE
  dufomap.cpp
  params.cpp
)
target_link_libraries(dufomap PUBLIC
  UFO::Map
  Eigen3::Eigen
  PCL::PCL
)
openlmm_set_global_target_properties(dufomap)

add_library(create_dufomap SHARED)
target_sources(create_dufomap PRIVATE 
create_dufomap.cpp
)
target_link_libraries(create_dufomap
dufomap
)
openlmm_set_create_target_properties(create_dufomap)