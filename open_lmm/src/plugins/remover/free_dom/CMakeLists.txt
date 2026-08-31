# cmake_minimum_required(VERSION 3.16)

# project(hmmMOS)

message(STATUS "================  BUILDING FREE_DOM  ======================")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# add_compile_options(-std=c++2a -O3)


set(SOURCES
    free_dom.cpp
    depth_image.cpp
    map.cpp
    mrmap.cpp
    params.cpp
    raycast.cpp
    scanmap.cpp
)

find_package(OpenCV REQUIRED)

add_library(free_dom SHARED)
target_sources(free_dom PRIVATE ${SOURCES})
target_link_libraries(free_dom
    PCL::PCL
    ${OpenCV_LIBS}
    open_lmm_utils
)
openlmm_set_global_target_properties(free_dom)

add_library(create_free_dom SHARED)
target_sources(create_free_dom PRIVATE 
create_free_dom.cpp
)
target_link_libraries(create_free_dom
free_dom
)
#TODO(gil) : process create_hmm_mos to parent directory and use for each
openlmm_set_create_target_properties(create_free_dom)
