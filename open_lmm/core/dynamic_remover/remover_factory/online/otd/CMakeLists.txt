# cmake_minimum_required(VERSION 3.16)

# project(hmmMOS)

message(STATUS "================  BUILDING OTD  ======================")

find_package(OpenMP REQUIRED)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# add_compile_options(-std=c++2a -O3)



set(SOURCES
    otd.cpp
    params.cpp
)

add_library(otd SHARED)
target_sources(otd PRIVATE ${SOURCES})
target_link_libraries(otd
    PCL::PCL
    OpenMP::OpenMP_CXX
)
openlmm_set_global_target_properties(otd)

add_library(create_otd SHARED)
target_sources(create_otd PRIVATE 
create_otd.cpp
)
target_link_libraries(create_otd
otd
)
#TODO(gil) : process create_otd to parent directory and use for each
openlmm_set_create_target_properties(create_otd)
