# cmake_minimum_required(VERSION 3.16)

# project(hmmMOS)

message(STATUS "================  BUILDING HMM_MOS  ======================")

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# add_compile_options(-std=c++2a -O3)


include(FetchContent)
FetchContent_Declare(
    unordered_dense
    GIT_REPOSITORY https://github.com/martinus/unordered_dense.git
    GIT_TAG        v4.5.0
)
FetchContent_MakeAvailable(unordered_dense)


set(SOURCES
    hmm_mos.cpp
    Map.cpp
    Scan.cpp
    utils.cpp
    params.cpp
)

add_library(hmm_mos SHARED)
target_sources(hmm_mos PRIVATE ${SOURCES})
target_link_libraries(hmm_mos
    TBB::tbb
    unordered_dense::unordered_dense
    PCL::PCL
    nanoflann::nanoflann
)
openlmm_set_global_target_properties(hmm_mos)

add_library(create_hmm_mos SHARED)
target_sources(create_hmm_mos PRIVATE 
create_hmm_mos.cpp
)
target_link_libraries(create_hmm_mos
hmm_mos
)
#TODO(gil) : process create_hmm_mos to parent directory and use for each
openlmm_set_create_target_properties(create_hmm_mos)
