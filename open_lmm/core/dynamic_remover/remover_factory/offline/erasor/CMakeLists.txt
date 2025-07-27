# cmake_minimum_required(VERSION 3.14)
# project(erasor_dynamic_map
# 	VERSION 1.0.0
# 	DESCRIPTION "ERASOR DynamicMap Benchmark"
# 	LANGUAGES C CXX
# )


# specify the C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED True)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# see here: https://github.com/PointCloudLibrary/pcl/issues/3680#issuecomment-700642602
if(NOT DEFINED CMAKE_SUPPRESS_DEVELOPER_WARNINGS)
     set(CMAKE_SUPPRESS_DEVELOPER_WARNINGS 1 CACHE INTERNAL "No dev warnings")
endif()

#############
# PACKAGES  #
#############

find_package(Eigen3 REQUIRED)
# find_package(PCL REQUIRED)

#############
# LIBRARIES #
#############
add_library(erasor SHARED erasor.cpp 
    erasor_core.cpp 
    utils.cpp
)
target_link_libraries(erasor 
    Eigen3::Eigen 
    PCL::PCL 
    small_gicp
)
openlmm_set_global_target_properties(erasor)

add_library(create_erasor SHARED 
    create_erasor.cpp
)
target_link_libraries(create_erasor erasor)
openlmm_set_create_target_properties(create_erasor)