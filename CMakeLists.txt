cmake_minimum_required(VERSION 3.0.2)
project(uni_mapper)

# set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_STANDARD 14)
# set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3 -pthread")
SET(CMAKE_BUILD_TYPE Release)

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

set(Boost_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
find_package(Boost REQUIRED COMPONENTS serialization iostreams)
find_package(GTSAM REQUIRED QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(Eigen3 3.3 REQUIRED)
find_package(Ceres REQUIRED)


find_package(catkin REQUIRED COMPONENTS
        cv_bridge
        geometry_msgs
        image_transport
        # livox_ros_driver
        nav_msgs
        pcl_conversions
        pcl_ros
        roscpp
        sensor_msgs
        std_msgs
        tf
        visualization_msgs
)




catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        # CATKIN_DEPENDS livox_ros_driver
        DEPENDS eigen PCL GTSAM
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
        ${EIGEN3_INCLUDE_DIR}
)

link_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_LIBRARY_DIRS}
        ${PCL_LIBRARY_DIRS}
        ${GTSAM_LIBRARY_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(main
        src/main.cpp
        src/STDesc.cpp
        src/UniMapServer.cpp
        src/IntraLoopPGO.cpp
        src/InterLoopPGO.cpp
        src/SingleMapDB.cpp
        src/utility.cpp
        src/RosParamServer.cpp
)
target_compile_options(main
        PRIVATE ${OpenMP_CXX_FLAGS}
)
target_link_libraries(main
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBS}
        ${CERES_LIBRARIES}
        gtsam
        Boost::serialization
)
