# inint from ros:noetic image
FROM ros:noetic-perception

ARG workspace_path

ENV DEBIAN_FRONTEND=noninteractive

###### basic installation for docker development ######
RUN apt-get update
RUN apt-get install -y x11-apps
RUN apt-get install -y libgl1-mesa-glx
RUN apt-get install -y libglib2.0-0
# boost install
RUN apt-get install -y libboost-all-dev
RUN apt-get install -y build-essential
RUN apt-get install -y libeigen3-dev
RUN apt-get install -y libsuitesparse-dev
RUN apt-get install -y freeglut3-dev
RUN apt-get install -y libqglviewer-dev-qt5
RUN apt-get install -y libyaml-cpp-dev
RUN apt-get install -y ros-${ROS_DISTRO}-rviz

###### for basic python package ######
RUN apt-get install -y python3-pip
RUN pip3 install opencv-python

###### for user development ######
RUN apt-get install -y nautilus
RUN apt-get install -y gedit

# Catkin Command Line Tools
RUN apt-get install -y python3-catkin-tools
# option for build system
RUN apt-get install -y ninja-build

# initial update
RUN apt install sudo
RUN sudo apt update -y
RUN sudo apt upgrade -y

# cmake install
RUN sudo apt install -y cmake

# gcc, g++ install
RUN sudo apt install -y gcc
RUN sudo apt install -y g++

# git install
RUN sudo apt install -y curl
RUN sudo apt install -y git

# ceres install
RUN sudo apt install -y libgoogle-glog-dev
RUN sudo apt install -y curl
RUN sudo apt install -y libgflags-dev
RUN sudo apt install -y libatlas-base-dev
RUN sudo apt install -y libeigen3-dev
RUN sudo apt install -y libsuitesparse-dev
RUN cd /tmp && \
    curl -O http://ceres-solver.org/ceres-solver-2.1.0.tar.gz && \
    tar zxf ceres-solver-2.1.0.tar.gz
RUN cd /tmp/ceres-solver-2.1.0 && \
    mkdir build && \
    cd build && \
    cmake .. && \
    make -j$(nproc) && \
    make install

# gtsam install
RUN sudo apt-get install -y pkg-config
RUN cd /tmp && \
    git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.1.1
RUN cd /tmp/gtsam && \
    mkdir build && \
    cd build && \
    cmake -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES=OFF .. && \
    make -j$(nproc) && \
    make install

RUN echo "export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH" >> ~/.bashrc
RUN echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
RUN echo "alias cw='cd /root/workspace'" >> ~/.bashrc
RUN echo "alias sd='source devel/setup.bash'" >> ~/.bashrc