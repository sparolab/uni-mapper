#! ROS2 humbe (Ubuntu 22.04)
FROM ros:humble-perception

ENV DEBIAN_FRONTEND=noninteractive

#! build tools and dependencies
RUN apt-get update && apt-get install -y \
    # basic tools
    build-essential \
    cmake \
    wget \
    git \
    libeigen3-dev \
    libboost-all-dev \
    libtbb-dev \
    # for module build
    liblzf-dev \
    libnanoflann-dev \
    libspdlog-dev \
    libfmt-dev \
    # for docker GUI
    libglib2.0-0 \
    libgl1-mesa-glx \
    x11-apps \
    && rm -rf /var/lib/apt/lists/*

#! Python packages
# RUN pip3 install --no-cache-dir opencv-python
# python3-pip \
# python3-colcon-common-extensions \
# python3-vcstool \
# python3-rosdep \


#! install cmake (3.25.3)
RUN cd /tmp && \
    wget https://cmake.org/files/v3.25/cmake-3.25.3.tar.gz && \
    tar -xvzf cmake-3.25.3.tar.gz && \
    cd cmake-3.25.3/ && \
    ./bootstrap --prefix=/usr/local && \
    make -j$(nproc) && \
    make install && \
    rm -rf /tmp/cmake-3.25.3


#! install GTSAM (4.2)
RUN apt-get update && \
    cd /tmp && \
    git clone https://github.com/borglab/gtsam.git && \
    cd gtsam && \
    git checkout 4.2a9 && \
    mkdir build && \
    cd build && \
    cmake \
      -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF \
      -DGTSAM_USE_SYSTEM_EIGEN=ON \
      -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
      -DGTSAM_BUILD_TESTS=OFF \
      -DGTSAM_WITH_TBB=OFF \
      .. && \
    make -j$(nproc) && \
    make install && \
    cd / && \
    rm -rf /tmp/gtsam

#! bash setup
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export LD_LIBRARY_PATH=/usr/local/lib/:$LD_LIBRARY_PATH" >> ~/.bashrc
RUN echo "alias cw='cd /root/workspace'" >> ~/.bashrc
RUN echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc
RUN echo "alias sb='source /root/workspace/install/setup.bash'" >> ~/.bashrc

#! set working directory
# WORKDIR /root/workspace

#! GUI forwarding (optional)
# ENV QT_X11_NO_MITSHM=1

#! other utilities/alias (optional)
# RUN echo "alias cw='cd /root/workspace'" >> ~/.bashrc
# RUN echo "alias cb='colcon build --symlink-install'" >> ~/.bashrc
# RUN echo "alias sb='source /root/open_lmm_ws/install/setup.bash'" >> ~/.bashrc

#! ENTRYPOINT, CMD
# ENTRYPOINT ["/ros_entrypoint.sh"]
# CMD ["bash"]
