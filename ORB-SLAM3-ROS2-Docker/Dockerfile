# Use the official ROS Humble base (Multi-platform: supports amd64 and arm64/v8)
FROM ros:humble-ros-base-jammy

ARG USE_CI
ARG DEBIAN_FRONTEND=noninteractive

# 1. Install system dependencies and ROS Desktop Full components
RUN apt-get update && apt-get install -y \
    gnupg2 \
    curl \
    lsb-release \
    vim \
    wget \
    python3-pip \
    # Desktop-full components manually added for ARM compatibility
    ros-humble-desktop-full \
    # Base tools
    cmake \
    build-essential \
    git \
    unzip \
    pkg-config \
    python3-dev \
    python3-numpy \
    # Pangolin & GUI dependencies
    libgl1-mesa-dev \
    libglew-dev \
    libeigen3-dev \
    libpng-dev \
    libjpeg-dev \
    libtiff-dev \
    libavcodec-dev \
    libavformat-dev \
    libswscale-dev \
    libgstreamer-plugins-base1.0-dev \
    libgstreamer1.0-dev \
    libgtk-3-dev \
    apt-transport-https \
    ca-certificates \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# 2. Build OpenCV (4.4.0 as requested)
RUN cd /tmp && git clone https://github.com/opencv/opencv.git && \
    cd opencv && \
    git checkout 4.4.0 && mkdir build && cd build && \
    cmake -D CMAKE_BUILD_TYPE=Release \
          -D BUILD_EXAMPLES=OFF \
          -D BUILD_DOCS=OFF \
          -D BUILD_PERF_TESTS=OFF \
          -D BUILD_TESTS=OFF \
          -D CMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/opencv

# 3. Build Pangolin (v0.9.1 as requested)
RUN cd /tmp && git clone https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && git checkout v0.9.1 && mkdir build && cd build && \
    cmake -DCMAKE_BUILD_TYPE=Release \
          -DCMAKE_CXX_FLAGS=-std=c++14 \
          -DCMAKE_INSTALL_PREFIX=/usr/local .. && \
    make -j$(nproc) && make install && \
    cd / && rm -rf /tmp/Pangolin

# Build vscode (can be removed later for deployment)
COPY ./container_root/shell_scripts/vscode_install.sh /root/
RUN cd /root/ && sudo chmod +x * && ./vscode_install.sh && rm -rf vscode_install.sh

RUN apt-get update && apt-get install ros-humble-pcl-ros tmux -y
RUN apt-get install ros-humble-nav2-common x11-apps nano -y
RUN apt-get install -y gdb gdbserver ros-humble-rmw-cyclonedds-cpp ros-humble-cv-bridge ros-humble-image-transport ros-humble-image-common ros-humble-vision-opencv

COPY ORB_SLAM3 /home/orb/ORB_SLAM3
COPY orb_slam3_ros2_wrapper /root/colcon_ws/src/orb_slam3_ros2_wrapper
COPY orb_slam3_map_generator /root/colcon_ws/src/orb_slam3_map_generator
COPY slam_msgs /root/colcon_ws/src/slam_msgs

# Build ORB-SLAM3 with its dependencies.
RUN if [ "$USE_CI" = "true" ]; then \
    . /opt/ros/humble/setup.sh && cd /home/orb/ORB_SLAM3 && mkdir -p build && ./build.sh && \
    . /opt/ros/humble/setup.sh && cd /root/colcon_ws/ && colcon build --symlink-install; \
    fi

RUN rm -rf /home/orb/ORB_SLAM3 /root/colcon_ws
