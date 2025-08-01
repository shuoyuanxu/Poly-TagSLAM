FROM ros:noetic-ros-core-focal

ENV DEBIAN_FRONTEND=noninteractive

# Install system tools and ROS dependencies
RUN apt-get update && apt-get install -y \
    build-essential cmake git wget curl \
    libboost-all-dev \
    libeigen3-dev \
    libxmlrpcpp-dev \
    python3-pip \
    ros-noetic-roscpp \
    ros-noetic-roslib \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-visualization-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-std-msgs \
    ros-noetic-apriltag-ros \
    ros-noetic-pcl-ros \
 && rm -rf /var/lib/apt/lists/*

# Install GTSAM from source
RUN git clone https://github.com/borglab/gtsam.git /opt/gtsam && \
    cd /opt/gtsam && \
    git checkout aba056176e98b95c807fb37c107188110ec54be2 && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=/usr/local \
             -DGTSAM_USE_SYSTEM_EIGEN=ON \
             -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    make -j$(nproc) && make install

# Create and set up Catkin workspace
RUN mkdir -p /catkin_ws/src
WORKDIR /catkin_ws/src

# Clone aprilslam
RUN git clone https://github.com/shuoyuanxu/Poly-TagSLAM.git aprilslamcpp

# Clone apriltag_ros at specified commit
RUN git clone https://github.com/AprilRobotics/apriltag_ros.git && \
    cd apriltag_ros && \
    git checkout 94b3b843324f6b64edc6ca59f9729e9021308406

# Build the workspace
WORKDIR /catkin_ws
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Source workspace automatically
RUN echo "source /catkin_ws/devel/setup.bash" >> /root/.bashrc

# Set runtime environment
ENV LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH

# Start with a bash shell
ENTRYPOINT ["/bin/bash"]

