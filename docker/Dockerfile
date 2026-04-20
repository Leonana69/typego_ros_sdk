# This is an auto generated Dockerfile for ros:ros-base
# generated from docker_images_ros2/create_ros_image.Dockerfile.em
FROM ros:humble-ros-core-jammy

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# bootstrap rosdep
RUN rosdep init && \
  rosdep update --rosdistro $ROS_DISTRO

# setup colcon mixin and metadata
RUN colcon mixin add default \
      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
    colcon mixin update && \
    colcon metadata add default \
      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
    colcon metadata update

# install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-base=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# install gstreamer
RUN apt-get update && apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-libav

# install slam_toolbox
RUN apt-get update && apt-get install -y ros-humble-slam-toolbox

# install nav2
RUN apt-get update && apt-get install -y ros-humble-navigation2 ros-humble-nav2-bringup

# install cyclonedds
RUN apt-get update && apt-get install -y ros-humble-rmw-cyclonedds-cpp ros-humble-cyclonedds
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV CYCLONEDDS_URI=/workspace/src/typego_sdk/config/cyclonedds_shm.xml

# install qt5
RUN apt-get update && apt-get install -y qtdeclarative5-dev libpcl-dev ros-humble-rviz2 ros-humble-pcl-conversions ros-humble-pcl-ros

# web gateway: rosbag2 + mcap storage + Python deps + Node.js (for frontend build)
RUN apt-get update && apt-get install -y \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-storage-mcap \
    python3-pip \
    curl \
    ca-certificates \
    && curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get install -y nodejs \
    && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir 'fastapi==0.115.*' 'uvicorn[standard]==0.32.*' \
    'pillow>=10' 'numpy>=1.24' 'aiofiles>=23' \
    'pydantic>=2,<3' 'PyYAML>=6'

# bag storage directory for the always-on rotator
RUN mkdir -p /data/bags

# source the setup files
RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /root/.bashrc

COPY ../src /workspace/src
COPY ../Makefile /workspace/Makefile
WORKDIR /workspace

ARG AUTONOMY_TYPE
ENV AUTONOMY_TYPE=${AUTONOMY_TYPE}

# Build SLAM dependencies when AUTONOMY_TYPE=full
RUN if [ "$AUTONOMY_TYPE" = "full" ]; then \
    cd /workspace/src/autonomy/slam/dependency/Sophus && \
    rm -rf build && mkdir build && cd build && \
    cmake .. -DBUILD_TESTS=OFF && \
    make && make install && \
    cd /workspace/src/autonomy/slam/dependency/gtsam && \
    rm -rf build && mkdir build && cd build && \
    cmake .. -DGTSAM_USE_SYSTEM_EIGEN=ON -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF && \
    make -j6 && make install && \
    /sbin/ldconfig -v; \
fi
RUN if [ "$AUTONOMY_TYPE" = "full" ]; then \
    apt-get update && apt-get install -y ros-humble-joy; \
fi

# build the web gateway frontend before colcon so ament_python packages
# install the compiled assets into share/typego_web_gateway/frontend/.
# Use `npm install` (not `npm ci`) because no lockfile is shipped — the
# frontend depends on a small, stable set of packages pinned in package.json.
RUN cd /workspace/src/typego_web_gateway/frontend \
    && npm install --no-audit --no-fund \
    && npm run build

RUN /bin/bash -c "source /opt/ros/$ROS_DISTRO/setup.bash && cd /workspace && make build"
RUN chmod +x /workspace/src/typego_sdk/scripts/*.sh

EXPOSE 8080
CMD ["bash", "-c", "source /workspace/install/setup.bash && /workspace/src/typego_sdk/scripts/launch.sh"]