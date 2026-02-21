FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

# Fix for Hash Sum mismatch error
# from this link https://stackoverflow.com/questions/67732260/how-to-fix-hash-sum-mismatch-in-docker-on-mac
RUN echo "Acquire::http::Pipeline-Depth 0;" > /etc/apt/apt.conf.d/99custom && \
    echo "Acquire::http::No-Cache true;" >> /etc/apt/apt.conf.d/99custom && \
    echo "Acquire::BrokenProxy    true;" >> /etc/apt/apt.conf.d/99custom

RUN rm -rf /var/lib/apt/lists/*
RUN apt clean\
    && apt update \
    && apt upgrade -y \
    && apt install -y \
    build-essential \
    clang cmake \
    g++ gcc gdb git \
    fuse \
    libomp-dev libboost-all-dev libeigen3-dev libyaml-cpp-dev \
    python3-pip \
    curl gnupg lsb-release \
    ros-humble-robot-localization \
    && apt clean

WORKDIR /

# libnabo
RUN cd /opt \
    && git clone -b 1.1.2 https://github.com/norlab-ulaval/libnabo.git \
    && cd libnabo/ \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DUSE_OPEN_MP=ON .. \
    && make -j 12 \
    && make install

# libpointmatcher
RUN cd /opt \
    && git clone https://github.com/norlab-ulaval/libpointmatcher.git \
    && cd libpointmatcher/ \
    && mkdir build && cd build \
    && cmake -DCMAKE_BUILD_TYPE=Release -DUSE_OPEN_MP=ON .. \
    && make -j 12 \
    && make install

# norlab_icp_mapper
RUN cd /opt \
    && git clone https://github.com/norlab-ulaval/norlab_icp_mapper.git \
    && cd norlab_icp_mapper/ \
    && mkdir build_docker && cd build_docker/ \
    && cmake -DCMAKE_BUILD_TYPE=Release -DUSE_OPEN_MP=ON .. \
    && make -j12 \
    && make install

WORKDIR /

# create ros workspace and other folders
RUN mkdir -p /ros2_ws/src

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN git clone https://github.com/norlab-ulaval/imu_tools.git -b fomo /ros2_ws/src/imu_tools
RUN git clone https://github.com/norlab-ulaval/norlab_imu_tools.git -b fomo /ros2_ws/src/norlab_imu_tools
RUN git clone https://github.com/norlab-ulaval/libpointmatcher_ros.git -b fomo /ros2_ws/src/libpointmatcher_ros
RUN git clone https://github.com/norlab-ulaval/norlab_icp_mapper_ros.git -b fomo /ros2_ws/src/norlab_icp_mapper_ros
RUN git clone https://github.com/norlab-ulaval/fomo-bench.git -b ros2 /tmp/fomo-bench \
    && mv /tmp/fomo-bench/ros_launchers /ros2_ws/src/ros_launchers

RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN rosdep update
RUN rosdep install --from-paths /ros2_ws/src --ignore-src -r -y
RUN cd /ros2_ws/ \
    && source /opt/ros/humble/setup.bash \
    && colcon build --symlink-install

RUN apt update && apt install -y ros-humble-rosbag2-storage-mcap

STOPSIGNAL SIGINT
# add additional commands here
CMD ["/bin/bash", "-c", "source /opt/ros/humble/setup.bash && source /ros2_ws/install/setup.bash && ros2 launch --noninteractive ros_launchers norlabIcpMapper.launch.py"]
