# 基于 dustynv 的 ROS 2 Humble 镜像（适用于 Jetson）
FROM dustynv/ros:humble-desktop-l4t-r35.3.1

# 维护者信息（可选）
LABEL maintainer="243040459@hdu.edu.cn"

# 设置工作空间路径
ENV ROS_WS /root/ros2_ws

# 创建工作空间目录
RUN mkdir -p $ROS_WS/src

# Step 2: 更新包列表并安装必要工具（此时不能有 ros2 源！）
RUN rm -f /etc/apt/sources.list.d/ros2.list
RUN apt-get update && \
    apt-get install -y curl gnupg2 lsb-release && \
    rm -rf /var/lib/apt/lists/*
# Step 3: 下载并安装新的 ROS 2 GPG 密钥
COPY ros.key /usr/share/keyrings/ros-archive-keyring.gpg
# Step 4: 添加 ROS 2 官方源（使用 signed-by）
RUN echo "deb [arch=arm64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list
# Step 5: 现在可以安全地更新并安装 ROS 2 软件包
RUN apt-get update

# 安装依赖（用于 GPS 驱动）
RUN python3 -m pip install --upgrade pip
RUN pip3 install --timeout 500 transforms3d numpy
RUN pip3 install --timeout 500 geographiclib numpy
RUN pip install pyserial
RUN sudo apt install -y libgeographic-dev libgeos-dev libproj-dev
RUN sudo apt install -y libgraphicsmagick++-dev libpcap-dev
RUN sudo apt install -y libgeographic-dev geographiclib-tools
RUN apt-get install -y python3-can can-utils iproute2 x11-apps vim

# 安装 rviz_imu_plugin
WORKDIR $ROS_WS/src
RUN git clone -b humble https://github.com/CCNYRoboticsLab/imu_tools.git
#COPY wit_ros2_imu wit_ros2_imu

WORKDIR $ROS_WS
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"

# 克隆GPS驱动源码
WORKDIR $ROS_WS/src
RUN git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git 
RUN git clone https://github.com/DLu/tf_transformations.git

WORKDIR $ROS_WS
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-select tf_transformations \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-select nmea_navsat_driver \
    --cmake-args -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"

# 克隆 navsat-transform 依赖源码
WORKDIR $ROS_WS/src
RUN git clone -b ros2-humble https://github.com/ros/diagnostics.git
RUN git clone -b ros2 https://github.com/ros-geographic-info/geographic_info.git
RUN git clone -b humble-devel https://github.com/cra-ros-pkg/robot_localization.git

WORKDIR $ROS_WS
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-select diagnostic_updater \
    --cmake-args -Wno-dev -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-select geographic_msgs \
    --cmake-args -Wno-dev -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-select geodesy \
    --cmake-args -Wno-dev -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-select robot_localization \
    --cmake-args -Wno-dev -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"
 
#NAV2依赖
WORKDIR $ROS_WS/src
RUN git clone -b ros2 https://github.com/ros/bond_core.git
RUN git clone -b humble https://github.com/ros-navigation/navigation2.git

WORKDIR $ROS_WS
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-up-to bondcpp \
    --cmake-args -Wno-dev -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"
RUN /bin/bash -c "source /opt/ros/${ROS_DISTRO}/install/setup.bash && \
    colcon build --packages-up-to nav2_costmap_2d \
    --cmake-args -Wno-dev -DBUILD_TESTING=OFF -DCMAKE_PREFIX_PATH=/usr/local \
    --parallel-workers $(nproc)"

# 自动 source 安装环境（每次启动容器时生效）
RUN echo "source $ROS_WS/install/setup.bash" >> ~/.bashrc

# 启动命令（可选）：默认启动 bash
CMD ["/bin/bash"]
