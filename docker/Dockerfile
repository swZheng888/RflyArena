##############################################################################
# UAV Multi-Controller Benchmark Docker Image
# 包含：RL (ONNX) + NMPC (acados/casadi) + Benchmark
#
# 构建 (在 trajectory_project 根目录执行):
#   docker build -t uav_benchmark:latest -f docker/Dockerfile .
#
# 导出:
#   docker save uav_benchmark:latest | gzip > uav_benchmark.tar.gz
#
# 注意:
#   本 Dockerfile 直接复制宿主机已编译好的 acados。
#   部署到不同架构 (ARM64) 时，需在目标平台重新编译 acados
#   并替换 /root/acados 目录后重新 build。
##############################################################################

FROM ros:noetic-ros-base

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# python → python3 (ROS scripts 用 #!/usr/bin/env python)
RUN ln -sf /usr/bin/python3 /usr/bin/python

# ============================================================
# 系统依赖
# ============================================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip python3-dev \
    python3-rospy python3-roslaunch \
    ros-noetic-mavros \
    ros-noetic-mavros-msgs \
    ros-noetic-nav-msgs \
    ros-noetic-geometry-msgs \
    ros-noetic-std-msgs \
    ros-noetic-tf2-ros \
    ros-noetic-tf2-geometry-msgs \
    ros-noetic-tf \
    ros-noetic-topic-tools \
    build-essential cmake \
    libblas-dev liblapack-dev \
    && rm -rf /var/lib/apt/lists/*

# ============================================================
# Python 依赖
# ============================================================
RUN pip3 install --no-cache-dir \
    "numpy>=1.21,<2.0" \
    scipy \
    onnxruntime \
    casadi \
    pyyaml \
    matplotlib \
    pandas

# ============================================================
# acados (从宿主机复制已编译版本)
# ============================================================
# 部署到不同 CPU 架构时需在目标平台重新编译 acados
COPY acados /opt/acados

ENV ACADOS_SOURCE_DIR=/opt/acados
ENV LD_LIBRARY_PATH=/opt/acados/lib

# 安装 acados Python 接口
RUN cd /opt/acados/interfaces/acados_template \
    && pip3 install --no-cache-dir -e .

# ============================================================
# catkin 工作区
# ============================================================
ENV CATKIN_WS=/catkin_ws
RUN mkdir -p ${CATKIN_WS}/src

# 复制源码
COPY src/quadrotor_msgs       ${CATKIN_WS}/src/quadrotor_msgs
COPY src/uav_utils            ${CATKIN_WS}/src/uav_utils
COPY src/cmake_utils          ${CATKIN_WS}/src/cmake_utils
COPY src/Px4Ctrl              ${CATKIN_WS}/src/Px4Ctrl
COPY src/rl_control           ${CATKIN_WS}/src/rl_control
COPY src/nmpc_control         ${CATKIN_WS}/src/nmpc_control
COPY src/benchmark_utils      ${CATKIN_WS}/src/benchmark_utils
COPY src/trajectory_publisher ${CATKIN_WS}/src/trajectory_publisher

# 编译 catkin 工作区
RUN source /opt/ros/noetic/setup.bash \
    && cd ${CATKIN_WS} \
    && catkin_make -DCMAKE_BUILD_TYPE=Release \
    && echo "source ${CATKIN_WS}/devel/setup.bash" >> /root/.bashrc

# ============================================================
# 入口点
# ============================================================
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENV CONTROLLER=rl
ENV ROS_MASTER_URI=http://localhost:11311
ENV ODOM_TOPIC=/vio/odometry_enu

ENTRYPOINT ["/entrypoint.sh"]
