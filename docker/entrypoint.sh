#!/bin/bash
##############################################################################
# UAV Controller Docker 入口点
# Docker 只运行控制器，benchmark 在宿主机运行
#
# 环境变量:
#   CONTROLLER      - 控制器: rl / nmpc (默认 rl)
#   ROS_MASTER_URI  - ROS Master 地址
#   ODOM_TOPIC      - 里程计话题
##############################################################################
set -e

source /opt/ros/noetic/setup.bash
source /catkin_ws/devel/setup.bash

export ODOM_TOPIC=${ODOM_TOPIC:-/vio/odometry_enu}
export CONTROLLER=${CONTROLLER:-rl}

echo "============================================================"
echo "  UAV Controller Docker"
echo "============================================================"
echo "  CONTROLLER:     $CONTROLLER"
echo "  ROS_MASTER_URI: $ROS_MASTER_URI"
echo "  ODOM_TOPIC:     $ODOM_TOPIC"
echo "============================================================"

# 等待 ROS Master
echo "[entrypoint] 等待 ROS Master..."
until rostopic list > /dev/null 2>&1; do
    sleep 1
done
echo "[entrypoint] ROS Master 已连接"

case "$CONTROLLER" in
    rl)
        echo "[entrypoint] 启动 RL 控制器..."
        exec roslaunch rl_control docker_rl.launch \
            odom_topic:="$ODOM_TOPIC"
        ;;
    nmpc)
        echo "[entrypoint] 启动 NMPC 控制器..."
        exec roslaunch nmpc_control docker_nmpc.launch \
            odom_topic:="$ODOM_TOPIC"
        ;;
    *)
        echo "[entrypoint] 未知控制器: $CONTROLLER (可选: rl / nmpc)"
        exit 1
        ;;
esac
