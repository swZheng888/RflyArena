#!/bin/bash
# auto_takeoff.sh - 自动发送起飞命令
# 用法: ./auto_takeoff.sh [延迟秒数]

DELAY=${1:-10}

echo "[auto_takeoff] 等待 ${DELAY} 秒后起飞..."
sleep $DELAY

echo "[auto_takeoff] 发送起飞命令..."
rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"

echo "[auto_takeoff] 起飞命令已发送"
