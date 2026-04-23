#!/bin/bash
##############################################################################
# 录制 benchmark bag 包
# 记录：期望轨迹、当前位置、控制指令、求解时间
#
# 用法:
#   ./record_bag.sh                    # 默认文件名 (时间戳)
#   ./record_bag.sh rl_circle_test     # 自定义文件名
##############################################################################

BAG_DIR="${HOME}/bag_records"
mkdir -p "$BAG_DIR"

# 文件名
if [ -n "$1" ]; then
    BAG_NAME="$1"
else
    BAG_NAME="flight_$(date +%Y%m%d_%H%M%S)"
fi

TOPICS="
/position_cmd
/vio/odometry_enu
/mavros/setpoint_raw/attitude
/rl_control/solve_time_ms
/nmpc_control/solve_time_ms
/nmpc_control/control_freq_hz
/pid_control/solve_time_ms
/control_node/mode
/mavros/state
/vrpn_client_node/droneyee08/pose
/vrpn_client_node/droneyee08/0/odometry
/mavros/vision_pose/pose
"

echo "============================================================"
echo "  开始录制 rosbag"
echo "============================================================"
echo "  保存路径: ${BAG_DIR}/${BAG_NAME}.bag"
echo "  录制话题:"
for t in $TOPICS; do echo "    - $t"; done
echo "============================================================"
echo "  按 Ctrl+C 停止录制"
echo "============================================================"

rosbag record -O "${BAG_DIR}/${BAG_NAME}" $TOPICS
