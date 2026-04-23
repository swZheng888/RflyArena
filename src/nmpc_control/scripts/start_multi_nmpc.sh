#!/bin/bash
# 启动多个NMPC控制器实例用于并行调参

# 默认启动4个实例
NUM_INSTANCES=${1:-4}

echo "==========================================================="
echo "启动 $NUM_INSTANCES 个NMPC控制器实例"
echo "==========================================================="

# 检查ROS环境
if [ -z "$ROS_MASTER_URI" ]; then
    echo "错误: ROS环境未配置"
    echo "请先运行: source /root/trajectory_project/devel/setup.bash"
    exit 1
fi

echo ""
echo "实例配置:"
echo "-----------------------------------------------------------"
for ((i=1; i<=$NUM_INSTANCES; i++)); do
    MAVLINK_PORT=$((14540 + (i-1) * 10))
    echo "  实例 $i (UAV $i):"
    echo "    - ROS命名空间: /uav$i"
    echo "    - MAVLink端口: $MAVLINK_PORT"
    echo "    - 里程计话题: /uav$i/vio/odometry"
    echo "    - 控制话题: /uav$i/mavros/setpoint_raw/attitude"
done
echo "==========================================================="

# 确保ROS Master运行
if ! pgrep -x "rosmaster" > /dev/null; then
    echo "启动 ROS Master..."
    roscore &
    ROSCORE_PID=$!
    echo "ROS Master PID: $ROSCORE_PID"
    echo "等待 ROS Master 启动..."
    sleep 3
else
    echo "ROS Master 已在运行"
fi

# 启动每个实例
for ((i=1; i<=$NUM_INSTANCES; i++)); do
    echo ""
    echo "[实例 $i] 启动中..."
    
    # 使用nohup在后台启动，输出到日志文件
    nohup roslaunch nmpc_control rflysim_nmpc_multi.launch \
        uav_id:=$i \
        > /tmp/nmpc_uav${i}.log 2>&1 &
    
    PID=$!
    echo "[实例 $i] 已启动 (PID: $PID)"
    echo "[实例 $i] 日志: /tmp/nmpc_uav${i}.log"
    
    # 等待节点启动
    sleep 3
done

echo ""
echo "==========================================================="
echo "✅ 所有实例已启动！"
echo "==========================================================="
echo ""
echo "监控日志:"
for ((i=1; i<=$NUM_INSTANCES; i++)); do
    echo "  tail -f /tmp/nmpc_uav${i}.log"
done
echo ""
echo "停止所有实例:"
echo "  pkill -f 'rflysim_nmpc_multi.launch'"
echo ""
echo "检查ROS话题:"
echo "  rostopic list | grep uav"
echo ""
