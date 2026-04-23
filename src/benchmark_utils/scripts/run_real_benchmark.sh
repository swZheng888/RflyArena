#!/bin/bash
# ============================================================
# 真机安全 Benchmark 启动脚本
# ============================================================
# 用法:
#   bash run_real_benchmark.sh [tasks_yaml] [odom_topic]
#
# 默认:
#   tasks_yaml = ../config/real_benchmark_tasks.yaml
#   odom_topic = /vrpn_client_node/droneyee08/0/odometry
#
# 工作流:
#   1. 启动此脚本 (控制器需已在运行)
#   2. 确认准备就绪后:
#      rostopic pub /real_benchmark/start std_msgs/Empty "{}"
#   3. 紧急中止:
#      rostopic pub /real_benchmark/abort std_msgs/Empty "{}"
# ============================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE="/root/trajectory_project"

# 参数
TASKS_FILE="${1:-$SCRIPT_DIR/../config/real_benchmark_tasks.yaml}"
# 仿真用 /vio/odometry_enu，真机用 /vrpn_client_node/droneyee08/0/odometry
ODOM_TOPIC="${2:-/vio/odometry_enu}"

# 日志目录
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$WORKSPACE/benchmark_results/real_${TIMESTAMP}"

# 颜色
GREEN='\033[0;32m'; YELLOW='\033[1;33m'; RED='\033[0;31m'; NC='\033[0m'

echo "============================================================"
echo -e "${GREEN} 真机安全 Benchmark${NC}"
echo "============================================================"
echo "  任务文件: $TASKS_FILE"
echo "  里程计:   $ODOM_TOPIC"
echo "  日志:     $LOG_DIR"
echo "============================================================"
echo ""
echo -e "${YELLOW}⚠️  确保:${NC}"
echo "  1. 控制器已启动 (RL/NMPC)"
echo "  2. 飞控已 Armed + Offboard"
echo "  3. 飞机已在悬停状态"
echo ""
echo -e "启动后发布 start 信号开始测试:"
echo -e "  ${GREEN}rostopic pub /real_benchmark/start std_msgs/Empty \"{}\"${NC}"
echo ""
echo -e "紧急中止:"
echo -e "  ${RED}rostopic pub /real_benchmark/abort std_msgs/Empty \"{}\"${NC}"
echo ""

cd "$WORKSPACE"
source devel/setup.bash

rosrun benchmark_utils real_benchmark_node.py \
    _tasks_file:="$TASKS_FILE" \
    _odom_topic:="$ODOM_TOPIC" \
    _log_dir:="$LOG_DIR" \
    _rate:=100
