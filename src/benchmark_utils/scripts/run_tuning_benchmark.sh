#!/bin/bash
# ============================================================
# PID自动调参 Benchmark脚本
# ============================================================
# 特点:
#   - 仅包含可行轨迹 (circle, figure8, ellipse, helix)
#   - 速度级别: 1x, 2x, 3x
#   - 无风扰 (专注于轨迹跟踪性能)
#   - 悬停测试
# ============================================================

set -e

# ==================== 配置 ====================
WORKSPACE="/root/trajectory_project"

# 控制器类型: pid 或 nmpc
CONTROLLER="${1:-pid}"

# 日志目录
LOG_BASE="$WORKSPACE/benchmark_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$LOG_BASE/${CONTROLLER}_tuning_${TIMESTAMP}"

ODOM_TOPIC="/vio/odometry_enu"
COORDINATE_FRAME=1  # ENU

# 测试时长
HOVER_DURATION=20       # 悬停20秒
TRAJECTORY_LOOPS=2      # 每个轨迹跑2圈

# 创建日志目录
mkdir -p "$LOG_DIR"

echo "============================================================"
echo "PID自动调参 Benchmark"
echo "============================================================"
echo "控制器: $CONTROLLER"
echo "日志目录: $LOG_DIR"
echo "里程计话题: $ODOM_TOPIC"
echo "============================================================"

echo ""
echo "请确保:"
echo "  1. RflySim仿真已启动"
echo "  2. 控制器已启动并处于Offboard模式"
echo "  3. 飞机已起飞"
echo ""
read -p "按Enter开始测试..."

# ============================================================
# 悬停测试
# ============================================================
echo ""
echo ">>> 悬停测试 <<<"
rosrun benchmark_utils benchmark_node.py \
    _task_type:=hover \
    _duration:=$HOVER_DURATION \
    _odom_topic:=$ODOM_TOPIC \
    _coordinate_frame:=$COORDINATE_FRAME \
    _log_dir:="$LOG_DIR/${CONTROLLER}_hover"

sleep 3

# ============================================================
# 可行轨迹测试 (多种轨迹, 多速度)
# ============================================================

# 可行轨迹列表
TRAJECTORIES=("circle" "figure8" "ellipse" "helix")

# 速度级别
SPEEDS=("1.0" "2.0" "3.0")

for SPEED in "${SPEEDS[@]}"; do
    echo ""
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║              速度级别: ${SPEED}x                              ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    
    for TRAJ in "${TRAJECTORIES[@]}"; do
        echo ""
        echo ">>> 轨迹: $TRAJ @ ${SPEED}x <<<"
        rosrun benchmark_utils benchmark_node.py \
            _task_type:=dynamic \
            _trajectory_type:=$TRAJ \
            _speed_level:=$SPEED \
            _num_loops:=$TRAJECTORY_LOOPS \
            _odom_topic:=$ODOM_TOPIC \
            _coordinate_frame:=$COORDINATE_FRAME \
            _log_dir:="$LOG_DIR/${CONTROLLER}_${TRAJ}_${SPEED}x"
        sleep 2
    done
done

# ============================================================
# 完成
# ============================================================
echo ""
echo "============================================================"
echo "所有测试完成!"
echo "结果保存在: $LOG_DIR"
echo "============================================================"

# 生成汇总报告
echo ""
echo "正在生成汇总报告..."
python3 "$WORKSPACE/src/benchmark_utils/scripts/generate_summary.py" \
    --results_dir "$LOG_DIR" 2>/dev/null || echo "汇总报告生成失败，请手动运行"

echo ""
echo "完成!"
