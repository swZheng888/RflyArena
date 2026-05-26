#!/bin/bash
# ============================================================
# 综合Benchmark评估脚本 v2
# ============================================================
# 用法: bash run_comprehensive_benchmark.sh [pid|nmpc|rl] [model_path]
#
# 测试矩阵 (共 11 项, 约 15 分钟):
#   A组 (无风扰):  悬停 + 3种可行轨迹×2速度 + 1种不可行轨迹
#   B组 (有风扰):  悬停 + 3种可行轨迹×1速度 + 1种不可行轨迹
# ============================================================

set -e

# ==================== 配置 ====================
WORKSPACE="/root/trajectory_project"

# 控制器类型
CONTROLLER="${1:-pid}"
if [[ "$CONTROLLER" != "pid" && "$CONTROLLER" != "nmpc" && "$CONTROLLER" != "rl" ]]; then
    echo "错误: 不支持的控制器 '$CONTROLLER'"
    echo "用法: $0 [pid|nmpc|rl] [model_path]"
    exit 1
fi

# RL 模型路径
RL_MODEL_PATH="${2:-$WORKSPACE/src/rl_control/model/policy.onnx}"

# 日志目录: benchmark_results/{控制器}_{时间戳}/
LOG_BASE="$WORKSPACE/benchmark_results"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
LOG_DIR="$LOG_BASE/${CONTROLLER}_${TIMESTAMP}"

# 话题
ODOM_TOPIC="/vio/odometry_enu"
COORDINATE_FRAME=2  # ENU

# 风扰
WIND_SPEED=3      # m/s
WIND_DIRECTION=45    # 度

# ==================== 测试时长 (优化: 缩短) ====================
HOVER_DURATION=10       # 悬停 30s (够评估精度)
TRAJ_DURATION=60        # 轨迹 30s (约2圈)
SLEEP_BETWEEN=2         # 测试间隔 2s

# ==================== 轨迹配置 ====================
FEASIBLE=("circle" "figure8" "ellipse")
INFEASIBLE=("zigzag")

# ==================== 颜色 ====================
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'
CYAN='\033[0;36m'; NC='\033[0m'

# 创建日志目录
mkdir -p "$LOG_DIR"

# ==================== 公共函数 ====================
TEST_NUM=0
TOTAL_TESTS=0

count_tests() {
    # A: 1悬停 + 3可行×2速度 + 1不可行 = 8
    # B: 1悬停 + 3可行×1速度 + 1不可行 = 5
    TOTAL_TESTS=13
}

run_test() {
    local name="$1"
    local task_type="$2"
    local traj_type="$3"
    local speed="$4"
    local duration="$5"
    local sub_dir="$6"

    TEST_NUM=$((TEST_NUM + 1))
    local test_dir="$LOG_DIR/$sub_dir"
    mkdir -p "$test_dir"

    echo ""
    echo -e "${CYAN}[$TEST_NUM/$TOTAL_TESTS] $name${NC}"
    echo -e "  轨迹=$traj_type 速度=${speed}x 时长=${duration}s"
    echo -e "  日志: $sub_dir/"

    rosrun benchmark_utils benchmark_node.py \
        _task_type:="$task_type" \
        _trajectory_type:="$traj_type" \
        _speed_level:="$speed" \
        _duration:="$duration" \
        _odom_topic:=$ODOM_TOPIC \
        _coordinate_frame:=$COORDINATE_FRAME \
        _log_dir:="$test_dir"

    sleep $SLEEP_BETWEEN
}

# ==================== 打印信息 ====================
count_tests

echo "============================================================"
echo -e "${GREEN} 综合Benchmark评估 v2${NC}"
echo "============================================================"
echo "  控制器:     $CONTROLLER"
echo "  测试项数:   $TOTAL_TESTS (预计 ~15 分钟)"
echo "  日志目录:   $LOG_DIR"
echo "  里程计:     $ODOM_TOPIC"
echo "  坐标系:     ENU"
echo "============================================================"

cd "$WORKSPACE"
source devel/setup.bash

echo ""
echo "请确保:"
echo "  1. RflySim 仿真已启动"
if [ "$CONTROLLER" = "rl" ]; then
    echo "  2. RL 控制器已启动 (model: $RL_MODEL_PATH)"
else
    echo "  2. $CONTROLLER 控制器已启动并处于 Offboard"
fi
echo "  3. 飞机已起飞"
echo ""
read -p "按 Enter 开始测试..."

# ============================================================
# A组: 无风扰
# ============================================================
echo ""
echo -e "${GREEN}╔════════════════════════════════════╗${NC}"
echo -e "${GREEN}║      A组: 无风扰 (baseline)        ║${NC}"
echo -e "${GREEN}╚════════════════════════════════════╝${NC}"

# A1: 悬停
run_test "A1: 悬停 (无风)" hover "circle" 1.0 $HOVER_DURATION \
    "A1_hover_no_wind"

# A2: 可行轨迹 @1.0x
for TRAJ in "${FEASIBLE[@]}"; do
    run_test "A2: $TRAJ @1.0x (无风)" dynamic "$TRAJ" 1.0 $TRAJ_DURATION \
        "A2_${TRAJ}_1.0x_no_wind"
done

# A3: 可行轨迹 @2.0x
for TRAJ in "${FEASIBLE[@]}"; do
    run_test "A3: $TRAJ @2.0x (无风)" dynamic "$TRAJ" 2.0 $TRAJ_DURATION \
        "A3_${TRAJ}_2.0x_no_wind"
done

# A4: 不可行轨迹
run_test "A4: zigzag @0.5x (无风)" dynamic "zigzag" 0.5 $TRAJ_DURATION \
    "A4_zigzag_0.5x_no_wind"

# ============================================================
# B组: 有风扰
# ============================================================
echo ""
echo -e "${YELLOW}╔════════════════════════════════════╗${NC}"
echo -e "${YELLOW}║      B组: 湍流风扰 (${WIND_SPEED}m/s)       ║${NC}"
echo -e "${YELLOW}╚════════════════════════════════════╝${NC}"

# 启动湍流风
echo -e "${YELLOW}启动湍流风扰动...${NC}"
roslaunch benchmark_utils wind_disturbance.launch \
    wind_type:=turbulent \
    turbulent_intensity:=$WIND_SPEED \
    start_delay:=2.0 &
WIND_PID=$!
sleep 3

# B1: 悬停 + 风
run_test "B1: 悬停 (湍流风)" hover "circle" 1.0 $HOVER_DURATION \
    "B1_hover_turbulent_wind"

# B2: 可行轨迹 @1.0x + 风
for TRAJ in "${FEASIBLE[@]}"; do
    run_test "B2: $TRAJ @1.0x (湍流风)" dynamic "$TRAJ" 1.0 $TRAJ_DURATION \
        "B2_${TRAJ}_1.0x_wind"
done

# B3: 不可行轨迹 + 风
run_test "B3: zigzag @0.5x (湍流风)" dynamic "zigzag" 0.5 $TRAJ_DURATION \
    "B3_zigzag_0.5x_wind"

# 停止风
kill $WIND_PID 2>/dev/null || true
echo -e "${GREEN}风扰已停止${NC}"

# ============================================================
# 分析
# ============================================================
echo ""
echo -e "${GREEN}============================================================${NC}"
echo -e "${GREEN} 所有测试完成！正在分析...${NC}"
echo -e "${GREEN}============================================================${NC}"

# 逐项分析
for dir in "$LOG_DIR"/*/; do
    if [ -d "$dir" ]; then
        test_name=$(basename "$dir")
        csv_file=$(ls -t "$dir"/*.csv 2>/dev/null | head -1)
        if [ -n "$csv_file" ]; then
            echo "  分析: $test_name"
            python3 "$WORKSPACE/src/benchmark_utils/scripts/benchmark_analyzer.py" \
                --csv "$csv_file" \
                --output "$dir" \
                --name "$test_name" 2>/dev/null || true
        fi
    fi
done

# 汇总
python3 "$WORKSPACE/src/benchmark_utils/scripts/generate_summary.py" \
    --results_dir "$LOG_DIR" 2>/dev/null || echo "Warning: 汇总报告生成失败"

# 本地排行榜提交包
python3 "$WORKSPACE/src/benchmark_utils/scripts/generate_submission_package.py" \
    --results-dir "$LOG_DIR" \
    --controller "$CONTROLLER" \
    --select-best \
    --analyze-missing \
    --no-plot 2>/dev/null || echo "Warning: 提交包生成失败"

# ==================== 输出结果 ====================
echo ""
echo "============================================================"
echo -e "${GREEN} 测试结果${NC}"
echo "============================================================"
echo "  目录:  $LOG_DIR"
echo "  提交包: $LOG_DIR/local_rank_ready.zip"
echo ""
echo "  结构:"
ls -d "$LOG_DIR"/*/ 2>/dev/null | while read d; do
    name=$(basename "$d")
    csv_count=$(ls "$d"/*.csv 2>/dev/null | wc -l)
    png_count=$(ls "$d"/*.png 2>/dev/null | wc -l)
    echo "    $name/  ($csv_count csv, $png_count png)"
done
echo ""
echo "============================================================"
