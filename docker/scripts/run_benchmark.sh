#!/bin/bash
#############################################################################
# 基准测试运行脚本
# 在受限资源环境中运行控制器基准测试
#
# 新架构 (默认): 宿主机运行轨迹发布，容器内只运行控制器
# 旧架构 (--legacy): 容器内运行所有组件
#############################################################################

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DOCKER_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_DIR="$(dirname "$DOCKER_DIR")"

# 默认值
PROFILE="jetson_nano"
CONTROLLER="nmpc"
TRAJECTORY="circle"
DURATION=30
SPEED=1.0
INTERACTIVE=false
SKIP_TAKEOFF=false
TAKEOFF_HEIGHT=1.0
ROS_MASTER_URI="http://127.0.0.1:11311"

# 新架构选项
LEGACY_MODE=false          # 默认使用新架构
HOST_BENCHMARK=true        # 宿主机运行 benchmark

# 清理函数
CONTAINER_PID=""
HOST_BENCHMARK_PID=""
cleanup() {
    echo ""
    echo -e "${YELLOW}[INFO] 清理中...${NC}"

    # 停止宿主机 benchmark 脚本
    if [ -n "$HOST_BENCHMARK_PID" ] && kill -0 $HOST_BENCHMARK_PID 2>/dev/null; then
        kill $HOST_BENCHMARK_PID 2>/dev/null || true
        echo -e "${GREEN}[INFO] 已停止宿主机 benchmark 脚本${NC}"
    fi

    # 停止容器
    if [ -n "$CONTAINER_NAME" ]; then
        docker stop "$CONTAINER_NAME" 2>/dev/null || true
        echo -e "${GREEN}[INFO] 已停止容器${NC}"
    fi
}

trap cleanup EXIT INT TERM

# 帮助信息
show_help() {
    echo ""
    echo -e "${BLUE}UAV Control Benchmark Runner${NC}"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo -e "${GREEN}架构模式:${NC}"
    echo "  默认 (推荐)    宿主机运行轨迹发布，容器只运行控制器"
    echo "  --legacy       容器内运行所有组件 (旧架构)"
    echo ""
    echo -e "${GREEN}资源配置 (--profile):${NC}"
    echo "  jetson_nano    Jetson Nano (1核, 2GB)"
    echo "  jetson_xavier  Jetson Xavier NX (2核, 4GB)"
    echo "  rpi4           Raspberry Pi 4 (0.5核, 1GB)"
    echo "  fmu            PX4 FMU 极限 (0.1核, 256MB)"
    echo "  unlimited      无资源限制 (对照组)"
    echo ""
    echo -e "${GREEN}Options:${NC}"
    echo "  -p, --profile PROFILE     资源配置 (默认: jetson_nano)"
    echo "  -c, --controller CTRL     控制器: nmpc/pid (默认: nmpc)"
    echo "  -t, --trajectory TRAJ     轨迹类型 (默认: circle)"
    echo "  -d, --duration SECS       测试时长 (默认: 30)"
    echo "  -s, --speed LEVEL         速度倍率 (默认: 1.0)"
    echo "  -i, --interactive         交互模式 (进入容器 shell)"
    echo "  --legacy                  使用旧架构 (容器内运行所有组件)"
    echo "  --skip-takeoff            跳过起飞"
    echo "  --takeoff-height HEIGHT   起飞高度 (默认: 1.0)"
    echo "  --ros-master URI          ROS Master URI"
    echo "  -h, --help                显示帮助"
    echo ""
    echo -e "${GREEN}示例:${NC}"
    echo "  # 新架构 (推荐): 容器只运行控制器"
    echo "  $0 -p jetson_nano -c nmpc -t circle"
    echo ""
    echo "  # 旧架构: 容器运行所有组件"
    echo "  $0 -p jetson_nano -c nmpc -t circle --legacy"
    echo ""
    echo "  # 交互模式"
    echo "  $0 -p jetson_nano -i"
    echo ""
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -p|--profile) PROFILE="$2"; shift 2 ;;
        -c|--controller) CONTROLLER="$2"; shift 2 ;;
        -t|--trajectory) TRAJECTORY="$2"; shift 2 ;;
        -d|--duration) DURATION="$2"; shift 2 ;;
        -s|--speed) SPEED="$2"; shift 2 ;;
        -i|--interactive) INTERACTIVE=true; shift ;;
        --legacy) LEGACY_MODE=true; HOST_BENCHMARK=false; shift ;;
        --skip-takeoff) SKIP_TAKEOFF=true; shift ;;
        --takeoff-height) TAKEOFF_HEIGHT="$2"; shift 2 ;;
        --ros-master) ROS_MASTER_URI="$2"; shift 2 ;;
        -h|--help) show_help; exit 0 ;;
        *) echo -e "${RED}Unknown option: $1${NC}"; show_help; exit 1 ;;
    esac
done

# 资源配置
case "$PROFILE" in
    "jetson_nano")  CPU_LIMIT="1.0"; MEM_LIMIT="2g" ;;
    "jetson_xavier") CPU_LIMIT="2.0"; MEM_LIMIT="4g" ;;
    "rpi4")         CPU_LIMIT="0.5"; MEM_LIMIT="1g" ;;
    "fmu")          CPU_LIMIT="0.1"; MEM_LIMIT="256m" ;;
    "unlimited")    CPU_LIMIT=""; MEM_LIMIT="" ;;
    *)
        echo -e "${RED}Error: Unknown profile '$PROFILE'${NC}"
        exit 1
        ;;
esac

# 镜像和容器名
IMAGE="uav-benchmark:latest"
CONTAINER_NAME="benchmark_${PROFILE}_$(date +%Y%m%d_%H%M%S)"

# 创建结果目录
RESULTS_DIR="$PROJECT_DIR/benchmark_results/docker_${PROFILE}"
TUNING_DIR="$PROJECT_DIR/tuning_results/docker_${PROFILE}"
LOG_DIR="$PROJECT_DIR/docker_logs"
mkdir -p "$RESULTS_DIR" "$TUNING_DIR" "$LOG_DIR"

# 日志文件
LOG_FILE="$LOG_DIR/benchmark_$(date +%Y%m%d_%H%M%S).log"

# ============================================================================
# 打印配置
# ============================================================================
echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  UAV Control Benchmark${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""

if [ "$LEGACY_MODE" = true ]; then
    echo -e "${YELLOW}架构模式:${NC}    旧架构 (容器内运行所有组件)"
else
    echo -e "${GREEN}架构模式:${NC}    新架构 (推荐 - 容器只运行控制器)"
fi

echo -e "${GREEN}Profile:${NC}      $PROFILE"
echo -e "${GREEN}CPU Limit:${NC}    ${CPU_LIMIT:-unlimited}"
echo -e "${GREEN}Memory:${NC}       ${MEM_LIMIT:-unlimited}"
echo -e "${GREEN}Controller:${NC}   $CONTROLLER"
echo -e "${GREEN}Trajectory:${NC}   $TRAJECTORY"
echo -e "${GREEN}Duration:${NC}     ${DURATION}s"
echo -e "${GREEN}Speed:${NC}        ${SPEED}x"
echo -e "${GREEN}Skip Takeoff:${NC} $SKIP_TAKEOFF"
echo -e "${YELLOW}Log file:${NC}     $LOG_FILE"
echo ""

# ============================================================================
# 交互模式
# ============================================================================
if [ "$INTERACTIVE" = true ]; then
    echo -e "${YELLOW}Starting interactive session...${NC}"
    echo ""

    docker run -it --rm \
        --name "$CONTAINER_NAME" \
        ${CPU_LIMIT:+--cpus="$CPU_LIMIT"} \
        ${MEM_LIMIT:+--memory="$MEM_LIMIT" --memory-swap="$MEM_LIMIT"} \
        -e ROS_MASTER_URI="$ROS_MASTER_URI" \
        -e ENABLE_RESOURCE_MONITOR=true \
        -e BENCHMARK_MODE=interactive \
        -e CONTROLLER="$CONTROLLER" \
        -e CPU_LIMIT="$CPU_LIMIT" \
        -e MEM_LIMIT="$MEM_LIMIT" \
        -v "$RESULTS_DIR:/root/catkin_ws/benchmark_results" \
        -v "$TUNING_DIR:/root/catkin_ws/tuning_results" \
        -v "$LOG_DIR:/root/catkin_ws/logs" \
        -v "$PROJECT_DIR/src:/root/catkin_ws/src:ro" \
        --network host \
        "$IMAGE" \
        bash

    exit 0
fi

# ============================================================================
# 旧架构模式 (--legacy)
# ============================================================================
if [ "$LEGACY_MODE" = true ]; then
    echo -e "${YELLOW}使用旧架构: 容器内运行所有组件...${NC}"
    echo ""

    docker run --rm \
        --name "$CONTAINER_NAME" \
        ${CPU_LIMIT:+--cpus="$CPU_LIMIT"} \
        ${MEM_LIMIT:+--memory="$MEM_LIMIT" --memory-swap="$MEM_LIMIT"} \
        -e ROS_MASTER_URI="$ROS_MASTER_URI" \
        -e ENABLE_RESOURCE_MONITOR=true \
        -e BENCHMARK_MODE="benchmark" \
        -e CONTROLLER="$CONTROLLER" \
        -e TRAJECTORY="$TRAJECTORY" \
        -e DURATION="$DURATION" \
        -e SPEED="$SPEED" \
        -e SKIP_TAKEOFF="$SKIP_TAKEOFF" \
        -e TAKEOFF_HEIGHT="$TAKEOFF_HEIGHT" \
        -v "$RESULTS_DIR:/root/catkin_ws/benchmark_results" \
        -v "$TUNING_DIR:/root/catkin_ws/tuning_results" \
        -v "$LOG_DIR:/root/catkin_ws/logs" \
        -v "$PROJECT_DIR/src:/root/catkin_ws/src:ro" \
        --network host \
        "$IMAGE" 2>&1 | tee "$LOG_FILE"

    echo ""
    echo -e "${GREEN}============================================${NC}"
    echo -e "${GREEN}✓ Benchmark completed (Legacy Mode)${NC}"
    echo -e "${GREEN}============================================${NC}"

    exit 0
fi

# ============================================================================
# 新架构模式 (默认): 宿主机运行 benchmark，容器只运行控制器
# ============================================================================
echo -e "${GREEN}使用新架构: 容器只运行控制器，宿主机运行轨迹发布${NC}"
echo ""

# 检查宿主机 ROS 环境
echo -e "${CYAN}[1/4] 检查宿主机 ROS 环境...${NC}"

if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

if [ -f "$PROJECT_DIR/devel/setup.bash" ]; then
    source "$PROJECT_DIR/devel/setup.bash"
fi

if ! rostopic list &>/dev/null; then
    echo -e "${RED}[ERROR] 无法连接 ROS Master!${NC}"
    echo -e "${RED}请确保已启动: roscore${NC}"
    exit 1
fi
echo -e "${GREEN}✓ ROS Master 已连接${NC}"

# 检查仿真器
if rostopic list 2>/dev/null | grep -q "/mavros/local_position/odom"; then
    echo -e "${GREEN}✓ MAVROS 已就绪${NC}"
else
    echo -e "${YELLOW}[WARN] /mavros/local_position/odom 未检测到${NC}"
    echo -e "${YELLOW}请确保仿真器和 MAVROS 已启动${NC}"
fi

# 启动容器 (只运行控制器)
echo ""
echo -e "${CYAN}[2/4] 启动 Docker 容器 (仅控制器)...${NC}"
echo -e "${CYAN}Docker command: docker run --cpus=$CPU_LIMIT --memory=$MEM_LIMIT ...${NC}"
echo ""

# 后台启动容器
docker run --rm \
    --name "$CONTAINER_NAME" \
    ${CPU_LIMIT:+--cpus="$CPU_LIMIT"} \
    ${MEM_LIMIT:+--memory="$MEM_LIMIT" --memory-swap="$MEM_LIMIT"} \
    -e ROS_MASTER_URI="$ROS_MASTER_URI" \
    -e ENABLE_RESOURCE_MONITOR=true \
    -e BENCHMARK_MODE="controller_only" \
    -e CONTROLLER="$CONTROLLER" \
    -e CPU_LIMIT="$CPU_LIMIT" \
    -e MEM_LIMIT="$MEM_LIMIT" \
    -v "$RESULTS_DIR:/root/catkin_ws/benchmark_results" \
    -v "$TUNING_DIR:/root/catkin_ws/tuning_results" \
    -v "$LOG_DIR:/root/catkin_ws/logs" \
    -v "$PROJECT_DIR/src:/root/catkin_ws/src:ro" \
    --network host \
    "$IMAGE" 2>&1 | tee "$LOG_DIR/controller_${CONTROLLER}.log" &

CONTAINER_PID=$!

# 等待控制器启动
echo ""
echo -e "${CYAN}[3/4] 等待容器内控制器启动...${NC}"
sleep 5

# 检查控制器是否就绪
wait_for_controller() {
    local timeout=30
    local count=0

    while [ $count -lt $timeout ]; do
        if [ "$CONTROLLER" = "nmpc" ]; then
            if rosnode list 2>/dev/null | grep -q "nmpc_control_node"; then
                return 0
            fi
        elif [ "$CONTROLLER" = "pid" ]; then
            if rosnode list 2>/dev/null | grep -q "px4ctrl"; then
                return 0
            fi
        fi

        sleep 1
        count=$((count + 1))

        if [ $((count % 5)) -eq 0 ]; then
            echo -e "${YELLOW}等待控制器... ($count/${timeout}s)${NC}"
        fi
    done

    return 1
}

if wait_for_controller; then
    echo -e "${GREEN}✓ 控制器已就绪${NC}"
else
    echo -e "${YELLOW}[WARN] 控制器未检测到，继续启动 benchmark...${NC}"
fi

# 启动宿主机 benchmark 脚本
echo ""
echo -e "${CYAN}[4/4] 启动宿主机 Benchmark 脚本...${NC}"
echo ""

SKIP_TAKEOFF_FLAG=""
if [ "$SKIP_TAKEOFF" = true ]; then
    SKIP_TAKEOFF_FLAG="--skip-takeoff"
fi

# 运行宿主机 benchmark 脚本
"$SCRIPT_DIR/host_benchmark.sh" \
    --trajectory "$TRAJECTORY" \
    --speed "$SPEED" \
    --duration "$DURATION" \
    --controller "$CONTROLLER" \
    --takeoff-height "$TAKEOFF_HEIGHT" \
    --log-dir "$RESULTS_DIR" \
    --skip-odom \
    --no-wait \
    $SKIP_TAKEOFF_FLAG 2>&1 | tee -a "$LOG_FILE"

BENCHMARK_EXIT_CODE=${PIPESTATUS[0]}

# 停止容器
echo ""
echo -e "${CYAN}停止控制器容器...${NC}"
docker stop "$CONTAINER_NAME" 2>/dev/null || true

# 等待容器退出
wait $CONTAINER_PID 2>/dev/null || true

# 显示结果
echo ""
echo -e "${CYAN}============================================${NC}"
if [ $BENCHMARK_EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}✓ Benchmark 完成 (新架构)${NC}"
else
    echo -e "${RED}✗ Benchmark 失败 (exit: $BENCHMARK_EXIT_CODE)${NC}"
fi
echo -e "${CYAN}============================================${NC}"
echo ""
echo -e "${GREEN}Results:${NC}"
echo "  - Benchmark: $RESULTS_DIR"
echo "  - Tuning:    $TUNING_DIR"
echo "  - Log:       $LOG_FILE"
echo ""

echo -e "${CYAN}CSV files in results directory:${NC}"
ls -la "$RESULTS_DIR"/*.csv 2>/dev/null | tail -5 || echo "  (no CSV files)"

exit $BENCHMARK_EXIT_CODE
