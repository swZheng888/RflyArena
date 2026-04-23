#!/bin/bash
#############################################################################
# 宿主机 Benchmark 脚本
# 在宿主机运行轨迹发布（benchmark_node），容器内只运行控制器
#
# 用法:
#   ./host_benchmark.sh [OPTIONS]
#
# 示例:
#   ./host_benchmark.sh -t circle -s 1.0 -d 30
#   ./host_benchmark.sh --trajectory figure8 --speed 1.5 --duration 60
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
TRAJECTORY="circle"
SPEED="1.0"
DURATION="30"
TASK_TYPE="dynamic"
CONTROLLER="nmpc"
TAKEOFF_HEIGHT="1.0"
LOG_DIR="${PROJECT_DIR}/benchmark_results"
WAIT_CONTROLLER=true
CONTROLLER_TIMEOUT=60
SKIP_ODOM=false
SKIP_TAKEOFF=false

# 清理函数
ODOM_PID=""
cleanup() {
    echo ""
    echo -e "${YELLOW}[INFO] 清理中...${NC}"

    # 停止 odom 节点
    if [ -n "$ODOM_PID" ] && kill -0 $ODOM_PID 2>/dev/null; then
        kill $ODOM_PID 2>/dev/null || true
        echo -e "${GREEN}[INFO] 已停止坐标转换节点${NC}"
    fi

    # 不杀死 benchmark_node，让它自然结束
}

trap cleanup EXIT

# 日志函数
log_info() { echo -e "${GREEN}[INFO]${NC} $1"; }
log_warn() { echo -e "${YELLOW}[WARN]${NC} $1"; }
log_error() { echo -e "${RED}[ERROR]${NC} $1"; }

# 帮助信息
show_help() {
    echo ""
    echo -e "${BLUE}宿主机 Benchmark 脚本${NC}"
    echo ""
    echo "在宿主机运行轨迹发布，配合 Docker 容器内的控制器使用。"
    echo ""
    echo "用法: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  -t, --trajectory TRAJ     轨迹类型 (默认: circle)"
    echo "  -s, --speed LEVEL         速度倍率 (默认: 1.0)"
    echo "  -d, --duration SECS       测试时长 (默认: 30)"
    echo "  -T, --task-type TYPE      任务类型: hover/dynamic (默认: dynamic)"
    echo "  -c, --controller CTRL     控制器类型: nmpc/pid (默认: nmpc)"
    echo "  --takeoff-height HEIGHT   起飞高度 (默认: 1.0)"
    echo "  --log-dir DIR             结果保存目录"
    echo "  --skip-odom               跳过坐标转换节点启动"
    echo "  --skip-takeoff            跳过起飞检测"
    echo "  --no-wait                 不等待控制器就绪"
    echo "  --timeout SECS            控制器等待超时 (默认: 60)"
    echo "  -h, --help                显示帮助"
    echo ""
    echo "支持的轨迹类型:"
    echo "  基础: circle, ellipse, long_ellipse, tilted_ellipse"
    echo "  八字: figure8, lemniscate"
    echo "  3D:   spiral_circle, cone3d, helix, spring"
    echo "  波浪: sine_wave, vertical_sine, horizontal_sine"
    echo "  几何: square, rectangle, triangle, star, diamond, hexagon, zigzag"
    echo ""
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -t|--trajectory) TRAJECTORY="$2"; shift 2 ;;
        -s|--speed) SPEED="$2"; shift 2 ;;
        -d|--duration) DURATION="$2"; shift 2 ;;
        -T|--task-type) TASK_TYPE="$2"; shift 2 ;;
        -c|--controller) CONTROLLER="$2"; shift 2 ;;
        --takeoff-height) TAKEOFF_HEIGHT="$2"; shift 2 ;;
        --log-dir) LOG_DIR="$2"; shift 2 ;;
        --skip-odom) SKIP_ODOM=true; shift ;;
        --skip-takeoff) SKIP_TAKEOFF=true; shift ;;
        --no-wait) WAIT_CONTROLLER=false; shift ;;
        --timeout) CONTROLLER_TIMEOUT="$2"; shift 2 ;;
        -h|--help) show_help; exit 0 ;;
        *) echo -e "${RED}未知选项: $1${NC}"; exit 1 ;;
    esac
done

# ============================================================================
# 打印配置
# ============================================================================
echo ""
echo -e "${CYAN}======================================================================${NC}"
echo -e "${CYAN}              宿主机 Benchmark 脚本${NC}"
echo -e "${CYAN}======================================================================${NC}"
echo ""
echo -e "${GREEN}任务类型:${NC}     $TASK_TYPE"
echo -e "${GREEN}轨迹类型:${NC}     $TRAJECTORY"
echo -e "${GREEN}速度倍率:${NC}     ${SPEED}x"
echo -e "${GREEN}测试时长:${NC}     ${DURATION}s"
echo -e "${GREEN}控制器:${NC}       $CONTROLLER"
echo -e "${GREEN}起飞高度:${NC}     ${TAKEOFF_HEIGHT}m"
echo -e "${GREEN}结果目录:${NC}     $LOG_DIR"
echo ""

# ============================================================================
# 检查 ROS 环境
# ============================================================================
log_info "检查 ROS 环境..."

# 加载 ROS 环境
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
fi

if [ -f "${PROJECT_DIR}/devel/setup.bash" ]; then
    source ${PROJECT_DIR}/devel/setup.bash
fi

# 检查 ROS Master
if ! rostopic list &>/dev/null; then
    log_error "无法连接 ROS Master!"
    log_error "请确保已启动: roscore"
    exit 1
fi
log_info "✓ ROS Master 已连接"

# ============================================================================
# 检查仿真器话题
# ============================================================================
log_info "检查仿真器话题..."

if rostopic list 2>/dev/null | grep -q "/mavros/local_position/odom"; then
    log_info "✓ /mavros/local_position/odom"
else
    log_warn "✗ /mavros/local_position/odom (需要 MAVROS)"
    log_warn "请确保已启动仿真器和 MAVROS"
fi

if rostopic list 2>/dev/null | grep -q "/mavros/state"; then
    log_info "✓ /mavros/state"
else
    log_warn "✗ /mavros/state"
fi

# ============================================================================
# 启动坐标转换节点 (如果需要)
# ============================================================================
if [ "$SKIP_ODOM" = false ]; then
    if rostopic list 2>/dev/null | grep -q "/vio/odometry_enu"; then
        log_info "✓ /vio/odometry_enu 已存在，跳过坐标转换节点"
    else
        log_info "启动坐标转换节点 (NED → ENU)..."

        # 查找 rflysim_odom_node.py
        ODOM_SCRIPT="${PROJECT_DIR}/src/nmpc_control/scripts/rflysim_odom_node.py"
        if [ ! -f "$ODOM_SCRIPT" ]; then
            log_error "找不到 rflysim_odom_node.py: $ODOM_SCRIPT"
            exit 1
        fi

        python3 "$ODOM_SCRIPT" &
        ODOM_PID=$!

        # 等待节点启动
        local attempts=0
        while [ $attempts -lt 10 ]; do
            if rostopic list 2>/dev/null | grep -q "/vio/odometry_enu"; then
                log_info "✓ 坐标转换节点已启动 (PID: $ODOM_PID)"
                break
            fi
            attempts=$((attempts + 1))
            sleep 1
        done

        if [ $attempts -eq 10 ]; then
            log_error "坐标转换节点启动失败！"
            exit 1
        fi
    fi
else
    log_info "跳过坐标转换节点启动 (--skip-odom)"
fi

# ============================================================================
# 等待容器内控制器就绪
# ============================================================================
if [ "$WAIT_CONTROLLER" = true ]; then
    log_info "等待容器内控制器就绪 (超时: ${CONTROLLER_TIMEOUT}s)..."

    wait_for_controller() {
        local timeout=$1
        local count=0

        while [ $count -lt $timeout ]; do
            # 检查控制器节点
            if [ "$CONTROLLER" = "nmpc" ]; then
                if rosnode list 2>/dev/null | grep -q "nmpc_control_node"; then
                    return 0
                fi
            elif [ "$CONTROLLER" = "pid" ]; then
                if rosnode list 2>/dev/null | grep -q "px4ctrl"; then
                    return 0
                fi
            fi

            # 备用检查：控制器发布的话题
            if rostopic info /mavros/setpoint_raw/attitude 2>/dev/null | grep -q "Publishers"; then
                return 0
            fi

            sleep 1
            count=$((count + 1))

            if [ $((count % 10)) -eq 0 ]; then
                log_info "等待中... ($count/${timeout}s)"
            fi
        done

        return 1
    }

    if wait_for_controller $CONTROLLER_TIMEOUT; then
        log_info "✓ 控制器已就绪"
    else
        log_warn "控制器未检测到，继续运行 benchmark..."
        log_warn "请确保 Docker 容器已启动控制器"
    fi
else
    log_info "跳过控制器等待 (--no-wait)"
fi

# ============================================================================
# 创建结果目录
# ============================================================================
mkdir -p "$LOG_DIR"

# ============================================================================
# 运行 Benchmark 节点
# ============================================================================
echo ""
echo -e "${CYAN}======================================================================${NC}"
echo -e "${CYAN}                     启动 Benchmark 测试${NC}"
echo -e "${CYAN}======================================================================${NC}"
echo ""

log_info "启动 benchmark_node..."
log_info "  task_type:        $TASK_TYPE"
log_info "  trajectory_type:  $TRAJECTORY"
log_info "  speed_level:      ${SPEED}x"
log_info "  duration:         ${DURATION}s"
log_info "  log_dir:          $LOG_DIR"
echo ""

# 运行 benchmark_node
rosrun benchmark_utils benchmark_node.py \
    _task_type:=$TASK_TYPE \
    _trajectory_type:=$TRAJECTORY \
    _speed_level:=$SPEED \
    _duration:=$DURATION \
    _odom_topic:=/vio/odometry_enu \
    _coordinate_frame:=1 \
    _log_dir:=$LOG_DIR \
    _takeoff_height_threshold:=0.5

EXIT_CODE=$?

# ============================================================================
# 测试完成
# ============================================================================
echo ""
echo -e "${CYAN}======================================================================${NC}"
if [ $EXIT_CODE -eq 0 ]; then
    echo -e "${GREEN}                     ✓ Benchmark 测试完成${NC}"
else
    echo -e "${RED}                     ✗ Benchmark 测试失败 (exit: $EXIT_CODE)${NC}"
fi
echo -e "${CYAN}======================================================================${NC}"
echo ""

log_info "结果目录: $LOG_DIR"
echo ""
echo "CSV 文件:"
ls -la "$LOG_DIR"/*.csv 2>/dev/null | tail -5 || echo "  (无 CSV 文件)"
echo ""

exit $EXIT_CODE
