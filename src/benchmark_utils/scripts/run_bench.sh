#!/bin/bash
# ============================================================
# 真机 Benchmark 快速启动脚本
# ============================================================
# 用法:
#   ./run_bench.sh ctrl nmpc       # 启动 NMPC 控制器
#   ./run_bench.sh ctrl pid        # 启动 PID 控制器
#   ./run_bench.sh ctrl rl         # 启动 RL 控制器
#   ./run_bench.sh bench nmpc      # 启动 benchmark
#   ./run_bench.sh trigger         # 发送开始信号
#   ./run_bench.sh abort           # 紧急中止
#   ./run_bench.sh sim nmpc        # 仿真模式 (RflySim)
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"

# source 工作空间
source "$WS_DIR/devel/setup.bash" 2>/dev/null || {
    echo "❌ 找不到 devel/setup.bash"
    exit 1
}

CMD="${1:-help}"
CTRL="${2:-nmpc}"

case "$CMD" in

    # ==================== 启动控制器 ====================
    ctrl|controller)
        echo "🚁 启动控制器: $CTRL"
        case "$CTRL" in
            nmpc)
                roslaunch nmpc_control real_exp.launch \
                    takeoff_vz_max:="${3:-0.3}" \
                    takeoff_height:="${4:-1.0}"
                ;;
            pid)
                roslaunch px4ctrl run.launch
                ;;
            rl)
                roslaunch rl_control real_exp.launch
                ;;
            pirl)
                roslaunch rl_control real_exp.launch controller_type:=pirl
                ;;
            *)
                echo "❌ 未知控制器: $CTRL (可选: nmpc/pid/rl/pirl)"
                exit 1
                ;;
        esac
        ;;

    # ==================== 仿真控制器 ====================
    sim)
        echo "🖥️  仿真模式: $CTRL"
        case "$CTRL" in
            nmpc)
                roslaunch nmpc_control rflysim_nmpc.launch
                ;;
            pid)
                roslaunch px4ctrl rflysim_pid.launch
                ;;
            rl)
                roslaunch rl_control rflysim_rl.launch
                ;;
            *)
                echo "❌ 未知控制器: $CTRL (可选: nmpc/pid/rl)"
                exit 1
                ;;
        esac
        ;;

    # ==================== 启动 Benchmark ====================
    bench|benchmark)
        RECORD="${3:-true}"
        echo "📊 启动 benchmark: $CTRL (录bag=$RECORD)"
        roslaunch benchmark_utils real_benchmark.launch \
            controller_name:="$CTRL" \
            record_bag:="$RECORD"
        ;;

    # ==================== 控制指令 ====================
    trigger|start)
        echo ">>> 发送开始信号..."
        rostopic pub -1 /real_benchmark/start std_msgs/Empty "{}"
        echo "✅ 已发送"
        ;;

    abort|stop)
        echo ">>> 紧急中止..."
        rostopic pub -1 /real_benchmark/abort std_msgs/Empty "{}"
        echo "⚠️  已中止"
        ;;

    takeoff)
        echo ">>> 起飞..."
        rostopic pub -1 /px4ctrl/takeoff_land quadrotor_msgs/TakeoffLand "takeoff_land_cmd: 1"
        ;;

    # ==================== 帮助 ====================
    help|*)
        echo "用法: $0 <命令> <控制器> [参数]"
        echo ""
        echo "控制器启动:"
        echo "  ctrl nmpc [vz_max] [height]  真机NMPC (默认vz=0.3, h=1.0)"
        echo "  ctrl pid                     真机PID"
        echo "  ctrl rl                      真机RL"
        echo "  sim  nmpc/pid/rl             仿真模式"
        echo ""
        echo "Benchmark:"
        echo "  bench nmpc/pid/rl [bag]      启动benchmark (bag=true/false)"
        echo "  trigger                      发送开始信号"
        echo "  abort                        紧急中止 → 悬停"
        echo "  takeoff                      PID起飞"
        echo ""
        echo "典型流程:"
        echo "  终端1: $0 ctrl nmpc"
        echo "  终端2: $0 bench nmpc"
        echo "  终端3: $0 trigger"
        ;;
esac
