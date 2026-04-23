#!/bin/bash
#############################################################################
# 实时资源监控脚本
# 监控正在运行的 Docker 容器资源使用情况
#############################################################################

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m'

# 帮助信息
show_help() {
    echo ""
    echo -e "${BLUE}Docker Container Resource Monitor${NC}"
    echo ""
    echo "Usage: $0 [OPTIONS] [CONTAINER_NAME]"
    echo ""
    echo "Options:"
    echo "  -a, --all         监控所有 benchmark 容器"
    echo "  -i, --interval N  刷新间隔秒数 (默认: 1)"
    echo "  -h, --help        显示帮助"
    echo ""
    echo "Examples:"
    echo "  $0 benchmark_jetson_nano"
    echo "  $0 -a"
    echo "  $0 -i 0.5 benchmark_rpi4"
    echo ""
}

# 默认值
INTERVAL=1
MONITOR_ALL=false
CONTAINER=""

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -a|--all)
            MONITOR_ALL=true
            shift
            ;;
        -i|--interval)
            INTERVAL="$2"
            shift 2
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            CONTAINER="$1"
            shift
            ;;
    esac
done

# 格式化字节数
format_bytes() {
    local bytes=$1
    if [ $bytes -ge 1073741824 ]; then
        echo "$(echo "scale=2; $bytes/1073741824" | bc)GB"
    elif [ $bytes -ge 1048576 ]; then
        echo "$(echo "scale=1; $bytes/1048576" | bc)MB"
    else
        echo "$(echo "scale=0; $bytes/1024" | bc)KB"
    fi
}

# 监控单个容器
monitor_container() {
    local container=$1

    # 检查容器是否存在
    if ! docker ps --format '{{.Names}}' | grep -q "^${container}$"; then
        echo -e "${RED}Container not found: $container${NC}"
        return 1
    fi

    echo ""
    echo -e "${BLUE}============================================${NC}"
    echo -e "${BLUE}  Monitoring: $container${NC}"
    echo -e "${BLUE}============================================${NC}"
    echo ""

    # 获取容器资源限制
    local cpu_limit=$(docker inspect --format '{{.HostConfig.NanoCpus}}' "$container" 2>/dev/null)
    local mem_limit=$(docker inspect --format '{{.HostConfig.Memory}}' "$container" 2>/dev/null)

    if [ "$cpu_limit" != "0" ] && [ -n "$cpu_limit" ]; then
        cpu_limit=$(echo "scale=2; $cpu_limit/1000000000" | bc)
        echo -e "${GREEN}CPU Limit:${NC} ${cpu_limit} cores"
    else
        echo -e "${GREEN}CPU Limit:${NC} unlimited"
    fi

    if [ "$mem_limit" != "0" ] && [ -n "$mem_limit" ]; then
        echo -e "${GREEN}Memory Limit:${NC} $(format_bytes $mem_limit)"
    else
        echo -e "${GREEN}Memory Limit:${NC} unlimited"
    fi
    echo ""

    # 实时监控
    echo -e "${CYAN}Press Ctrl+C to stop${NC}"
    echo ""
    printf "%-12s %-12s %-15s %-15s %-10s\n" "CPU %" "MEM %" "MEM USAGE" "NET I/O" "BLOCK I/O"
    echo "----------------------------------------------------------------"

    while true; do
        stats=$(docker stats --no-stream --format \
            "{{.CPUPerc}}\t{{.MemPerc}}\t{{.MemUsage}}\t{{.NetIO}}\t{{.BlockIO}}" \
            "$container" 2>/dev/null)

        if [ -z "$stats" ]; then
            echo -e "${RED}Container stopped${NC}"
            break
        fi

        echo -e "\r$stats"
        sleep "$INTERVAL"
    done
}

# 监控所有 benchmark 容器
monitor_all() {
    echo ""
    echo -e "${BLUE}============================================${NC}"
    echo -e "${BLUE}  All Benchmark Containers${NC}"
    echo -e "${BLUE}============================================${NC}"
    echo ""
    echo -e "${CYAN}Press Ctrl+C to stop${NC}"
    echo ""

    while true; do
        clear
        echo -e "${BLUE}UAV Benchmark Resource Monitor${NC} - $(date '+%Y-%m-%d %H:%M:%S')"
        echo ""
        docker stats --no-stream --format \
            "table {{.Name}}\t{{.CPUPerc}}\t{{.MemUsage}}\t{{.MemPerc}}\t{{.NetIO}}" \
            $(docker ps --filter "name=benchmark_" --format "{{.Names}}" 2>/dev/null) 2>/dev/null

        sleep "$INTERVAL"
    done
}

# 主逻辑
if [ "$MONITOR_ALL" = true ]; then
    monitor_all
elif [ -n "$CONTAINER" ]; then
    monitor_container "$CONTAINER"
else
    # 列出可用容器
    echo ""
    echo -e "${BLUE}Available benchmark containers:${NC}"
    echo ""
    docker ps --filter "name=benchmark_" --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" 2>/dev/null

    if [ $? -ne 0 ] || [ -z "$(docker ps --filter 'name=benchmark_' --format '{{.Names}}' 2>/dev/null)" ]; then
        echo -e "${YELLOW}No benchmark containers running${NC}"
        echo ""
        echo "Start a container first:"
        echo "  ./run_benchmark.sh -p jetson_nano -i"
    fi
    echo ""
fi
