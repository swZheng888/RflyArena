#!/bin/bash
#############################################################################
# 综合对比测试脚本
# 在不同资源配置下对比 PID/NMPC 控制器性能
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
PROJECT_DIR="$(dirname "$(dirname "$SCRIPT_DIR")")"

# 配置
PROFILES=("jetson_nano" "jetson_xavier" "rpi4" "unlimited")
CONTROLLERS=("pid" "nmpc")
TRAJECTORIES=("circle" "figure8" "ellipse")
SPEEDS=("1.0" "1.5")
DURATION=30
RESULTS_DIR="$PROJECT_DIR/benchmark_results/comparison_$(date +%Y%m%d_%H%M%S)"

# 帮助信息
show_help() {
    echo ""
    echo -e "${BLUE}Comprehensive Comparison Benchmark${NC}"
    echo ""
    echo "在不同资源配置下对比 PID/NMPC 控制器"
    echo ""
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "Options:"
    echo "  --profiles LIST    资源配置列表 (逗号分隔)"
    echo "  --controllers LIST 控制器列表 (逗号分隔)"
    echo "  --trajectories LIST 轨迹列表 (逗号分隔)"
    echo "  --speeds LIST      速度列表 (逗号分隔)"
    echo "  --duration SECS    测试时长 (默认: 30)"
    echo "  --quick            快速测试 (单轨迹，单速度)"
    echo "  -h, --help         显示帮助"
    echo ""
    echo "Examples:"
    echo "  $0                                    # 完整测试"
    echo "  $0 --quick                            # 快速测试"
    echo "  $0 --profiles jetson_nano,rpi4        # 指定配置"
    echo "  $0 --controllers nmpc --trajectories circle"
    echo ""
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --profiles)
            IFS=',' read -ra PROFILES <<< "$2"
            shift 2
            ;;
        --controllers)
            IFS=',' read -ra CONTROLLERS <<< "$2"
            shift 2
            ;;
        --trajectories)
            IFS=',' read -ra TRAJECTORIES <<< "$2"
            shift 2
            ;;
        --speeds)
            IFS=',' read -ra SPEEDS <<< "$2"
            shift 2
            ;;
        --duration)
            DURATION="$2"
            shift 2
            ;;
        --quick)
            TRAJECTORIES=("circle")
            SPEEDS=("1.0")
            shift
            ;;
        -h|--help)
            show_help
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# 创建结果目录
mkdir -p "$RESULTS_DIR"

# 计算总测试数
TOTAL_TESTS=$((${#PROFILES[@]} * ${#CONTROLLERS[@]} * ${#TRAJECTORIES[@]} * ${#SPEEDS[@]}))

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  Comprehensive Comparison Benchmark${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${GREEN}Profiles:${NC}      ${PROFILES[*]}"
echo -e "${GREEN}Controllers:${NC}   ${CONTROLLERS[*]}"
echo -e "${GREEN}Trajectories:${NC}  ${TRAJECTORIES[*]}"
echo -e "${GREEN}Speeds:${NC}        ${SPEEDS[*]}"
echo -e "${GREEN}Duration:${NC}      ${DURATION}s"
echo -e "${GREEN}Total Tests:${NC}   $TOTAL_TESTS"
echo -e "${GREEN}Results:${NC}       $RESULTS_DIR"
echo ""

# 确认
read -p "Start benchmark? [y/N] " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "Aborted"
    exit 0
fi

# 初始化结果 CSV
SUMMARY_FILE="$RESULTS_DIR/summary.csv"
echo "profile,controller,trajectory,speed,rms_error,max_error,avg_cpu,max_cpu,avg_mem_mb,max_mem_mb,status" > "$SUMMARY_FILE"

# 运行测试
TEST_NUM=0
FAILED=0

for profile in "${PROFILES[@]}"; do
    for controller in "${CONTROLLERS[@]}"; do
        for trajectory in "${TRAJECTORIES[@]}"; do
            for speed in "${SPEEDS[@]}"; do
                TEST_NUM=$((TEST_NUM + 1))
                TEST_NAME="${profile}_${controller}_${trajectory}_${speed}x"

                echo ""
                echo -e "${CYAN}[$TEST_NUM/$TOTAL_TESTS] $TEST_NAME${NC}"
                echo "----------------------------------------"

                # 创建测试目录
                TEST_DIR="$RESULTS_DIR/$TEST_NAME"
                mkdir -p "$TEST_DIR"

                # 运行测试
                START_TIME=$(date +%s)

                "$SCRIPT_DIR/run_benchmark.sh" \
                    -p "$profile" \
                    -c "$controller" \
                    -t "$trajectory" \
                    -s "$speed" \
                    -d "$DURATION" \
                    2>&1 | tee "$TEST_DIR/output.log"

                EXIT_CODE=${PIPESTATUS[0]}
                END_TIME=$(date +%s)
                ELAPSED=$((END_TIME - START_TIME))

                if [ $EXIT_CODE -eq 0 ]; then
                    echo -e "${GREEN}✓ Completed in ${ELAPSED}s${NC}"
                    STATUS="success"

                    # 解析结果 (假设结果文件存在)
                    # 这里需要根据实际输出格式解析
                    RMS_ERROR="N/A"
                    MAX_ERROR="N/A"
                    AVG_CPU="N/A"
                    MAX_CPU="N/A"
                    AVG_MEM="N/A"
                    MAX_MEM="N/A"

                else
                    echo -e "${RED}✗ Failed (exit code: $EXIT_CODE)${NC}"
                    STATUS="failed"
                    FAILED=$((FAILED + 1))
                    RMS_ERROR="N/A"
                    MAX_ERROR="N/A"
                    AVG_CPU="N/A"
                    MAX_CPU="N/A"
                    AVG_MEM="N/A"
                    MAX_MEM="N/A"
                fi

                # 记录结果
                echo "$profile,$controller,$trajectory,$speed,$RMS_ERROR,$MAX_ERROR,$AVG_CPU,$MAX_CPU,$AVG_MEM,$MAX_MEM,$STATUS" >> "$SUMMARY_FILE"

            done
        done
    done
done

# 打印总结
echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  Benchmark Complete${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${GREEN}Total Tests:${NC}  $TOTAL_TESTS"
echo -e "${GREEN}Successful:${NC}   $((TOTAL_TESTS - FAILED))"
echo -e "${RED}Failed:${NC}       $FAILED"
echo ""
echo -e "${GREEN}Results:${NC}      $RESULTS_DIR"
echo -e "${GREEN}Summary:${NC}      $SUMMARY_FILE"
echo ""

# 生成报告
echo "Generating report..."

cat > "$RESULTS_DIR/report.md" << EOF
# UAV Control Benchmark Report

**Date:** $(date '+%Y-%m-%d %H:%M:%S')

## Configuration

| Parameter | Value |
|-----------|-------|
| Profiles | ${PROFILES[*]} |
| Controllers | ${CONTROLLERS[*]} |
| Trajectories | ${TRAJECTORIES[*]} |
| Speeds | ${SPEEDS[*]} |
| Duration | ${DURATION}s |

## Summary

- Total Tests: $TOTAL_TESTS
- Successful: $((TOTAL_TESTS - FAILED))
- Failed: $FAILED

## Results

See \`summary.csv\` for detailed results.

## Analysis

_(To be filled after data analysis)_
EOF

echo -e "${GREEN}Report generated: $RESULTS_DIR/report.md${NC}"
