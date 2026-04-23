#!/bin/bash
#############################################################################
# Docker 镜像构建脚本
# 构建 UAV Control Benchmark 标准化环境
#############################################################################

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

# 脚本目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"
DOCKER_DIR="$PROJECT_DIR/docker"

# 默认值
IMAGE_NAME="uav-benchmark"
IMAGE_TAG="latest"
NO_CACHE=false
PUSH=false
REGISTRY=""

# 帮助信息
show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo ""
    echo "构建 UAV Control Benchmark Docker 镜像"
    echo ""
    echo "Options:"
    echo "  -n, --name NAME       镜像名称 (默认: uav-benchmark)"
    echo "  -t, --tag TAG         镜像标签 (默认: latest)"
    echo "  -p, --profile PROFILE 资源配置文件 (jetson_nano|jetson_xavier|rpi4|fmu)"
    echo "  --no-cache            不使用缓存构建"
    echo "  --push                构建后推送到仓库"
    echo "  -r, --registry URL    Docker 仓库地址"
    echo "  -h, --help            显示帮助信息"
    echo ""
    echo "Examples:"
    echo "  $0                           # 构建默认镜像"
    echo "  $0 -t v1.0.0                 # 构建指定版本"
    echo "  $0 -p jetson_nano            # 为 Jetson Nano 构建"
    echo "  $0 --push -r myregistry.com  # 构建并推送"
}

# 解析参数
while [[ $# -gt 0 ]]; do
    case $1 in
        -n|--name)
            IMAGE_NAME="$2"
            shift 2
            ;;
        -t|--tag)
            IMAGE_TAG="$2"
            shift 2
            ;;
        -p|--profile)
            PROFILE="$2"
            shift 2
            ;;
        --no-cache)
            NO_CACHE=true
            shift
            ;;
        --push)
            PUSH=true
            shift
            ;;
        -r|--registry)
            REGISTRY="$2/"
            shift 2
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

# 完整镜像名
FULL_IMAGE="${REGISTRY}${IMAGE_NAME}:${IMAGE_TAG}"

echo ""
echo -e "${BLUE}============================================${NC}"
echo -e "${BLUE}  UAV Control Benchmark - Docker Build${NC}"
echo -e "${BLUE}============================================${NC}"
echo ""
echo -e "${GREEN}Project Directory:${NC} $PROJECT_DIR"
echo -e "${GREEN}Image Name:${NC} $FULL_IMAGE"
echo -e "${GREEN}No Cache:${NC} $NO_CACHE"
echo ""

# 检查 Docker
if ! command -v docker &> /dev/null; then
    echo -e "${RED}Error: Docker is not installed${NC}"
    exit 1
fi

# 构建参数
BUILD_ARGS=""
if [ "$NO_CACHE" = true ]; then
    BUILD_ARGS="--no-cache"
fi

# 开始构建
echo -e "${YELLOW}Starting build...${NC}"
echo ""

cd "$PROJECT_DIR"

docker build \
    $BUILD_ARGS \
    -f docker/Dockerfile \
    -t "$FULL_IMAGE" \
    .

BUILD_STATUS=$?

if [ $BUILD_STATUS -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓ Build successful: $FULL_IMAGE${NC}"
    echo ""

    # 显示镜像信息
    docker images "$IMAGE_NAME:$IMAGE_TAG" --format "table {{.Repository}}\t{{.Tag}}\t{{.Size}}\t{{.CreatedAt}}"

    # 推送到仓库
    if [ "$PUSH" = true ]; then
        echo ""
        echo -e "${YELLOW}Pushing to registry...${NC}"
        docker push "$FULL_IMAGE"
        echo -e "${GREEN}✓ Push successful${NC}"
    fi

    # 构建不同配置的镜像
    if [ -n "$PROFILE" ]; then
        PROFILE_IMAGE="${REGISTRY}${IMAGE_NAME}:${PROFILE}"
        echo ""
        echo -e "${YELLOW}Tagging for profile: $PROFILE${NC}"
        docker tag "$FULL_IMAGE" "$PROFILE_IMAGE"
        echo -e "${GREEN}✓ Tagged: $PROFILE_IMAGE${NC}"
    fi

else
    echo ""
    echo -e "${RED}✗ Build failed${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Next steps:${NC}"
echo "  1. 启动容器: ./docker/scripts/run_benchmark.sh"
echo "  2. 运行测试: docker-compose up benchmark_jetson_nano"
echo ""
