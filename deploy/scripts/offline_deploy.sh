#!/bin/bash
# ============================================================
# Offline Deployment Script
# 从本地 tar 包加载镜像并启动（无需 Docker Hub）
# 使用方法: bash scripts/offline_deploy.sh [image_tar_dir]
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
IMAGE_DIR="${1:-${DEPLOY_DIR}/dist}"

cd "$DEPLOY_DIR"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}==========================================${NC}"
echo -e "${CYAN} xr_teleoperate 离线部署${NC}"
echo -e "${CYAN}==========================================${NC}"
echo ""

# 检查 Docker
if ! command -v docker >/dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker 未安装${NC}"
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker 未运行，请先启动 Docker${NC}"
    exit 1
fi

# 检查镜像文件
if [ ! -d "$IMAGE_DIR" ]; then
    echo -e "${RED}ERROR: 镜像目录不存在: ${IMAGE_DIR}${NC}"
    echo -e "${YELLOW}请先解压 xr_teleoperate-offline-*.tar.gz${NC}"
    exit 1
fi

# 查找镜像 tar 文件
XR_TAR=$(ls "${IMAGE_DIR}"/xr-teleoperate-*.tar.gz 2>/dev/null | head -n 1)
TELE_TAR=$(ls "${IMAGE_DIR}"/teleimager-*.tar.gz 2>/dev/null | head -n 1)

if [ -z "$XR_TAR" ] || [ ! -f "$XR_TAR" ]; then
    echo -e "${RED}ERROR: 未找到 xr-teleoperate 镜像文件${NC}"
    echo -e "${YELLOW}请在 ${IMAGE_DIR} 中放入 xr-teleoperate-*.tar.gz${NC}"
    exit 1
fi

if [ -z "$TELE_TAR" ] || [ ! -f "$TELE_TAR" ]; then
    echo -e "${RED}ERROR: 未找到 teleimager 镜像文件${NC}"
    echo -e "${YELLOW}请在 ${IMAGE_DIR} 中放入 teleimager-*.tar.gz${NC}"
    exit 1
fi

# 加载镜像
echo -e "${YELLOW}[1/4] 加载 Docker 镜像...${NC}"
echo "  加载: $(basename "$XR_TAR")"
docker load < "$XR_TAR"
echo "  加载: $(basename "$TELE_TAR")"
docker load < "$TELE_TAR"
echo -e "${GREEN}✓ 镜像加载完成${NC}"

# 检查配置文件
echo ""
echo -e "${YELLOW}[2/4] 检查配置文件...${NC}"
if [ ! -f .env ]; then
    echo -e "${YELLOW}  .env 不存在，从 .env.example 复制...${NC}"
    cp .env.example .env
    echo -e "${YELLOW}  请编辑 .env 文件，设置 IMG_SERVER_IP 等配置${NC}"
    echo -e "${YELLOW}  按 Enter 继续...${NC}"
    read -r
fi

# 创建必要目录
echo ""
echo -e "${YELLOW}[3/4] 创建必要目录...${NC}"
mkdir -p ../data ../guidelogs
echo -e "${GREEN}✓ 目录创建完成${NC}"

# 启动服务
echo ""
echo -e "${YELLOW}[4/4] 启动服务...${NC}"
docker compose -f docker-compose.remote.yml up -d

# 等待服务就绪
echo ""
echo -e "${YELLOW}等待服务就绪...${NC}"
sleep 10

# 验证
echo ""
echo -e "${CYAN}==========================================${NC}"
echo -e "${GREEN} 部署完成！${NC}"
echo -e "${CYAN}==========================================${NC}"
echo ""

docker compose -f docker-compose.remote.yml ps

echo ""
echo -e "${YELLOW}常用命令:${NC}"
echo "  查看日志: docker compose -f docker-compose.remote.yml logs -f xr-teleoperate"
echo "  重启服务: docker compose -f docker-compose.remote.yml restart"
echo "  停止服务: docker compose -f docker-compose.remote.yml down"
echo "  健康检查: docker compose -f docker-compose.remote.yml exec xr-teleoperate bash /workspace/scripts/docker_healthcheck.sh"
