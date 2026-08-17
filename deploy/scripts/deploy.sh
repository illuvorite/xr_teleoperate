#!/bin/bash
# ============================================================
# xr_teleoperate 一键部署脚本 (Linux/Bash)
# 使用方法: bash scripts/deploy.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$DEPLOY_DIR"

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

echo -e "${CYAN}==========================================${NC}"
echo -e "${CYAN} xr_teleoperate 部署脚本${NC}"
echo -e "${CYAN}==========================================${NC}"
echo ""

# 加载 .env
if [ -f .env ]; then
    source .env
else
    echo -e "${RED}ERROR: .env 文件不存在，请先 cp .env.example .env 并配置${NC}"
    exit 1
fi

# 检查 Docker
if ! command -v docker >/dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker 未安装${NC}"
    exit 1
fi

if ! docker info >/dev/null 2>&1; then
    echo -e "${RED}ERROR: Docker 未运行，请先启动 Docker${NC}"
    exit 1
fi

# 检查 NVIDIA Container Toolkit
echo -e "${YELLOW}[检查] NVIDIA Container Toolkit...${NC}"
if ! docker run --rm --gpus all nvidia/cuda:12.4.1-cudnn9-devel-ubuntu20.04 nvidia-smi >/dev/null 2>&1; then
    echo -e "${RED}ERROR: NVIDIA Container Toolkit 未正确配置${NC}"
    echo -e "${YELLOW}请参考: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html${NC}"
    exit 1
fi
echo -e "${GREEN}✓ NVIDIA Container Toolkit 正常${NC}"

# 拉取镜像
echo ""
echo -e "${YELLOW}[1/3] 拉取镜像...${NC}"
docker compose -f docker-compose.remote.yml pull

# 启动服务
echo ""
echo -e "${YELLOW}[2/3] 启动服务...${NC}"
docker compose -f docker-compose.remote.yml up -d

# 等待服务就绪
echo ""
echo -e "${YELLOW}[3/3] 等待服务就绪...${NC}"
sleep 10

# 验证
echo ""
echo -e "${CYAN}==========================================${NC}"
echo -e "${GREEN} 部署完成！${NC}"
echo -e "${CYAN}==========================================${NC}"
echo ""

# 显示服务状态
docker compose -f docker-compose.remote.yml ps

echo ""
echo -e "${YELLOW}常用命令:${NC}"
echo "  查看日志: docker compose -f docker-compose.remote.yml logs -f xr-teleoperate"
echo "  重启服务: docker compose -f docker-compose.remote.yml restart"
echo "  停止服务: docker compose -f docker-compose.remote.yml down"
echo "  健康检查: docker compose -f docker-compose.remote.yml exec xr-teleoperate bash /workspace/scripts/docker_healthcheck.sh"

echo ""
echo -e "${YELLOW}XR 设备访问地址:${NC}"
HOST_IP=$(docker compose -f docker-compose.remote.yml exec xr-teleoperate env 2>/dev/null | grep IMG_SERVER_IP | cut -d= -f2 || echo "自动检测中...")
echo -e "  ${CYAN}https://${HOST_IP}:8012/?ws=wss://${HOST_IP}:8012${NC}"
