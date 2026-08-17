#!/bin/bash
# ============================================================
# xr_teleoperate Build & Push Script
# 构建镜像并推送到 Docker Hub
# 使用方法: bash scripts/push.sh
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"

cd "$DEPLOY_DIR"

# 加载 .env
if [ -f .env ]; then
    source .env
else
    echo "ERROR: .env 文件不存在，请先 cp .env.example .env 并配置"
    exit 1
fi

# 检查必填变量
if [ -z "${DOCKER_USERNAME:-}" ]; then
    echo "ERROR: DOCKER_USERNAME 未设置"
    exit 1
fi

if [ -z "${IMAGE_TAG:-}" ]; then
    IMAGE_TAG="latest"
fi

# 设置镜像名称
DOCKER_REGISTRY="${DOCKER_REGISTRY:-docker.io}"
XR_TELEOPERATE_IMAGE="${DOCKER_REGISTRY}/${DOCKER_USERNAME}/xr-teleoperate:${IMAGE_TAG}"
TELEIMAGER_IMAGE="${DOCKER_REGISTRY}/${DOCKER_USERNAME}/teleimager:${IMAGE_TAG}"

echo "=========================================="
echo " 构建并推送镜像到 Docker Hub"
echo "=========================================="
echo "xr-teleoperate: ${XR_TELEOPERATE_IMAGE}"
echo "teleimager:     ${TELEIMAGER_IMAGE}"
echo "=========================================="

# 检查 Docker 是否运行
if ! docker info >/dev/null 2>&1; then
    echo "ERROR: Docker 未运行，请先启动 Docker"
    exit 1
fi

# 检查是否已登录 Docker Hub
if ! docker info | grep -q "Username: ${DOCKER_USERNAME}"; then
    echo "INFO: 未检测到 Docker Hub 登录，请先执行: docker login"
    exit 1
fi

# 构建镜像
echo ""
echo "[1/5] 构建镜像..."
docker compose build

# 标记镜像
echo ""
echo "[2/5] 标记镜像..."
docker tag xr-teleoperate:latest "${XR_TELEOPERATE_IMAGE}"
docker tag teleimager:latest "${TELEIMAGER_IMAGE}"

# 推送镜像
echo ""
echo "[3/5] 推送 xr-teleoperate..."
docker push "${XR_TELEOPERATE_IMAGE}"

echo ""
echo "[4/5] 推送 teleimager..."
docker push "${TELEIMAGER_IMAGE}"

# 验证
echo ""
echo "[5/5] 验证..."
docker manifest inspect "${XR_TELEOPERATE_IMAGE}" > /dev/null 2>&1 && echo "✓ xr-teleoperate 推送成功" || echo "✗ xr-teleoperate 推送失败"
docker manifest inspect "${TELEIMAGER_IMAGE}" > /dev/null 2>&1 && echo "✓ teleimager 推送成功" || echo "✗ teleimager 推送失败"

echo ""
echo "=========================================="
echo " 推送完成！"
echo "=========================================="
echo ""
echo "其他机器部署命令:"
echo "  git clone --depth 1 https://github.com/illuvorite/xr_teleoperate.git"
echo "  cd xr_teleoperate/deploy"
echo "  cp .env.example .env"
echo "  docker compose -f docker-compose.remote.yml pull"
echo "  docker compose -f docker-compose.remote.yml up -d"
