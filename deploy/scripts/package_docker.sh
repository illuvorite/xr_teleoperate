#!/bin/bash
# ============================================================
# Package Docker images into tar files for offline deployment
# 使用方法: bash scripts/package_docker.sh [output_dir]
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEPLOY_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_DIR="${1:-${DEPLOY_DIR}/dist}"

cd "$DEPLOY_DIR"

# 加载 .env
if [ -f .env ]; then
    source .env
else
    echo "ERROR: .env 文件不存在，请先 cp .env.example .env 并配置"
    exit 1
fi

DOCKER_REGISTRY="${DOCKER_REGISTRY:-docker.io}"
DOCKER_USERNAME="${DOCKER_USERNAME:-yourusername}"
IMAGE_TAG="${IMAGE_TAG:-latest}"

XR_TELEOPERATE_IMAGE="${DOCKER_REGISTRY}/${DOCKER_USERNAME}/xr-teleoperate:${IMAGE_TAG}"
TELEIMAGER_IMAGE="${DOCKER_REGISTRY}/${DOCKER_USERNAME}/teleimager:${IMAGE_TAG}"

mkdir -p "$OUTPUT_DIR"

echo "=========================================="
echo " 打包 Docker 镜像"
echo "=========================================="
echo "xr-teleoperate: ${XR_TELEOPERATE_IMAGE}"
echo "teleimager:     ${TELEIMAGER_IMAGE}"
echo "输出目录:       ${OUTPUT_DIR}"
echo "=========================================="

# 检查镜像是否存在
if ! docker image inspect "${XR_TELEOPERATE_IMAGE}" >/dev/null 2>&1; then
    echo "ERROR: 镜像 ${XR_TELEOPERATE_IMAGE} 不存在，请先构建"
    exit 1
fi

if ! docker image inspect "${TELEIMAGER_IMAGE}" >/dev/null 2>&1; then
    echo "ERROR: 镜像 ${TELEIMAGER_IMAGE} 不存在，请先构建"
    exit 1
fi

# 导出镜像
echo ""
echo "[1/4] 导出 xr-teleoperate 镜像..."
docker save "${XR_TELEOPERATE_IMAGE}" | gzip > "${OUTPUT_DIR}/xr-teleoperate-${IMAGE_TAG}.tar.gz"
echo "✓ 已保存: ${OUTPUT_DIR}/xr-teleoperate-${IMAGE_TAG}.tar.gz"

echo ""
echo "[2/4] 导出 teleimager 镜像..."
docker save "${TELEIMAGER_IMAGE}" | gzip > "${OUTPUT_DIR}/teleimager-${IMAGE_TAG}.tar.gz"
echo "✓ 已保存: ${OUTPUT_DIR}/teleimager-${IMAGE_TAG}.tar.gz"

# 计算校验和
echo ""
echo "[3/4] 计算校验和..."
cd "$OUTPUT_DIR"
sha256sum "xr-teleoperate-${IMAGE_TAG}.tar.gz" > "xr-teleoperate-${IMAGE_TAG}.tar.gz.sha256"
sha256sum "teleimager-${IMAGE_TAG}.tar.gz" > "teleimager-${IMAGE_TAG}.tar.gz.sha256"

# 打包 deploy 配置文件
echo ""
echo "[4/4] 打包部署配置文件..."
tar -czf "${OUTPUT_DIR}/deploy-config-${IMAGE_TAG}.tar.gz" -C "$DEPLOY_DIR" \
    docker-compose.remote.yml \
    .env.example \
    scripts/

# 创建总压缩包
echo ""
echo "创建完整离线部署包..."
cd "$OUTPUT_DIR"
tar -czf "../xr_teleoperate-offline-${IMAGE_TAG}.tar.gz" \
    "xr-teleoperate-${IMAGE_TAG}.tar.gz" \
    "teleimager-${IMAGE_TAG}.tar.gz" \
    "xr-teleoperate-${IMAGE_TAG}.tar.gz.sha256" \
    "teleimager-${IMAGE_TAG}.tar.gz.sha256" \
    "deploy-config-${IMAGE_TAG}.tar.gz"

echo ""
echo "=========================================="
echo " 打包完成！"
echo "=========================================="
echo ""
echo "离线部署包: ${OUTPUT_DIR}/../xr_teleoperate-offline-${IMAGE_TAG}.tar.gz"
echo ""
echo "文件清单:"
ls -lh "${OUTPUT_DIR}/"
echo ""
echo "目标机器部署步骤:"
echo "  1. 复制 xr_teleoperate-offline-${IMAGE_TAG}.tar.gz 到目标机器"
echo "  2. tar -xzf xr_teleoperate-offline-${IMAGE_TAG}.tar.gz"
echo "  3. docker load < xr-teleoperate-${IMAGE_TAG}.tar.gz"
echo "  4. docker load < teleimager-${IMAGE_TAG}.tar.gz"
echo "  5. cd deploy && cp .env.example .env && 编辑配置"
echo "  6. docker compose -f docker-compose.remote.yml up -d"
