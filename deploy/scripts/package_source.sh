#!/bin/bash
# ============================================================
# Package source code for offline deployment
# 使用方法: bash scripts/package_source.sh [output_dir]
# ============================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
OUTPUT_DIR="${1:-${PROJECT_ROOT}/dist}"

cd "$PROJECT_ROOT"

mkdir -p "$OUTPUT_DIR"

echo "=========================================="
echo " 打包 xr_teleoperate 源码"
echo "=========================================="
echo ""

# 1. 初始化子模块（确保代码完整）
echo "[1/5] 初始化 Git 子模块..."
git submodule update --init --depth 1

# 2. 创建源码压缩包（排除 .git、__pycache__、.kilo 等）
echo "[2/5] 创建源码压缩包..."
tar -czf "${OUTPUT_DIR}/xr_teleoperate-source.tar.gz" \
    --exclude='.git' \
    --exclude='.gitmodules' \
    --exclude='.kilo' \
    --exclude='__pycache__' \
    --exclude='*.pyc' \
    --exclude='*.log' \
    --exclude='data/' \
    --exclude='guidelogs/' \
    --exclude='.env' \
    --exclude='dist/' \
    --exclude='node_modules' \
    .

# 3. 创建依赖包列表
echo "[3/5] 生成依赖包列表..."
pip freeze > "${OUTPUT_DIR}/requirements-frozen.txt"
echo "✓ 已生成: ${OUTPUT_DIR}/requirements-frozen.txt"

# 4. 创建离线安装脚本
echo "[4/5] 生成离线安装脚本..."
cat > "${OUTPUT_DIR}/install.sh" << 'INSTALL_EOF'
#!/bin/bash
set -euo pipefail

echo "=========================================="
echo " xr_teleoperate 离线安装脚本"
echo "=========================================="
echo ""

# 检查 Python
if ! command -v python3.8 >/dev/null 2>&1; then
    echo "ERROR: Python 3.8 未安装"
    echo "Ubuntu 20.04 请执行: sudo apt install python3.8 python3.8-dev python3.8-venv"
    exit 1
fi

# 创建虚拟环境
echo "[1/5] 创建 Python 虚拟环境..."
python3.8 -m venv venv
source venv/bin/activate

# 升级 pip
echo "[2/5] 升级 pip..."
pip install --no-cache-dir --upgrade pip setuptools wheel

# 安装依赖
echo "[3/5] 安装 Python 依赖..."
if [ -f requirements-frozen.txt ]; then
    pip install --no-cache-dir -r requirements-frozen.txt
else
    pip install --no-cache-dir -r requirements.txt
fi

# 安装子模块
echo "[4/5] 安装子模块..."
pip install --no-cache-dir -e teleop/teleimager --no-deps
pip install --no-cache-dir -e teleop/televuer
pip install --no-cache-dir -e teleop/robot_control/dex-retargeting --no-deps

# 安装 unitree_sdk2_python
echo "[5/5] 安装 unitree_sdk2_python..."
git clone --depth 1 https://github.com/unitreerobotics/unitree_sdk2_python.git /tmp/unitree_sdk2
pip install --no-cache-dir -e /tmp/unitree_sdk2
rm -rf /tmp/unitree_sdk2

# 创建目录
mkdir -p data guidelogs certs

echo ""
echo "=========================================="
echo " 安装完成！"
echo "=========================================="
echo ""
echo "下一步:"
echo "  1. 配置证书: 将 cert.pem/key.pem 放入 certs/ 目录"
echo "  2. 配置环境: cp .env.example .env && 编辑配置"
echo "  3. 启动程序: source venv/bin/activate && python teleop/teleop_hand_and_arm.py"
INSTALL_EOF
chmod +x "${OUTPUT_DIR}/install.sh"

# 5. 创建 README
echo "[5/5] 生成部署说明..."
cat > "${OUTPUT_DIR}/OFFLINE_DEPLOY.md" << 'EOF'
# 离线部署说明

本目录包含 xr_teleoperate 的离线部署文件。

## 文件说明

```
xr_teleoperate-source.tar.gz    # 完整源码
requirements-frozen.txt         # Python 依赖（精确版本）
install.sh                      # 自动安装脚本
OFFLINE_DEPLOY.md               # 本文件
```

## 目标机器要求

- **OS**: Ubuntu 20.04
- **Python**: 3.8
- **CUDA**: 12.x (可选，用于 GPU 加速)
- **Git**: 用于克隆 unitree_sdk2_python

## 部署步骤

```bash
# 1. 解压源码
tar -xzf xr_teleoperate-source.tar.gz
cd xr_teleoperate

# 2. 运行安装脚本
bash dist/install.sh

# 3. 配置证书
cp teleop/televuer/cert.pem certs/
cp teleop/televuer/key.pem certs/

# 4. 配置环境
cp .env.example .env
nano .env  # 设置 IMG_SERVER_IP 等

# 5. 激活虚拟环境并启动
source venv/bin/activate
python teleop/teleop_hand_and_arm.py \
    --arm G1_29 \
    --input-mode controller \
    --display-mode immersive \
    --network-interface eth0 \
    --motion \
    --static-dashboard
```

## 系统依赖（需提前安装）

```bash
sudo apt update
sudo apt install -y \
    git git-lfs curl wget ca-certificates \
    build-essential cmake pkg-config \
    python3.8 python3.8-dev python3.8-venv \
    libopencv-dev libgl1-mesa-glx libglu1-mesa \
    libzmq3-dev libyaml-dev libboost-all-dev libspdlog-dev \
    libusb-1.0-0-dev libudev-dev
```
EOF

echo ""
echo "=========================================="
echo " 打包完成！"
echo "=========================================="
echo ""
echo "输出文件:"
ls -lh "${OUTPUT_DIR}/"
echo ""
echo "部署步骤:"
echo "  1. 复制整个 dist/ 目录到目标机器"
echo "  2. 在目标机器上执行: bash dist/install.sh"
echo "  3. 配置 .env 和证书"
echo "  4. 运行启动命令"
