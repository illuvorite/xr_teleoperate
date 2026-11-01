#!/bin/bash
# scripts/postinstall_env.sh — 在 /opt/xr-teleoperate-main 装 conda env "tv"
# 由 xr-teleop-control install-env 调用,不直接由 deb postinst 跑(避免阻塞 dpkg 安装)。
set -e

PROJECT_ROOT="/opt/xr-teleoperate-main"
ENV_FILE="${PROJECT_ROOT}/.deb-build/tv-env.yml"

# 1. 检查用户(conda 装在 unitree 用户家目录)
TARGET_USER="unitree"
TARGET_HOME="/home/unitree"
if [ "$(whoami)" != "$TARGET_USER" ] && [ "$EUID" -eq 0 ]; then
  # root 调 su -c
  exec su - "$TARGET_USER" -c "$0 \"\$@\" --as-user" -- "$@"
fi
if [ "$(whoami)" != "$TARGET_USER" ]; then
  echo "[install-env] FATAL: must run as $TARGET_USER (or via sudo -u $TARGET_USER xr-teleop-control install-env)" >&2
  exit 1
fi

echo "[install-env] Target user: $(whoami) HOME=$HOME"

# 2. conda 检查
if [ ! -x "$HOME/miniconda3/bin/conda" ]; then
  echo "[install-env] Miniconda not found, installing..."
  MINIFORGE_INSTALLER="$HOME/miniforge.sh"
  if [ ! -f "$MINIFORGE_INSTALLER" ]; then
    echo "[install-env] Downloading Miniforge (CondaForge) for aarch64..."
    curl -fsSL -o "$MINIFORGE_INSTALLER" \
      "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh"
  fi
  bash "$MINIFORGE_INSTALLER" -b -p "$HOME/miniconda3"
  rm -f "$MINIFORGE_INSTALLER"
fi

# shellcheck disable=SC1091
source "$HOME/miniconda3/etc/profile.d/conda.sh"
conda activate base >/dev/null 2>&1 || true

# 3. 检查 tv env
if conda env list | awk '{print $1}' | grep -qx "tv"; then
  echo "[install-env] conda env 'tv' already exists. Verify python imports..."
  conda activate tv
  python -c "import vuer, aiohttp; print('tv env OK, vuer:', vuer.__file__)" \
    || { echo "[install-env] tv env broken, recreating..."; conda env remove -n tv -y; }
fi

# 4. 创建/重建 tv env(从 yml)
if ! conda env list | awk '{print $1}' | grep -qx "tv"; then
  echo "[install-env] Creating conda env 'tv' from $ENV_FILE..."
  echo "[install-env] (这一步会下载 3GB+ 的依赖包,可能耗时 10+ 分钟)"
  conda env create -f "$ENV_FILE"
fi

# 5. 激活并验证
conda activate tv
python -c "import vuer, aiohttp, numpy; print('[install-env] tv env OK; vuer:', vuer.__file__)" \
  || { echo "[install-env] FATAL: tv env still broken"; exit 2; }

# 6. 预热 _patched_client_root(把 vuer 客户端补丁缓存建好)
#    注意:该缓存以 _PATCHED_CLIENT_MARKER 作为版本标记。改动了任何注入脚本
#    (关网格 / 隐藏菜单 / auto-orbit / 电量上报)之后必须提升该 marker,
#    并删掉旧缓存目录,否则会继续复用旧页面,新脚本不生效。
echo "[install-env] Pre-building vuer client patch cache..."
python -c "from teleop.televuer.src.televuer.televuer import _patched_client_root; \
  print('client_patch:', _patched_client_root())" \
  || true

# 7. 标记完成
mkdir -p "$HOME/.cache/xr_teleoperate"
touch "$HOME/.cache/xr_teleoperate/postinstall.ok"

echo "[install-env] DONE. Now: xr-teleop-control start"