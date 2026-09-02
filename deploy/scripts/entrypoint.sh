#!/bin/bash
set -euo pipefail

echo "=========================================="
echo " xr_teleoperate Docker Entrypoint"
echo "=========================================="

# 容器内 / 系统 SSH non-login 环境下 conda 不一定在 PATH;
# 优先 conda 命令,找不到则尝试常见路径 /home/unitree/miniconda3。
CONDA_HOME="${XR_TELEOP_CONDA_HOME:-}"
if [ -z "$CONDA_HOME" ] && command -v conda >/dev/null 2>&1; then
  CONDA_HOME="$(conda info --base)"
fi
if [ -z "$CONDA_HOME" ] && [ -f "/home/unitree/miniconda3/etc/profile.d/conda.sh" ]; then
  CONDA_HOME="/home/unitree/miniconda3"
fi
if [ -n "$CONDA_HOME" ] && [ -f "$CONDA_HOME/etc/profile.d/conda.sh" ]; then
  # shellcheck disable=SC1090
  source "$CONDA_HOME/etc/profile.d/conda.sh"
  echo "[entrypoint] conda loaded from $CONDA_HOME"
fi

# ---- Auto-detect host IP ----
# VR 对外网卡默认 wlan0(PICO / 前端浏览器所在 Wi-Fi);
# eth0 仅供 ROS / DDS,与外部网络解耦。
# 任何环境变量缺失时,这里与 start_xr_teleop.sh 走同样的回退顺序。
if [ -z "${IMG_SERVER_IP:-}" ]; then
    DETECTED_IP=$(python -c "
from teleop.utils.host_info import detect_ips
print(detect_ips()['wan_ip'])
" 2>/dev/null || true)
    if [ -n "$DETECTED_IP" ]; then
        echo "[entrypoint] Auto-detected host IP: ${DETECTED_IP}"
        export IMG_SERVER_IP="$DETECTED_IP"
    else
        echo "[entrypoint] WARNING: Could not auto-detect IP, using fallback 127.0.0.1"
        export IMG_SERVER_IP="127.0.0.1"
    fi
else
    echo "[entrypoint] Using configured IMG_SERVER_IP: ${IMG_SERVER_IP}"
fi

# 同步导出对外 host / port,XR_TELEOP_PUBLIC_HOST 若未显式设置则沿用检测结果。
export XR_TELEOP_PUBLIC_HOST="${XR_TELEOP_PUBLIC_HOST:-$IMG_SERVER_IP}"
export XR_TELEOP_PUBLIC_PORT="${XR_TELEOP_PUBLIC_PORT:-8012}"

# 把 eth0 注入给 DDS_INTERFACE,仅当用户没显式覆盖时。
if [ -z "${DDS_INTERFACE:-}" ]; then
    export DDS_INTERFACE="eth0"
fi

# 确保证书软链存在(项目内置 certs)
mkdir -p /root/.config/xr_teleoperate
if [ -f /workspace/certs/cert.pem ] && [ -f /workspace/certs/key.pem ]; then
    ln -sf /workspace/certs/cert.pem /root/.config/xr_teleoperate/cert.pem
    ln -sf /workspace/certs/key.pem /root/.config/xr_teleoperate/key.pem
    echo "[entrypoint] Certificates linked to ~/.config/xr_teleoperate/"
else
    echo "[entrypoint] WARNING: /workspace/certs/ missing; relying on module fallback"
fi

# 确保运行时目录存在
mkdir -p /workspace/data /workspace/guidelogs

# 显示 GPU 状态
if command -v nvidia-smi >/dev/null 2>&1; then
    echo "[entrypoint] GPU detected:"
    nvidia-smi -L || true
else
    echo "[entrypoint] WARNING: nvidia-smi not found; GPU may not be available"
fi

echo "[entrypoint] XR_TELEOP_CERT=${XR_TELEOP_CERT:-<not set>}"
echo "[entrypoint] XR_TELEOP_KEY=${XR_TELEOP_KEY:-<not set>}"
echo "[entrypoint] XR_TELEOP_PUBLIC_HOST=${XR_TELEOP_PUBLIC_HOST}"
echo "[entrypoint] XR_TELEOP_PUBLIC_PORT=${XR_TELEOP_PUBLIC_PORT}"
echo "[entrypoint] IMG_SERVER_IP=${IMG_SERVER_IP}"
echo "[entrypoint] DDS_INTERFACE=${DDS_INTERFACE}"

echo "=========================================="
echo " Starting xr_teleoperate..."
echo "=========================================="

# 执行容器 CMD
exec "$@"