#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate tv
echo "[xr_teleop] activated conda env: ${CONDA_DEFAULT_ENV:-unknown}"

export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$REPO_ROOT"

# 图像服务使用 wlan0（XR 设备所在 Wi-Fi），DDS 仍使用 eth0（机器人网络）。
# 允许通过 IMG_SERVER_IP 手动覆盖自动检测结果。
detect_robot_ip() {
  local iface="${1:-eth0}"
  local ip=""

  if command -v ip &>/dev/null; then
    ip=$(ip -4 addr show "$iface" 2>/dev/null \
      | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n1 || true)
  fi

  if [ -z "$ip" ] && command -v ip &>/dev/null; then
    ip=$(ip route get 1.1.1.1 2>/dev/null | grep -oP '(?<=src\s)\S+' | head -n1 || true)
  fi

  if [ -z "$ip" ] && command -v hostname &>/dev/null; then
    ip=$(hostname -I 2>/dev/null | awk '{print $1}')
  fi

  echo "${ip:-192.168.2.203}"
}

IMG_SERVER_IP="${IMG_SERVER_IP:-$(detect_robot_ip wlan0)}"
echo "[xr_teleop] image server IP: $IMG_SERVER_IP"

# Use the binocular WebRTC plane as the complete Pico view by default. Set
# DISPLAY_MODE=ego or DISPLAY_MODE=pass-through when a mixed view is needed.
DISPLAY_MODE="${DISPLAY_MODE:-immersive}"
echo "[xr_teleop] XR display mode: $DISPLAY_MODE"
echo "[xr_teleop] Pico viewer URL: https://${IMG_SERVER_IP}:8012"

python teleop/teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode controller \
  --display-mode "$DISPLAY_MODE" \
  --network-interface eth0 \
  --img-server-ip "$IMG_SERVER_IP" \
  --motion

# 双目摄像头不兼容 static dashboard，因此不启用 --static-dashboard。
