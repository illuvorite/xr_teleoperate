#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

conda activate tv
echo "[xr_teleop] activated conda env: $(conda info --envs | grep '\*' | awk '{print $1}')"

cd "$REPO_ROOT"

# 自动检测机器人 IP，允许通过环境变量覆盖
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

IMG_SERVER_IP="${IMG_SERVER_IP:-$(detect_robot_ip eth0)}"
echo "[xr_teleop] 机器人 IP: $IMG_SERVER_IP"

python teleop/teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode controller \
  --display-mode immersive \
  --network-interface eth0 \
  --img-server-ip "$IMG_SERVER_IP" \
  --motion \
  --static-dashboard
