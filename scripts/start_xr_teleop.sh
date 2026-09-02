#!/usr/bin/env bash
# start_xr_teleop.sh
# 启动 XR 遥操主服务。
#
# 设计要点(IP 动态化):
#   * 默认对外网网卡是 wlan0(PICO 头显 / 前端浏览器所在的 Wi-Fi 网段)。
#   * eth0 仍用于 ROS / DDS 控制通道,与外部 Wi-Fi 解耦。
#   * 所有 IP 都不写死,通过 XR_TELEOP_IFACE / XR_TELEOP_BACKEND_URL 等环境变量
#     注入或由自动检测得出。
#   * 启动后调用 heartbeat_register.py,把 {robotId, wlan0Ip, eth0Ip} 推送到
#     autobot-guide-service 后端(/api/v1/teleop/hosts/register)。
#
# 环境变量:
#   XR_TELEOP_IFACE         对外网卡,默认 wlan0
#   IMG_SERVER_IP           覆盖自动检测结果(向后兼容)
#   XR_TELEOP_ROBOT_ID      机器人 ID(必填,用于上报后端)
#   XR_TELEOP_BACKEND_URL   后端地址,默认 http://127.0.0.1:19269
#   XR_TELEOP_PUBLIC_HOST   VR 对外域名/IP(覆盖自动检测)
#   XR_TELEOP_PUBLIC_PORT   VR 对外端口,默认 8012
#   XR_TELEOP_PUBLIC_URL    VR 对外完整 URL(优先于 HOST/PORT)
#   XR_TELEOP_ROBOT_TOKEN   后端 Sa-Token 机器人鉴权 token
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# 激活 conda 'tv' 环境。优先用 conda 命令;若 PATH 中没有 conda(SSH non-login 环境),
# 则直接用常见的安装路径 /home/unitree/miniconda3;用户可通过 XR_TELEOP_CONDA_HOME 覆盖。
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
  conda activate tv
  echo "[xr_teleop] activated conda env: ${CONDA_DEFAULT_ENV:-unknown} (from $CONDA_HOME)"
else
  echo "[xr_teleop] WARN: conda 'tv' env not found (CONDA_HOME='${CONDA_HOME:-<unset>}'); continuing without conda activation" >&2
fi

export PYTHONPATH="$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$REPO_ROOT"

# ---------------------------------------------------------------------------
# 1. 自动检测本机网卡 IP(wlan0 优先;eth0 留作 DDS)
# ---------------------------------------------------------------------------
detect_first_ipv4() {
  local iface="$1"
  ip -4 addr show "$iface" 2>/dev/null \
    | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | head -n1 || true
}

detect_default_route_ip() {
  ip route get 1.1.1.1 2>/dev/null | grep -oP '(?<=src\s)\S+' | head -n1 || true
}

detect_hostname_i() {
  hostname -I 2>/dev/null | awk '{print $1}' || true
}

# 对外网卡(wlan0 优先,失败则按 en*/eth*/wlan* 顺序回退,最后走路由/hostname)
XR_TELEOP_IFACE="${XR_TELEOP_IFACE:-wlan0}"
WAN_IP=""
for candidate in "$XR_TELEOP_IFACE" enP* eth0 wlan0; do
  ip_candidate="$(detect_first_ipv4 "$candidate" 2>/dev/null || true)"
  if [ -n "$ip_candidate" ]; then
    WAN_IP="$ip_candidate"
    XR_TELEOP_IFACE="$candidate"
    break
  fi
done
if [ -z "$WAN_IP" ]; then
  WAN_IP="$(detect_default_route_ip || true)"
fi
if [ -z "$WAN_IP" ]; then
  WAN_IP="$(detect_hostname_i || true)"
fi

# eth0 仅作 ROS / DDS 用,尽力取一次;不影响主流程。
ETH0_IP="$(detect_first_ipv4 eth0 || true)"

# IMG_SERVER_IP 用于图像服务对外地址,默认等于 WAN_IP,可通过环境变量覆盖。
IMG_SERVER_IP="${IMG_SERVER_IP:-$WAN_IP}"
WAN_IP="${WAN_IP:-127.0.0.1}"
ETH0_IP="${ETH0_IP:-}"

if [ -z "${XR_TELEOP_ROBOT_ID:-}" ]; then
  echo "[xr_teleop] WARN: XR_TELEOP_ROBOT_ID not set, using default 'g1_001'." >&2
  XR_TELEOP_ROBOT_ID="g1_001"
fi

echo "[xr_teleop] detected WAN interface : ${XR_TELEOP_IFACE}"
echo "[xr_teleop] WAN IP (VR served from): ${WAN_IP}"
[ -n "$ETH0_IP" ] && echo "[xr_teleop] eth0 IP (ROS/DDS only) : ${ETH0_IP}"
echo "[xr_teleop] image server IP        : ${IMG_SERVER_IP}"
echo "[xr_teleop] robot id              : ${XR_TELEOP_ROBOT_ID}"

# 把检测到的 IP 注入到 TeleVuer / teleimager 的环境变量。
export XR_TELEOP_PUBLIC_HOST="${XR_TELEOP_PUBLIC_HOST:-$WAN_IP}"
export XR_TELEOP_PUBLIC_PORT="${XR_TELEOP_PUBLIC_PORT:-8012}"
export DDS_INTERFACE="${DDS_INTERFACE:-eth0}"

# ---------------------------------------------------------------------------
# 2. 启动后台心跳脚本,把当前 IP 注册到 autobot-guide-service 后端。
#    心跳独立于主进程:主进程崩了心跳还在跑,直到 SIGTERM 才退出。
# ---------------------------------------------------------------------------
HEARTBEAT_SCRIPT="$SCRIPT_DIR/heartbeat_register.py"
if [ -x "$HEARTBEAT_SCRIPT" ] || [ -f "$HEARTBEAT_SCRIPT" ]; then
  export XR_TELEOP_WAN_IP="$WAN_IP"
  export XR_TELEOP_ETH0_IP="$ETH0_IP"
  echo "[xr_teleop] launching heartbeat to backend: ${XR_TELEOP_BACKEND_URL:-http://127.0.0.1:19269}"
  python "$HEARTBEAT_SCRIPT" &
  HEARTBEAT_PID=$!
  echo "[xr_teleop] heartbeat pid: ${HEARTBEAT_PID}"
  # 主进程退出时一并杀掉心跳
  trap 'echo "[xr_teleop] stopping heartbeat (pid=${HEARTBEAT_PID})"; kill "${HEARTBEAT_PID}" 2>/dev/null || true; wait "${HEARTBEAT_PID}" 2>/dev/null || true' EXIT INT TERM
else
  echo "[xr_teleop] WARN: heartbeat_register.py not found, skipping backend registration" >&2
fi

# ---------------------------------------------------------------------------
# 3. 启动主进程
# ---------------------------------------------------------------------------
DISPLAY_MODE="${DISPLAY_MODE:-immersive}"
echo "[xr_teleop] XR display mode: $DISPLAY_MODE"
echo "[xr_teleop] Pico viewer URL: https://${WAN_IP}:${XR_TELEOP_PUBLIC_PORT}"

python teleop/teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode controller \
  --display-mode "$DISPLAY_MODE" \
  --network-interface "${DDS_INTERFACE}" \
  --img-server-ip "$IMG_SERVER_IP" \
  --motion

# 双目摄像头不兼容 static dashboard,因此不启用 --static-dashboard。