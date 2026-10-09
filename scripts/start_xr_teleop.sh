#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

source "$(conda info --base)/etc/profile.d/conda.sh"
conda activate tv
echo "[xr_teleop] activated conda env: ${CONDA_DEFAULT_ENV:-unknown}"

export PYTHONPATH="$REPO_ROOT/teleop/televuer/src:$REPO_ROOT/teleop/teleimager/src:$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
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

# ---------------------------------------------------------------------------
# dex1-1 夹爪服务（随遥操一起拉起，刻意不做 systemd 常驻单元）
# ---------------------------------------------------------------------------
# 为什么必须起：主程序带 --ee dex1，启动时会阻塞等待 rt/dex1/{left,right}/state
# 有发布者（unitree_sdk2py 的 ChannelSubscriber.Read() 默认 timeout=None，即无限阻塞）。
# 没有这个服务就会一直刷 "Waiting to subscribe dds..."，遥操永远进不到主循环。
#
# 为什么要先判重：两个实例同时 open /dev/ttyUSB1、/dev/ttyUSB2 会互相抢串口，
# 表现为单侧电机 "does not reply"、state 话题频率减半、夹爪不动或乱动。
# 所以先查有没有在跑的，有就直接复用，绝不起第二个。
DEX1_DIR="$REPO_ROOT/third_party/dex1_1_service"
DEX1_BIN="$DEX1_DIR/bin/dex1_1_gripper_server"
DEX1_LOG="${TMPDIR:-/tmp}/xr_dex1_gripper.log"
DEX1_NET_IF="${DEX1_NET_IF:-eth0}"

start_dex1_gripper() {
  if [ ! -x "$DEX1_BIN" ]; then
    echo "[xr_teleop] 错误：找不到夹爪服务 $DEX1_BIN" >&2
    return 1
  fi

  # 方括号写法：避免 pgrep 匹配到本脚本自身的命令行
  if pgrep -f "[d]ex1_1_gripper_server" >/dev/null 2>&1; then
    echo "[xr_teleop] dex1 夹爪服务已在运行，复用（不再起第二个实例）"
    return 0
  fi

  # 包内自带 libserialport.so.0 与 libUnitreeMotorSDK_Arm64.so，靠 LD_LIBRARY_PATH 加载，
  # 这样新机器人不需要额外 apt 安装任何依赖。
  export LD_LIBRARY_PATH="$DEX1_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

  # 起服务前先清一次电机锁存故障：
  # M4010 堵转/过流后会锁存 MError=0x200（绕组过热位），此后电机拒绝进入 FOC，
  # 表现为「夹爪完全不动，但服务/DDS/话题一切正常」，极难排查。
  # 该错误位断电不一定清得掉，status=7 复位指令实测能清（0x200 -> 0x0）。
  echo "[xr_teleop] 清除夹爪电机故障（status=7 复位）..."
  if ! "$DEX1_BIN" --reset 2>&1 | sed 's/^/[xr_teleop][reset] /'; then
    echo "[xr_teleop] 警告：清故障失败，继续尝试启动服务" >&2
  fi

  echo "[xr_teleop] 启动 dex1 夹爪服务..."
  setsid nohup "$DEX1_BIN" --network "$DEX1_NET_IF" > "$DEX1_LOG" 2>&1 < /dev/null &

  # 等 DDS 发布器建起来（日志出现 "Dex1-1 Gripper Server started."）
  local i
  for i in $(seq 1 30); do
    if grep -q "Gripper Server started" "$DEX1_LOG" 2>/dev/null; then
      echo "[xr_teleop] dex1 夹爪服务已就绪 (pid $(pgrep -f '[d]ex1_1_gripper_server' | head -n1))"
      echo "[xr_teleop] 日志 $DEX1_LOG ；需要释放串口时: sudo pkill -f dex1_1_gripper_server"
      return 0
    fi
    sleep 0.5
  done

  echo "[xr_teleop] 错误：dex1 夹爪服务 15 秒内未就绪，遥操会卡在 'Waiting to subscribe dds...'" >&2
  echo "[xr_teleop]       最近日志：" >&2
  tail -n 20 "$DEX1_LOG" 2>/dev/null | sed 's/^/[xr_teleop]   /' >&2
  echo "[xr_teleop]       最常见原因：unitree 不在 dialout 组，打不开 /dev/ttyUSB*。" >&2
  echo "[xr_teleop]       修复：sudo usermod -aG dialout unitree （重新登录后重跑本脚本）" >&2
  return 1
}

start_dex1_gripper || echo "[xr_teleop] 继续启动遥操（夹爪可能不可用）" >&2

# 末尾 "$@" 透传额外参数，供 scripts/manual_teleop.sh 在后台启动时追加 --ipc
# （改用主程序自带的 IPC 输入通道，远程/无终端场景下也能发 R/Q）。
# 不传参数时行为与原来完全一致：前台运行 + sshkeyboard 键盘控制。
python teleop/teleop_hand_and_arm.py \
  --arm G1_29 \
  --ee dex1 \
  --input-mode controller \
  --display-mode "$DISPLAY_MODE" \
  --network-interface eth0 \
  --img-server-ip "$IMG_SERVER_IP" \
  --motion "$@"

# 双目摄像头不兼容 static dashboard，因此不启用 --static-dashboard。
