#!/usr/bin/env bash
# =============================================================================
# start_teleop.sh — XR 遥操作启动脚本（唯一入口）
#
# 职责：选末端 → 体检 → 拉起主程序；并提供远程启停控制。
#
# ┌─ 用法 ────────────────────────────────────────────────────────────────────┐
# │ 现场（有终端，交互选择末端）：                                             │
# │   bash scripts/start_teleop.sh                                            │
# │                                                                           │
# │ 指定末端，跳过菜单：                                                       │
# │   bash scripts/start_teleop.sh --ee none|auto|dex1|dex3|inspire|brainco       │
# │                                                                           │
# │ 远程 / 无终端（部署器、SSH 无 TTY）：                                      │
# │   bash scripts/start_teleop.sh start [--ee X]   后台待命，机器人不动        │
# │   bash scripts/start_teleop.sh control          开始跟随（等价按 R）        │
# │   bash scripts/start_teleop.sh stop             停止退出（等价按 Q，回零）  │
# │   bash scripts/start_teleop.sh detect           诊断：打印 IP 与末端探测结果 │
# │   bash scripts/start_teleop.sh status           输出 TELEOP_* KEY=VALUE    │
# │   bash scripts/start_teleop.sh logs [行数]      查看日志                    │
# └───────────────────────────────────────────────────────────────────────────┘
#
# 末端是自动探测的：脚本会订阅 DDS 状态话题，看机器人上实际装着哪种末端
# （Dex3 / Inspire / BrainCo / Dex1），并把探测结果作为菜单默认项。
# 直接回车即用探测结果；也可以手动选别的。
#
# 末端选项：
#   1) 无手        —— 纯手臂遥操，不等待任何末端话题
#   2) 两指夹爪    —— Unitree Dex1-1（需串口，脚本会自动体检并拉起夹爪服务）
#   3) 三指夹爪    —— Unitree Dex3-1
#   4) 因时灵巧手  —— Inspire（FTP 协议）
#
# 安全约束（不要绕过）：
#   * 不写 systemd unit、不 enable、不设开机自启、不自动重启；
#   * stop 优先走程序自身的退出路径（会执行 ctrl_dual_arm_go_home 手臂回零），
#     绝不用 SIGTERM / kill -9 —— 那会跳过回零，手臂停在未知姿态。
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# ---------------------------------------------------------------- 运行参数
CONDA_BIN="${XR_TELEOP_CONDA_BIN:-/home/unitree/miniconda3}"
CONDA_ENV="${XR_TELEOP_CONDA_ENV:-tv}"
PY="$CONDA_BIN/envs/$CONDA_ENV/bin/python"

# cv2 依赖 OpenMP 运行时，而 libgomp 用的是静态 TLS，必须在进程启动前预加载；
# 否则 import cv2 会报 `libgomp.so.1: cannot allocate memory in static TLS block`，
# 表现为遥操一起来就 ImportError 退出。start_camera.sh 一直有这一步，本脚本此前漏了。
GOMP_LIB="$CONDA_BIN/envs/$CONDA_ENV/lib/libgomp.so.1"
if [ -f "$GOMP_LIB" ]; then
  export LD_PRELOAD="$GOMP_LIB${LD_PRELOAD:+:$LD_PRELOAD}"
fi

RUN_USER="${XR_TELEOP_RUN_USER:-unitree}"
STATE_DIR="${XR_TELEOP_STATE_DIR:-/home/unitree/.cache/xr_teleoperate}"
PID_FILE="$STATE_DIR/teleop-manual.pid"
LOG_FILE="$STATE_DIR/teleop-manual.log"
PROC_PATTERN="[t]eleop_hand_and_arm.py"

DISPLAY_MODE="${DISPLAY_MODE:-immersive}"
MOTION="${MOTION:-1}"

DEX1_DIR="$REPO_ROOT/third_party/dex1_1_service"
DEX1_BIN="$DEX1_DIR/bin/dex1_1_gripper_server"
DEX1_LOG="${TMPDIR:-/tmp}/xr_dex1_gripper.log"
DEX1_NET_IF="${DEX1_NET_IF:-eth0}"

CURRENT_USER="$(id -un 2>/dev/null || echo unknown)"
SUDO=""
if [ "$CURRENT_USER" = "root" ] && [ "$CURRENT_USER" != "$RUN_USER" ]; then
  SUDO="sudo -u $RUN_USER"
fi

# ---------------------------------------------------------------- 基础工具
run_as_user() {
  if [ -n "$SUDO" ]; then
    $SUDO env PATH="$CONDA_BIN/bin:$PATH" "$@"
  else
    env PATH="$CONDA_BIN/bin:$PATH" "$@"
  fi
}

proc_pids() { pgrep -f "$PROC_PATTERN" 2>/dev/null || true; }
proc_running() { [ -n "$(proc_pids)" ]; }
proc_pid() { proc_pids | head -n1; }
is_ipc_mode() { pgrep -af "$PROC_PATTERN" 2>/dev/null | grep -q -- '--ipc'; }

field_of() { printf '%s\n' "$1" | sed -n "s/^$2=//p" | head -n1; }

usage() { sed -n '3,/^set -euo pipefail/p' "${BASH_SOURCE[0]}" | sed '$d' | sed 's/^# \{0,1\}//'; }

# ---------------------------------------------------------------- 网络地址自动探测
# 图像服务（teleimager）必须绑在「XR 头显能访问的那张网卡」上，通常是 Wi-Fi；
# DDS 走 eth0（机器人内部网）。两者搞混是现场最常见的连不上原因。
#
# 探测顺序：
#   1) 所有无线网卡（/sys/class/net/*/wireless）里第一个拿到全局 IPv4 的
#   2) 默认路由的源地址（有线也行，只要能出网）
#   3) hostname -I 的第一个
#
# 输出："<IP> <网卡名>"，调用方用 read -r ip ifc <<< "$(detect_wifi_ip)" 接收。
detect_wifi_ip() {
  local d ifc ip

  # 1) 无线网卡优先；scope global 已排除 169.254 链路本地地址
  for d in /sys/class/net/*; do
    [ -e "$d/wireless" ] || continue
    ifc="$(basename "$d")"
    ip="$(ip -4 -o addr show dev "$ifc" scope global 2>/dev/null | awk '{print $4}' | cut -d/ -f1 | head -n1)"
    if [ -n "$ip" ]; then
      echo "$ip $ifc"
      return 0
    fi
  done

  # 2) 没有可用无线网卡：退回默认路由的源地址
  if command -v ip >/dev/null 2>&1; then
    ip="$(ip route get 1.1.1.1 2>/dev/null | grep -oP '(?<=src\s)\S+' | head -n1 || true)"
    if [ -n "$ip" ]; then
      ifc="$(ip route get 1.1.1.1 2>/dev/null | grep -oP '(?<=dev\s)\S+' | head -n1 || true)"
      echo "$ip ${ifc:-?}"
      return 0
    fi
  fi

  # 3) 最后兜底
  ip="$(hostname -I 2>/dev/null | awk '{print $1}')"
  echo "${ip:-127.0.0.1} ?"
}

# =============================================================================
# 末端自动探测
#
# 机器人上到底装了哪种末端？靠 DDS 的 state 话题判断 —— 谁在发状态，谁就装着：
#   rt/dex3/{left,right}/state       三指夹爪 Dex3-1（机器人本体发布）
#   rt/inspire_hand/state/{l,r}      因时灵巧手（FTP）
#   rt/inspire/state                 因时灵巧手（DFX）
#   rt/brainco/{left,right}/state    强脑灵巧手
#   rt/dex1/{left,right}/state       两指夹爪（由本仓库的夹爪服务发布）
#
# 两指夹爪特殊：这个话题是我们自己起的服务发的。所以先看服务在不在，
# 不在就用串口握手确认夹爪物理在位，再拉服务、再探一次话题。
# =============================================================================
DETECTED_EE="none"

detect_ee_dds() {
  local iface="${1:-}"
  if [ ! -x "$PY" ]; then
    echo "NO_PYTHON=1"
    return 0
  fi
  ( cd "$REPO_ROOT" 2>/dev/null || exit 0
    run_as_user env PYTHONPATH="$REPO_ROOT" "$PY" - "$iface" <<'PY' 2>/dev/null || true
import sys
import threading

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.unitree_go.msg.dds_ import MotorStates_
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_

iface = sys.argv[1] if len(sys.argv) > 1 and sys.argv[1] else None
try:
    ChannelFactoryInitialize(0, networkInterface=iface)
except Exception:
    pass  # 已初始化过就忽略


def alive(topic, msgtype, timeout=0.7):
    """订阅一次，timeout 秒内有数据即认为该末端在线（只旁听，不打开设备）。

    注意：ChannelSubscriber.Read(timeout) 的 timeout 在本版 SDK 里不生效
    （实测会永久阻塞），所以必须用线程 + join 做超时，线程设为 daemon
    保证进程能正常退出。
    """
    got = {"ok": False}

    def worker():
        try:
            sub = ChannelSubscriber(topic, msgtype)
            sub.Init()
            if sub.Read() is not None:
                got["ok"] = True
        except Exception:
            pass

    t = threading.Thread(target=worker, daemon=True)
    t.start()
    t.join(timeout)
    return got["ok"]


res = {
    "DEX3": alive("rt/dex3/left/state", HandState_) or alive("rt/dex3/right/state", HandState_),
    "BRAINCO": alive("rt/brainco/left/state", MotorStates_) or alive("rt/brainco/right/state", MotorStates_),
    "DEX1": alive("rt/dex1/left/state", MotorStates_) or alive("rt/dex1/right/state", MotorStates_),
    "INSPIRE_DFX": alive("rt/inspire/state", MotorStates_),
}
try:
    import inspire_sdkpy.inspire_dds as inspire_dds
    res["INSPIRE_FTP"] = (alive("rt/inspire_hand/state/l", inspire_dds.inspire_hand_state)
                          or alive("rt/inspire_hand/state/r", inspire_dds.inspire_hand_state))
except Exception:
    res["INSPIRE_FTP"] = False

for k, v in res.items():
    print("%s=%d" % (k, 1 if v else 0))
PY
  )
}

# 探测结论写进 DETECTED_EE（none/dex1/dex3/inspire/brainco），并打印依据
# 参数：$1 = DDS 网卡（默认 eth0）；$2 = 是否允许顺手拉起夹爪服务（1/0，默认 1）
#      诊断模式传 0：只握手确认在位，不留下任何常驻进程
detect_ee() {
  local net_if="${1:-eth0}"
  local allow_start="${2:-1}"
  local out dex1 dex3 brainco idfx iftp
  local dex1_via="状态话题"

  out="$(detect_ee_dds "$net_if")"
  if printf '%s' "$out" | grep -q "NO_PYTHON=1"; then
    echo "[teleop][detect] 找不到 $PY，跳过末端探测" >&2
    DETECTED_EE="none"
    return 0
  fi

  dex1="$(field_of "$out" DEX1)"
  dex3="$(field_of "$out" DEX3)"
  brainco="$(field_of "$out" BRAINCO)"
  idfx="$(field_of "$out" INSPIRE_DFX)"
  iftp="$(field_of "$out" INSPIRE_FTP)"

  # 两指夹爪：话题没通就自己把服务拉起来再探一次
  if [ "${dex1:-0}" != "1" ]; then
    if check_dex1_gripper >/dev/null 2>&1; then
      if [ "$allow_start" = "1" ]; then
        if start_dex1_gripper >/dev/null 2>&1; then
          sleep 0.5
          dex1="$(field_of "$(detect_ee_dds "$net_if")" DEX1)"
          [ "${dex1:-0}" = "1" ] && echo "[teleop][detect] 两指夹爪：串口握手通过 + 状态话题已上线"
        fi
      else
        # 诊断模式：握手通过就认定在位，不启动服务
        dex1=1
        dex1_via="串口握手"
        echo "[teleop][detect] 两指夹爪：串口握手通过（诊断模式，未启动服务）"
      fi
    fi
  fi

  DETECTED_EE="none"
  if [ "${dex3:-0}" = "1" ]; then
    DETECTED_EE="dex3"
    echo "[teleop][detect] 检测到三指夹爪 Dex3-1（rt/dex3/*/state 有数据）"
  elif [ "${iftp:-0}" = "1" ] || [ "${idfx:-0}" = "1" ]; then
    DETECTED_EE="inspire"
    echo "[teleop][detect] 检测到因时灵巧手 Inspire（rt/inspire*/*state 有数据）"
  elif [ "${brainco:-0}" = "1" ]; then
    DETECTED_EE="brainco"
    echo "[teleop][detect] 检测到强脑灵巧手 BrainCo（rt/brainco/*/state 有数据）"
  elif [ "${dex1:-0}" = "1" ]; then
    DETECTED_EE="dex1"
    echo "[teleop][detect] 检测到两指夹爪 Dex1-1（依据：$dex1_via）"
  else
    echo "[teleop][detect] 未检测到任何末端，按纯手臂启动"
  fi
  return 0
}

# 两指夹爪（Dex1-1）体检
#
# 为什么必须先体检：主程序带 --ee dex1 时会无限阻塞等待 rt/dex1/{left,right}/state
# 的发布者。夹爪不可用时若照旧带 --ee，遥操会永远停在 "Waiting to subscribe dds..."，
# 现场表现是「遥操起不来」，比「没有夹爪」更难处理。
# =============================================================================
dex1_skip() { echo "[teleop][gripper] 跳过夹爪：$*" >&2; }

check_dex1_gripper() {
  if pgrep -f "[d]ex1_1_gripper_server" >/dev/null 2>&1; then
    echo "[teleop][gripper] 服务已在运行，复用（跳过体检）"
    return 0
  fi
  if [ ! -x "$DEX1_BIN" ]; then
    dex1_skip "找不到可执行文件 $DEX1_BIN"; return 1
  fi
  if [ "$(ls /dev/ttyUSB* 2>/dev/null | wc -l)" -eq 0 ]; then
    dex1_skip "没有 /dev/ttyUSB*（夹爪未接入或 USB 未识别）"; return 1
  fi
  if ! id -nG 2>/dev/null | tr ' ' '\n' | grep -qx dialout; then
    dex1_skip "当前用户不在 dialout 组，打不开串口（修复：sudo usermod -aG dialout \$USER 后重新登录）"
    return 1
  fi

  local out rc
  set +e
  out=$(LD_LIBRARY_PATH="$DEX1_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" "$DEX1_BIN" --reset 2>&1)
  rc=$?
  set -e
  echo "$out" | sed 's/^/[teleop][gripper][reset] /'

  if [ "$rc" -ne 0 ]; then
    dex1_skip "握手失败（--reset 退出码 $rc：未检测到电机，检查夹爪供电/USB）"; return 1
  fi
  if echo "$out" | grep -q "no reply after reset"; then
    dex1_skip "有电机无应答（串口被占用或电机未上电；可 sudo pkill -f dex1_1_gripper_server 后重试）"; return 1
  fi
  if ! echo "$out" | grep -q "after reset:"; then
    dex1_skip "没有电机返回状态"; return 1
  fi
  if echo "$out" | grep -q "故障未清除"; then
    dex1_skip "电机仍有故障码未清除（建议夹爪断电重启后再试）"; return 1
  fi

  echo "[teleop][gripper] 体检通过"
  return 0
}

start_dex1_gripper() {
  if [ ! -x "$DEX1_BIN" ]; then
    echo "[teleop] 错误：找不到夹爪服务 $DEX1_BIN" >&2; return 1
  fi
  # 方括号写法：避免 pgrep 匹配到本脚本自身
  if pgrep -f "[d]ex1_1_gripper_server" >/dev/null 2>&1; then
    echo "[teleop] dex1 夹爪服务已在运行，复用（不再起第二个实例）"
    return 0
  fi
  # 包内自带 libserialport.so.0 与 libUnitreeMotorSDK_Arm64.so，靠 LD_LIBRARY_PATH 加载
  export LD_LIBRARY_PATH="$DEX1_DIR/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
  echo "[teleop] 启动 dex1 夹爪服务..."
  setsid nohup "$DEX1_BIN" --network "$DEX1_NET_IF" > "$DEX1_LOG" 2>&1 < /dev/null &

  local i
  for i in $(seq 1 30); do
    if grep -q "Gripper Server started" "$DEX1_LOG" 2>/dev/null; then
      echo "[teleop] dex1 夹爪服务已就绪 (pid $(pgrep -f '[d]ex1_1_gripper_server' | head -n1))"
      echo "[teleop] 日志 $DEX1_LOG ；释放串口：sudo pkill -f dex1_1_gripper_server"
      return 0
    fi
    sleep 0.5
  done
  echo "[teleop] 错误：夹爪服务 15 秒内未就绪" >&2
  tail -n 20 "$DEX1_LOG" 2>/dev/null | sed 's/^/[teleop]   /' >&2
  return 1
}

# =============================================================================
# 末端选择
# =============================================================================
EE_KIND=""
EE_GIVEN=0        # 是否显式指定了 --ee

select_ee_interactive() {
  local tty_in="/dev/tty"
  if [ ! -r "$tty_in" ]; then
    EE_KIND="${DETECTED_EE:-none}"
    echo "[teleop] 无终端，按自动探测结果启动：$(ee_label "$EE_KIND")"
    return 0
  fi

  # 探测到哪个，就把哪个做成默认项（直接回车即用）
  local default_choice=1
  case "${DETECTED_EE:-none}" in
    dex1)    default_choice=2 ;;
    dex3)    default_choice=3 ;;
    inspire) default_choice=4 ;;
  esac

  echo ""
  echo "==================== 选择末端执行器 ===================="
  echo "  1) 无手          —— 纯手臂遥操"
  echo "  2) 两指夹爪      —— Unitree Dex1-1"
  echo "  3) 三指夹爪      —— Unitree Dex3-1"
  echo "  4) 因时灵巧手    —— Inspire"
  echo "--------------------------------------------------------"
  echo "  自动探测：$(ee_label "${DETECTED_EE:-none}")  → 直接回车即用"
  echo "========================================================"
  local choice=""
  read -r -p "请输入 1-4 [回车=${default_choice}]: " choice < "$tty_in" || choice=""
  [ -z "$choice" ] && choice="$default_choice"
  case "$choice" in
    1) EE_KIND="none" ;;
    2) EE_KIND="dex1" ;;
    3) EE_KIND="dex3" ;;
    4) EE_KIND="inspire" ;;
    *) echo "[teleop] 无效输入「$choice」，使用探测结果" >&2; EE_KIND="${DETECTED_EE:-none}" ;;
  esac
}

ee_label() {
  case "$1" in
    none)    echo "无手（纯手臂）" ;;
    auto)    echo "自动（有夹爪就带上）" ;;
    dex1)    echo "两指夹爪 Dex1-1" ;;
    dex3)    echo "三指夹爪 Dex3-1" ;;
    inspire) echo "因时灵巧手 Inspire" ;;
    *)       echo "$1" ;;
  esac
}

# 根据 EE_KIND 生成主程序的末端参数，并做必要的前置准备
# 同时确定 --input-mode：主程序明确拒绝 dex3/inspire + controller，必须用 hand
INPUT_MODE="controller"
build_ee_args() {
  EE_ARGS=()

  if [ "$EE_KIND" = "auto" ]; then
    EE_KIND="${DETECTED_EE:-none}"
    echo "[teleop] 采用自动探测结果：$(ee_label "$EE_KIND")"
  fi

  case "$EE_KIND" in
    none)
      echo "[teleop] 末端：无手（纯手臂遥操）"
      ;;
    dex1)
      echo "[teleop] 末端：两指夹爪 Dex1-1"
      if check_dex1_gripper && start_dex1_gripper; then
        EE_ARGS=(--ee dex1)
      else
        echo "[teleop][gripper] 夹爪不可用，本次降级为无手模式（不带 --ee）" >&2
        EE_KIND="none"
      fi
      ;;
    dex3)
      echo "[teleop] 末端：三指夹爪 Dex3-1"
      EE_ARGS=(--ee dex3)
      ;;
    inspire)
      echo "[teleop] 末端：因时灵巧手 Inspire（FTP）"
      EE_ARGS=(--ee inspire_ftp)
      ;;
    brainco)
      echo "[teleop] 末端：强脑灵巧手 BrainCo"
      EE_ARGS=(--ee brainco)
      ;;
    *)
      echo "[teleop] 错误：未知末端 $EE_KIND" >&2; exit 2 ;;
  esac

  case "$EE_KIND" in
    dex3|inspire) INPUT_MODE="hand" ;;
    *)            INPUT_MODE="controller" ;;
  esac
  echo "[teleop] 输入模式：$INPUT_MODE（由末端决定）"
}

# =============================================================================
# 启动主程序（前台 / 后台 IPC）
# =============================================================================
build_cmd() {
  local ip ifc
  read -r ip ifc <<< "$(detect_wifi_ip)"
  if [ -n "${IMG_SERVER_IP:-}" ]; then
    ip="$IMG_SERVER_IP"
    ifc="手动指定"
  fi
  echo "[teleop] 图像服务地址：$ip（网卡 ${ifc:-?}）   显示模式：$DISPLAY_MODE" >&2

  # 必须用 conda 环境里的绝对路径解释器，不能写裸 `python`。
  # 后台启动走的是 `env PATH="$CONDA_BIN/bin:$PATH"`，那里只加了 conda 的 base bin，
  # 裸 python 会解析成 base 环境的解释器 —— 缺 yaml / dex_retargeting，
  # 表现为启动后立刻 ModuleNotFoundError（yaml 在第 2 行就炸）。
  CMD=("$PY" teleop/teleop_hand_and_arm.py --arm G1_29)
  if [ "${#EE_ARGS[@]}" -gt 0 ]; then
    CMD+=("${EE_ARGS[@]}")
  fi
  CMD+=(
    --input-mode "$INPUT_MODE"
    --display-mode "$DISPLAY_MODE"
    --network-interface eth0
    --img-server-ip "$ip"
  )
  [ "$MOTION" = "1" ] && CMD+=(--motion)
}

launch_foreground() {
  cd "$REPO_ROOT"
  build_cmd
  echo "[teleop] 启动命令：${CMD[*]}"
  echo "[teleop] 进入后按 R 开始跟随、按 Q 停止退出（会执行手臂回零）"
  # shellcheck disable=SC1090
  source "$(conda info --base)/etc/profile.d/conda.sh"
  conda activate "$CONDA_ENV"
  export PYTHONPATH="$REPO_ROOT/teleop/televuer/src:$REPO_ROOT/teleop/teleimager/src:$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
  exec "${CMD[@]}" "$@"
}

prepare_state_dir() {
  mkdir -p "$STATE_DIR" 2>/dev/null || true
  [ -n "$SUDO" ] && chown "$RUN_USER" "$STATE_DIR" 2>/dev/null || true
  touch "$LOG_FILE" 2>/dev/null || true
  [ -n "$SUDO" ] && chown "$RUN_USER" "$LOG_FILE" 2>/dev/null || true
}

# ---------------------------------------------------------------- IPC 交互
ipc_interact() {
  local cmd="${1:-}"
  if [ ! -x "$PY" ]; then
    echo "IPC_ONLINE=0"; echo "IPC_NOTE=python_missing:$PY"; return 0
  fi
  cd "$REPO_ROOT" 2>/dev/null || return 0
  run_as_user env PYTHONPATH="$REPO_ROOT" "$PY" - "$cmd" <<'PY' 2>/dev/null || true
import sys, time

try:
    from teleop.utils.ipc import IPC_Client
except Exception as exc:
    print("IPC_ONLINE=0")
    print("IPC_NOTE=import_failed:%s" % exc)
    raise SystemExit(0)

cmd = sys.argv[1] if len(sys.argv) > 1 else ""
client = IPC_Client(hb_fps=10.0)

deadline = time.time() + 1.8
while not client.is_online() and time.time() < deadline:
    time.sleep(0.1)

online = client.is_online()
state = client.latest_state()
print("IPC_ONLINE=%d" % (1 if online else 0))

if cmd and online:
    reply = client.send_data(cmd)
    print("IPC_SEND_STATUS=%s" % reply.get("status", "error"))
    print("IPC_SEND_MSG=%s" % str(reply.get("msg", ""))[:120])
    time.sleep(1.2)
    state = client.latest_state()

print("IPC_START=%d" % (1 if state.get("START") else 0))
print("IPC_STOP=%d" % (1 if state.get("STOP") else 0))
print("IPC_READY=%d" % (1 if state.get("READY") else 0))
client.stop()
PY
}

# ---------------------------------------------------------------- 子命令
do_start() {
  if [ ! -x "$PY" ]; then
    echo "TELEOP_RUNNING=0"
    echo "ERROR=找不到 conda 环境 ${CONDA_ENV} 的解释器：$PY（先跑环境安装）"
    return 1
  fi
  if proc_running; then
    echo "TELEOP_RUNNING=1"
    echo "TELEOP_PID=$(proc_pid)"
    echo "ERROR=遥操作主程序已在运行；如需重启请先执行 stop"
    return 1
  fi

  # 后台模式没有终端，末端只能靠探测决定
  if [ "$EE_GIVEN" = "0" ] || [ -z "$EE_KIND" ]; then
    echo "[teleop] 正在自动探测末端（订阅 DDS 状态话题）..." >&2
    detect_ee eth0
    EE_KIND="auto"
  fi
  build_ee_args
  build_cmd

  prepare_state_dir
  rm -f "$PID_FILE" 2>/dev/null || true
  printf '\n[start_teleop] %s start ee=%s --ipc (repo=%s user=%s)\n' \
    "$(date '+%Y-%m-%d %H:%M:%S')" "$EE_KIND" "$REPO_ROOT" "$RUN_USER" >> "$LOG_FILE" 2>/dev/null || true

  cd "$REPO_ROOT" || return 1

  # setsid + nohup：脱离当前会话，SSH/部署器断开不会被带走。
  # --ipc：改用 IPC 输入通道，不依赖终端键盘。
  if [ -n "$SUDO" ]; then
    setsid nohup $SUDO env PATH="$CONDA_BIN/bin:$PATH" \
      "${CMD[@]}" --ipc >> "$LOG_FILE" 2>&1 &
  else
    setsid nohup env PATH="$CONDA_BIN/bin:$PATH" \
      "${CMD[@]}" --ipc >> "$LOG_FILE" 2>&1 &
  fi
  echo "$!" > "$PID_FILE" 2>/dev/null || true

  local i
  for i in $(seq 1 20); do
    sleep 0.5
    if proc_running; then
      echo "$(proc_pid)" > "$PID_FILE" 2>/dev/null || true
      echo "TELEOP_RUNNING=1"
      echo "TELEOP_PID=$(proc_pid)"
      echo "TELEOP_IPC=1"
      echo "TELEOP_EE=$EE_KIND"
      echo "TELEOP_CONTROL_ACTIVE=0"
      echo "TELEOP_LOG=$LOG_FILE"
      echo "MESSAGE=已后台启动（末端=$(ee_label "$EE_KIND")），等待开始跟随指令"
      return 0
    fi
  done

  echo "TELEOP_RUNNING=0"
  echo "TELEOP_LOG=$LOG_FILE"
  echo "ERROR=启动失败，10 秒内未见主程序进程；查看日志：bash ${BASH_SOURCE[0]} logs"
  return 1
}

do_control() {
  if ! proc_running; then
    echo "TELEOP_RUNNING=0"; echo "ERROR=遥操作主程序未运行；请先启动"; return 1
  fi
  if ! is_ipc_mode; then
    echo "TELEOP_RUNNING=1"
    echo "ERROR=当前进程未使用 IPC 模式（不是本脚本后台启动的），无法远程发送 R；请先用 stop 结束它"
    return 1
  fi

  local out online status
  out="$(ipc_interact CMD_START)"
  printf '%s\n' "$out"
  online="$(field_of "$out" IPC_ONLINE)"
  status="$(field_of "$out" IPC_SEND_STATUS)"

  if [ "$online" = "1" ] && [ "$status" = "ok" ]; then
    echo "TELEOP_RUNNING=1"
    echo "TELEOP_CONTROL_ACTIVE=1"
    echo "MESSAGE=已开始跟随（等价于终端按 R）"
    return 0
  fi
  echo "TELEOP_CONTROL_ACTIVE=0"
  echo "ERROR=开始跟随指令未送达（ipc_online=${online:-0} status=${status:-unknown}）"
  return 1
}

do_stop() {
  if ! proc_running; then
    rm -f "$PID_FILE" 2>/dev/null || true
    echo "TELEOP_RUNNING=0"; echo "TELEOP_CONTROL_ACTIVE=0"; echo "MESSAGE=遥操作主程序未运行"
    return 0
  fi

  local graceful=0 out
  if is_ipc_mode; then
    out="$(ipc_interact CMD_STOP)"
    printf '%s\n' "$out"
    [ "$(field_of "$out" IPC_SEND_STATUS)" = "ok" ] && graceful=1
  fi

  local i
  for i in $(seq 1 16); do
    sleep 0.5
    if ! proc_running; then
      rm -f "$PID_FILE" 2>/dev/null || true
      echo "TELEOP_RUNNING=0"; echo "TELEOP_CONTROL_ACTIVE=0"
      if [ "$graceful" = "1" ]; then
        echo "MESSAGE=已优雅退出（等价于终端按 Q，执行了手臂回零）"
      else
        echo "MESSAGE=已退出"
      fi
      return 0
    fi
  done

  # 兜底用 SIGINT：走 KeyboardInterrupt 分支，finally 里仍会执行手臂回零。
  # 绝不用 SIGTERM（默认动作直接杀进程，finally 不执行，手臂停在未知姿态）。
  echo "MESSAGE=指令未使主程序退出，改用 SIGINT 兜底"
  pkill -INT -f "$PROC_PATTERN" 2>/dev/null || true

  for i in $(seq 1 12); do
    sleep 0.5
    if ! proc_running; then
      rm -f "$PID_FILE" 2>/dev/null || true
      echo "TELEOP_RUNNING=0"; echo "TELEOP_CONTROL_ACTIVE=0"; echo "MESSAGE=已通过 SIGINT 停止"
      return 0
    fi
  done

  echo "TELEOP_RUNNING=1"
  echo "TELEOP_PID=$(proc_pid)"
  echo "ERROR=停止失败，进程仍在运行；请现场排查（避免强杀，以免手臂未回零）"
  return 1
}

do_status() {
  local running=0 pid="" ipc=0 control=0

  if proc_running; then
    running=1
    pid="$(proc_pid)"
    if is_ipc_mode; then
      ipc=1
      local out; out="$(ipc_interact '')"
      printf '%s\n' "$out" | grep -E '^IPC_(ONLINE|NOTE)=' || true
      if [ "$(field_of "$out" IPC_ONLINE)" = "1" ] && [ "$(field_of "$out" IPC_START)" = "1" ]; then
        control=1
      fi
    fi
    echo "$pid" > "$PID_FILE" 2>/dev/null || true
  else
    rm -f "$PID_FILE" 2>/dev/null || true
  fi

  echo "TELEOP_RUNNING=$running"
  echo "TELEOP_PID=$pid"
  echo "TELEOP_IPC=$ipc"
  echo "TELEOP_CONTROL_ACTIVE=$control"
  echo "TELEOP_LOG=$LOG_FILE"
}

# =============================================================================
# 主流程
# =============================================================================
SUB="${1:-}"
case "$SUB" in
  start)
    shift
    while [[ $# -gt 0 ]]; do
      case "$1" in
        --ee) EE_KIND="${2:-}"; EE_GIVEN=1; shift $(( $# >= 2 ? 2 : 1 )) ;;
        *) echo "[teleop] start 不认识的参数：$1" >&2; exit 2 ;;
      esac
    done
    case "$EE_KIND" in
      ""|none|auto|dex1|dex3|inspire|brainco) ;;
      *) echo "[teleop] 错误：--ee 仅支持 none|auto|dex1|dex3|inspire|brainco（当前 $EE_KIND）" >&2; exit 2 ;;
    esac
    do_start; exit $?
    ;;
  detect)
    # 诊断：把网络与末端探测结果打出来，不启动任何东西
    read -r _ip _ifc <<< "$(detect_wifi_ip)"
    echo "WIFI_IP=$_ip"
    echo "WIFI_IFACE=$_ifc"
    detect_ee eth0 0
    echo "DETECTED_EE=$DETECTED_EE"
    exit 0
    ;;
  control|follow) do_control; exit $? ;;
  stop)           do_stop; exit $? ;;
  status)         do_status; exit $? ;;
  logs)           tail -n "${2:-100}" "$LOG_FILE" 2>/dev/null || echo "暂无日志：$LOG_FILE"; exit $? ;;
  help|-h|--help) usage; exit 0 ;;
esac

# 前台模式：解析 --ee 或走交互菜单
while [[ $# -gt 0 ]]; do
  case "$1" in
    --ee) EE_KIND="${2:-}"; EE_GIVEN=1; shift $(( $# >= 2 ? 2 : 1 )) ;;
    *) echo "[teleop] 不认识的参数：$1（--help 查看用法）" >&2; exit 2 ;;
  esac
done

case "$EE_KIND" in
  ""|none|auto|dex1|dex3|inspire|brainco) ;;
  *) echo "[teleop] 错误：--ee 仅支持 none|auto|dex1|dex3|inspire|brainco（当前 $EE_KIND）" >&2; exit 2 ;;
esac

if proc_running; then
  echo "[teleop] 错误：遥操作主程序已在运行（pid $(proc_pid)）" >&2
  echo "[teleop]       先执行：bash ${BASH_SOURCE[0]} stop" >&2
  exit 1
fi

# 没指定 --ee、或显式 --ee auto：都先自动探测
if [ "$EE_GIVEN" = "0" ] || [ "$EE_KIND" = "auto" ]; then
  echo "[teleop] 正在自动探测末端（订阅 DDS 状态话题）..."
  detect_ee eth0
fi

# 没显式指定：弹菜单，默认项就是探测结果
if [ "$EE_GIVEN" = "0" ]; then
  select_ee_interactive
fi

build_ee_args
launch_foreground
