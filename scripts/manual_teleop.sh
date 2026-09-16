#!/usr/bin/env bash
# manual_teleop.sh — 遥操作主程序的人工启停包装（供部署器「手动操作」按钮调用）
#
# 背景（为什么需要这个包装）：
#   * scripts/start_xr_teleop.sh 是前台阻塞脚本；
#   * teleop/teleop_hand_and_arm.py 默认用 sshkeyboard 读键盘 —— 二者都要求一个真实终端(TTY)。
#   部署器/远程 SSH 没有 TTY，直接拉起会卡在"等待 R"且无法用 Q 退出，只能强杀（会跳过手臂回零）。
#   因此这里改用主程序自带的 IPC 输入通道（--ipc），把"按键"变成可远程发送的指令：
#
#   start    后台启动主程序（--ipc），写 PID 与日志。此时程序只等指令，机器人不会动。
#   control  发送 CMD_START —— 等价于在终端里按 R（开始跟随）。
#   stop     发送 CMD_STOP  —— 等价于按 Q（清理并退出、手臂回零）；IPC 不可用时才 SIGINT 兜底。
#   status   输出 KEY=VALUE，供部署器解析回显（进程是否存活 / 是否已开控）。
#   logs     查看日志。
#
# 安全约束（与部署器 DangerousScriptPolicy 一致）：
#   * 不写 systemd unit、不 enable、不设开机自启、不自动重启；
#   * 只允许人工显式触发 start/control/stop；
#   * 停止优先走 CMD_STOP（程序自身的退出路径，会执行 ctrl_dual_arm_go_home），
#     坚决不用 SIGTERM / systemctl stop（会跳过回零清理）。
#
# 用法: manual_teleop.sh {start|control|stop|status|logs [行数]}

set -u

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# conda env 位置（脚本里用 `conda info --base`，非交互 SSH 下 conda 可能不在 PATH，这里显式提供）
CONDA_BIN="${XR_TELEOP_CONDA_BIN:-/home/unitree/miniconda3}"
CONDA_ENV="${XR_TELEOP_CONDA_ENV:-tv}"
PY="$CONDA_BIN/envs/$CONDA_ENV/bin/python"

# 运行主程序的用户（与现场手动启动保持一致：DDS 组播、相机、串口权限都挂在该用户上）
RUN_USER="${XR_TELEOP_RUN_USER:-unitree}"
STATE_DIR="${XR_TELEOP_STATE_DIR:-/home/unitree/.cache/xr_teleoperate}"
PID_FILE="$STATE_DIR/teleop-manual.pid"
LOG_FILE="$STATE_DIR/teleop-manual.log"
PROC_PATTERN="[t]eleop_hand_and_arm.py"

CURRENT_USER="$(id -un 2>/dev/null || echo unknown)"
SUDO=""
if [ "$CURRENT_USER" = "root" ] && [ "$CURRENT_USER" != "$RUN_USER" ]; then
  SUDO="sudo -u $RUN_USER"
fi

# ---------------------------------------------------------------- 基础工具

run_as_user() {
  # 统一以 RUN_USER 身份执行，并把 conda 的 bin 放进 PATH（否则启动脚本第 7 行的
  # `conda info --base` 在非交互环境下会直接失败）。
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

prepare_state_dir() {
  mkdir -p "$STATE_DIR" 2>/dev/null || true
  if [ -n "$SUDO" ]; then
    chown "$RUN_USER" "$STATE_DIR" 2>/dev/null || true
  fi
  touch "$LOG_FILE" 2>/dev/null || true
  if [ -n "$SUDO" ]; then
    chown "$RUN_USER" "$LOG_FILE" 2>/dev/null || true
  fi
}

# 与主程序的 IPC 服务交互（REQ/REP 发指令 + PUB/SUB 读心跳）。
# $1 = CMD_START | CMD_STOP | 空（只读状态）
ipc_interact() {
  local cmd="${1:-}"
  if [ ! -x "$PY" ]; then
    echo "IPC_ONLINE=0"
    echo "IPC_NOTE=python_missing:$PY"
    return 0
  fi

  cd "$REPO_ROOT" 2>/dev/null || return 0
  run_as_user env PYTHONPATH="$REPO_ROOT" "$PY" - "$cmd" <<'PY' 2>/dev/null || true
import sys, time

try:
    from teleop.utils.ipc import IPC_Client
except Exception as exc:  # 依赖缺失时给出可诊断信息，不阻断脚本
    print("IPC_ONLINE=0")
    print("IPC_NOTE=import_failed:%s" % exc)
    raise SystemExit(0)

cmd = sys.argv[1] if len(sys.argv) > 1 else ""
client = IPC_Client(hb_fps=10.0)

# 主程序判定在线需要连续 3 次心跳（约 0.3s）；给 1.8s 余量（回读状态要快，避免打开卡片卡顿）。
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
    time.sleep(1.2)          # 等状态机走一步再读心跳
    state = client.latest_state()

print("IPC_START=%d" % (1 if state.get("START") else 0))
print("IPC_STOP=%d" % (1 if state.get("STOP") else 0))
print("IPC_READY=%d" % (1 if state.get("READY") else 0))
client.stop()
PY
}

field_of() { # field_of "输出" KEY
  printf '%s\n' "$1" | sed -n "s/^$2=//p" | head -n1
}

# ---------------------------------------------------------------- 子命令

do_start() {
  if proc_running; then
    echo "TELEOP_RUNNING=1"
    echo "TELEOP_PID=$(proc_pid)"
    echo "ERROR=遥操作主程序已在运行；如需重启请先执行 stop"
    return 1
  fi

  if [ ! -f "$REPO_ROOT/scripts/start_xr_teleop.sh" ]; then
    echo "TELEOP_RUNNING=0"
    echo "ERROR=未找到启动脚本: $REPO_ROOT/scripts/start_xr_teleop.sh"
    return 1
  fi

  prepare_state_dir
  rm -f "$PID_FILE" 2>/dev/null || true
  printf '\n[manual_teleop] %s start --ipc (repo=%s user=%s)\n' \
    "$(date '+%Y-%m-%d %H:%M:%S')" "$REPO_ROOT" "$RUN_USER" >> "$LOG_FILE" 2>/dev/null || true

  cd "$REPO_ROOT" || return 1

  # setsid + nohup：进程脱离当前会话，部署器/SSH 断开也不会被带走。
  # --ipc：主程序改用 IPC 输入通道，不再依赖终端键盘。
  if [ -n "$SUDO" ]; then
    setsid nohup $SUDO env PATH="$CONDA_BIN/bin:$PATH" \
      bash "$REPO_ROOT/scripts/start_xr_teleop.sh" --ipc >> "$LOG_FILE" 2>&1 &
  else
    setsid nohup env PATH="$CONDA_BIN/bin:$PATH" \
      bash "$REPO_ROOT/scripts/start_xr_teleop.sh" --ipc >> "$LOG_FILE" 2>&1 &
  fi
  echo "$!" > "$PID_FILE" 2>/dev/null || true

  # 等主程序真正出现（最多 10s）
  local i
  for i in $(seq 1 20); do
    sleep 0.5
    if proc_running; then
      echo "$(proc_pid)" > "$PID_FILE" 2>/dev/null || true
      echo "TELEOP_RUNNING=1"
      echo "TELEOP_PID=$(proc_pid)"
      echo "TELEOP_IPC=1"
      echo "TELEOP_CONTROL_ACTIVE=0"
      echo "TELEOP_LOG=$LOG_FILE"
      echo "MESSAGE=已后台启动，等待开始跟随指令（等价于终端等待按 R）"
      return 0
    fi
  done

  echo "TELEOP_RUNNING=0"
  echo "TELEOP_LOG=$LOG_FILE"
  echo "ERROR=启动后未检测到遥操作进程（约 10 秒）。日志末尾："
  tail -n 20 "$LOG_FILE" 2>/dev/null || true
  return 1
}

do_control() {
  if ! proc_running; then
    echo "TELEOP_RUNNING=0"
    echo "ERROR=遥操作主程序未运行；请先启动"
    return 1
  fi

  if ! is_ipc_mode; then
    echo "TELEOP_RUNNING=1"
    echo "ERROR=当前进程未使用 IPC 模式（不是本脚本启动的），无法远程发送 R；请先用 stop 结束它，再由部署器重新启动"
    return 1
  fi

  local out; out="$(ipc_interact CMD_START)"
  printf '%s\n' "$out"

  local online status
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
    echo "TELEOP_RUNNING=0"
    echo "TELEOP_CONTROL_ACTIVE=0"
    echo "MESSAGE=遥操作主程序未运行"
    return 0
  fi

  local graceful=0
  if is_ipc_mode; then
    local out; out="$(ipc_interact CMD_STOP)"
    printf '%s\n' "$out"
    if [ "$(field_of "$out" IPC_SEND_STATUS)" = "ok" ]; then
      graceful=1
    fi
  fi

  # 等它自己走完退出流程（finally 里会执行手臂回零）
  local i
  for i in $(seq 1 16); do
    sleep 0.5
    if ! proc_running; then
      rm -f "$PID_FILE" 2>/dev/null || true
      echo "TELEOP_RUNNING=0"
      echo "TELEOP_CONTROL_ACTIVE=0"
      echo "MESSAGE=$([ "$graceful" = "1" ] && echo '已优雅退出（等价于终端按 Q，执行了手臂回零）' || echo '已退出')"
      return 0
    fi
  done

  # 兜底：SIGINT 会走 KeyboardInterrupt 分支，仍会执行 finally 里的手臂回零；
  # 绝不用 SIGTERM（默认动作是直接杀进程，finally 不执行，手臂停在未知姿态）。
  echo "MESSAGE=指令未使主程序退出，改用 SIGINT 兜底"
  pkill -INT -f "$PROC_PATTERN" 2>/dev/null || true

  for i in $(seq 1 12); do
    sleep 0.5
    if ! proc_running; then
      rm -f "$PID_FILE" 2>/dev/null || true
      echo "TELEOP_RUNNING=0"
      echo "TELEOP_CONTROL_ACTIVE=0"
      echo "MESSAGE=已通过 SIGINT 停止"
      return 0
    fi
  done

  echo "TELEOP_RUNNING=1"
  echo "TELEOP_PID=$(proc_pid)"
  echo "ERROR=停止失败，进程仍在运行；请现场排查后手动处理（避免使用强杀，以免手臂未回零）"
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

usage() {
  cat <<'EOF'
manual_teleop.sh {start|control|stop|status|logs [行数]}

  start     后台启动遥操作主程序（IPC 模式，此时不会跟随，机器人不动）
  control   发送开始跟随指令（等价于在终端按 R）
  stop      发送退出指令（等价于按 Q，程序会执行手臂回零）
  status    输出 TELEOP_* KEY=VALUE 状态
  logs N    查看最后 N 行日志（默认 100）

安全：不写 systemd unit、不设开机自启、不自动重启；仅人工显式触发。
EOF
}

case "${1:-status}" in
  start) do_start; exit $? ;;
  control|follow) do_control; exit $? ;;
  stop) do_stop; exit $? ;;
  status) do_status; exit $? ;;
  logs) tail -n "${2:-100}" "$LOG_FILE" 2>/dev/null || echo "暂无日志"; exit $? ;;
  help|-h|--help) usage; exit 0 ;;
  *) usage; exit 2 ;;
esac
