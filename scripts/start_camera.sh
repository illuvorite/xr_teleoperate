#!/usr/bin/env bash
# =============================================================================
# start_camera.sh — 摄像头画面服务（teleimager）启动脚本
#
# 职责：只做一件事 —— 把机器人头部相机的画面服务拉起来。
#
# 相机自动探测顺序（越靠前越优先）：
#   1) 双目相机（SCAM 立体相机，SBS 双眼画面）—— 有就用它
#   2) 深度相机（Intel RealSense）           —— 没有双目就退而求其次
#   3) 无相机                                —— 都没有也不报错，服务照常启动
#
# 「服务照常启动」的含义（重要）：
#   无相机时进程依然常驻，60000 端口（相机配置查询）可用，/status 相关逻辑不崩；
#   只是不出图（60001 不监听）。部署器的健康检查是「进程存活」，所以不会判红。
#   这样现场就算相机坏了/没插，也能把服务先跑起来，不至于整个部署链路失败。
#
# 用法：
#   bash scripts/start_camera.sh                  # 前台启动（自动探测，systemd 用这个）
#   bash scripts/start_camera.sh --detect         # 只探测并打印结果，不启动
#   bash scripts/start_camera.sh --restart        # 先停旧实例再启动
#   bash scripts/start_camera.sh --status         # 查看运行状态
#   bash scripts/start_camera.sh --logs [行数]    # 查看日志（默认 100 行）
#   bash scripts/start_camera.sh --find-camera    # 列出相机与 sysfs 路径（填 physical_path 用）
#   DAEMON=1 bash scripts/start_camera.sh         # 后台启动
#   CAM_KIND=scam|realsense|none bash scripts/start_camera.sh   # 强制指定，跳过探测
#
# 环境变量：
#   DAEMON=1          后台运行（nohup + pid 文件）
#   CAM_KIND=         强制相机类型，跳过自动探测
#   CONDA_BASE=       conda 安装根目录（默认自动发现）
#   CAMERA_SDK_DIR=   SCAM SDK 目录（默认 vendor/camera_sdk_test）
#   GLIBC235_DIR=     SCAM 用的 glibc-2.35 加载器目录
#   LOG_FILE=         日志路径（默认 teleimager_auto.log）
# =============================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TEL_DIR="$REPO_ROOT/teleop/teleimager"
PIDFILE="$TEL_DIR/.teleimager.pid"
LOG_FILE="${LOG_FILE:-$TEL_DIR/teleimager_auto.log}"
STATE_DIR="${XR_TELEOP_STATE_DIR:-/home/unitree/.cache/xr_teleoperate}"

DAEMON="${DAEMON:-0}"
CAM_KIND="${CAM_KIND:-auto}"
RESTART=""
ACTION="run"

# ---------------------------------------------------------------- 参数解析
while [[ $# -gt 0 ]]; do
  case "$1" in
    --detect)  ACTION="detect"; shift ;;
    --find-camera|--cf) ACTION="find-camera"; shift ;;
    --restart) RESTART=1; shift ;;
    --status)  ACTION="status"; shift ;;
    --logs)    ACTION="logs"; LOG_LINES="${2:-100}"; shift $(( $# >= 2 ? 2 : 1 )) ;;
    -h|--help) sed -n '2,34p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'; exit 0 ;;
    *) echo "[camera] 未知参数：$1（--help 查看用法）" >&2; exit 2 ;;
  esac
done

# ---------------------------------------------------------------- 状态/日志（不需要 conda）
if [[ "$ACTION" == "status" ]]; then
  if pgrep -f "teleimager-server --no-affinity" >/dev/null 2>&1; then
    echo "CAMERA_RUNNING=1"
    echo "CAMERA_PID=$(pgrep -f 'teleimager-server --no-affinity' | head -n1)"
  else
    echo "CAMERA_RUNNING=0"
    echo "CAMERA_PID="
  fi
  echo "CAMERA_LOG=$LOG_FILE"
  exit 0
fi

if [[ "$ACTION" == "logs" ]]; then
  tail -n "${LOG_LINES:-100}" "$LOG_FILE" 2>/dev/null || echo "暂无日志：$LOG_FILE"
  exit 0
fi

# ---------------------------------------------------------------- conda 环境
# 探测阶段就要用到 python（pyrealsense2 / SCAM SDK 都在 tv 环境里），所以先激活。
CONDA_BASE="${CONDA_BASE:-}"
if [[ -z "$CONDA_BASE" ]]; then
  if command -v conda >/dev/null 2>&1; then
    CONDA_BASE="$(conda info --base)"
  elif [[ -f "$HOME/miniconda3/etc/profile.d/conda.sh" ]]; then
    CONDA_BASE="$HOME/miniconda3"
  elif [[ -f /opt/miniconda3/etc/profile.d/conda.sh ]]; then
    CONDA_BASE=/opt/miniconda3
  fi
fi
if [[ -z "$CONDA_BASE" || ! -f "$CONDA_BASE/etc/profile.d/conda.sh" ]]; then
  echo "[camera] 错误：找不到 conda（已尝试 PATH/miniconda3//opt/miniconda3），可用 CONDA_BASE= 指定" >&2
  exit 1
fi
# shellcheck disable=SC1090
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate "${XR_TELEOP_CONDA_ENV:-tv}"

export PYTHONPATH="$REPO_ROOT/teleop/teleimager/src:$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$REPO_ROOT"

# ---------------------------------------------------------------- 相机探测
VENDOR_DIR="${VENDOR_DIR:-$TEL_DIR/vendor}"
CAMERA_SDK_DIR="${CAMERA_SDK_DIR:-$VENDOR_DIR/camera_sdk_test}"
export CAMERA_SDK_DIR

# 探测结果通过这三个全局变量返回
DETECTED_KIND="none"
DETECTED_SERIAL=""
DETECTED_NOTE=""

# 探测用 python 片段：
#   SCAM     → 直接调 SDK 的 SCAM_EnumDevices，只认 isExist 的设备（不只看库文件在不在）
#   RealSense→ pyrealsense2 枚举设备拿 ASIC 序列号
#   都没有   → none
#
# 注意：只做「枚举」，不打开设备，所以不会抢占相机，探测完立刻释放。
probe_camera() {
  python - <<'PY' 2>/dev/null || true
import ctypes
import os
import sys
import time

sdk_dir = os.environ.get("CAMERA_SDK_DIR", "")
lib_path = os.path.join(sdk_dir, "build", "libscam.so")
scam_count = 0
scam_err = ""

# ---- 1) 双目（SCAM）----
if os.path.isfile(lib_path):
    try:
        from teleimager.scam_camera import _DeviceInfo, _ScamLibrary

        lib = _ScamLibrary(lib_path)
        if lib.lib.SCAM_Initialize() == 0:
            cap = 8
            arr = (_DeviceInfo * cap)()
            n = ctypes.c_int(0)
            lib.lib.SCAM_EnumDevices(arr, cap, ctypes.byref(n))
            for i in range(min(n.value, cap)):
                if arr[i].isExist:
                    scam_count += 1
            lib.lib.SCAM_Release()
        else:
            scam_err = "SCAM_Initialize failed"
    except Exception as exc:
        scam_err = "%s: %s" % (type(exc).__name__, exc)
else:
    scam_err = "libscam.so not found"

if scam_count > 0:
    print("KIND=scam")
    print("SCAM_COUNT=%d" % scam_count)
    raise SystemExit(0)

# ---- 2) 深度（RealSense）----
# 必须重试：服务重启的瞬间，上一个进程还没把相机放掉（或 USB 正在重新枚举），
# 这时 query_devices() 会返回空。只探一次就会误判成「没有相机」，
# 进而用空配置启动 —— 表现为服务活着但 60001 不出图，极难排查。
rs_serials = []
rs_err = ""
for _attempt in range(6):
    try:
        import pyrealsense2 as rs

        ctx = rs.context()
        for dev in ctx.query_devices():
            try:
                rs_serials.append(dev.get_info(rs.camera_info.serial_number))
            except Exception:
                continue
    except Exception as exc:
        rs_err = "%s: %s" % (type(exc).__name__, exc)
    if rs_serials:
        break
    time.sleep(1.0)

if rs_serials:
    print("KIND=realsense")
    print("RS_SERIAL=%s" % rs_serials[0])
    print("RS_COUNT=%d" % len(rs_serials))
    raise SystemExit(0)

# ---- 3) 都没有 ----
print("KIND=none")
print("SCAM_NOTE=%s" % (scam_err or "no device"))
print("RS_NOTE=%s" % (rs_err or "no device"))
PY
}

if [[ "$CAM_KIND" != "auto" ]]; then
  case "$CAM_KIND" in
    scam|realsense|none) DETECTED_KIND="$CAM_KIND" ;;
    *) echo "[camera] 错误：CAM_KIND 仅支持 scam|realsense|none（当前 $CAM_KIND）" >&2; exit 2 ;;
  esac
  DETECTED_NOTE="由 CAM_KIND 强制指定"
else
  probe_out="$(probe_camera)"
  DETECTED_KIND="$(printf '%s\n' "$probe_out" | sed -n 's/^KIND=//p' | head -n1)"
  DETECTED_SERIAL="$(printf '%s\n' "$probe_out" | sed -n 's/^RS_SERIAL=//p' | head -n1)"
  [[ -z "$DETECTED_KIND" ]] && DETECTED_KIND="none"
fi

# ---------------------------------------------------------------- 选择配置与后端
# 三种情况对应三套配置：
#   scam      → 用主配置 cam_config_server.yaml（本身就是 SCAM 双目配置）
#   realsense → 用 cam_config_server_realsense.yaml + --rs 参数
#   none      → 现场生成一份「全部相机关闭」的配置，保证服务能起但不碰相机
BASE_CONFIG="$TEL_DIR/cam_config_server.yaml"
RS_CONFIG="$TEL_DIR/cam_config_server_realsense.yaml"
ACTIVE_CONFIG=""
CAM_BACKEND="opencv"
EXTRA_ARGS=""

case "$DETECTED_KIND" in
  scam)
    ACTIVE_CONFIG="$BASE_CONFIG"
    CAM_BACKEND="scam"
    EXTRA_ARGS=""
    ;;
  realsense)
    if [[ ! -f "$RS_CONFIG" ]]; then
      echo "[camera] 警告：缺少 RealSense 配置 $RS_CONFIG，降级为无相机模式" >&2
      DETECTED_KIND="none"
    else
      ACTIVE_CONFIG="$RS_CONFIG"
      CAM_BACKEND="opencv"
      EXTRA_ARGS="--rs"
    fi
    ;;
esac

if [[ "$DETECTED_KIND" == "none" ]]; then
  # 无相机配置：三路相机全部 enable_zmq/enable_webrtc = false。
  # ImageServer 会跳过所有相机，_cameras 为空 → start() 什么都不做 →
  # wait() 阻塞在 stop_event 上 → 进程常驻但不碰相机。
  mkdir -p "$STATE_DIR" 2>/dev/null || true
  ACTIVE_CONFIG="$STATE_DIR/cam_config_none.yaml"
  cat > "$ACTIVE_CONFIG" <<'YAML'
# 自动生成：未检测到任何相机（双目/深度均不可用）
# 服务仍然启动，但不出图。接入相机后重新执行 start_camera.sh 即可。
head_camera:
  enable_zmq: false
  enable_webrtc: false
  type: opencv
  image_shape: [720, 1280]
  binocular: false
  fps: 30
left_wrist_camera:
  enable_zmq: false
  enable_webrtc: false
  type: opencv
  image_shape: [480, 640]
  binocular: false
  fps: 30
right_wrist_camera:
  enable_zmq: false
  enable_webrtc: false
  type: opencv
  image_shape: [480, 640]
  binocular: false
  fps: 30
YAML
  CAM_BACKEND="opencv"
  EXTRA_ARGS=""
fi

echo "[camera] 探测结果：$DETECTED_KIND${DETECTED_SERIAL:+（RealSense SN=$DETECTED_SERIAL）}"
echo "[camera] 使用配置：$ACTIVE_CONFIG  backend=$CAM_BACKEND ${EXTRA_ARGS}"

if [[ "$ACTION" == "detect" ]]; then
  echo "CAM_KIND=$DETECTED_KIND"
  echo "CAM_SERIAL=$DETECTED_SERIAL"
  echo "CAM_CONFIG=$ACTIVE_CONFIG"
  echo "CAM_BACKEND=$CAM_BACKEND"
  exit 0
fi

# --find-camera：列出所有 V4L2 相机与 sysfs 接口路径，用于填写
# cam_config_server.yaml 的 physical_path / video_id（排查「相机在不在位」时也用这个）
if [[ "$ACTION" == "find-camera" ]]; then
  exec "$CONDA_PREFIX/bin/teleimager-server" --cf
fi

# ---------------------------------------------------------------- 释放相机
# 机器人官方的 master_service 会在 USB 相机出现后拉起 videohub_pc4，并抓住
# RealSense 的 /dev/video4 UVC 节点不放。此时 pipeline.start() 会报：
#     xioctl(VIDIOC_S_FMT) failed Last Error: Device or resource busy
# 一相机只能喂一路，遥操优先 —— 必须先把它请走。
#
# systemd 模式下这一步由 unit 的 ExecStartPre 以 root 完成；手动跑本脚本时
# 只能尽力而为：先自己 pkill（大概率因属 root 而失败），再试免密 sudo，
# 都不行就打印明确命令后退出，而不是丢一堆 Python traceback 给用户。
release_camera() {
  pgrep -f "[v]ideohub_pc4 /dev/video4" >/dev/null 2>&1 || return 0

  echo "[camera] 检测到 videohub_pc4 占用 RealSense 节点，尝试释放..."
  pkill -f "[v]ideohub_pc4 /dev/video4" 2>/dev/null || true
  sleep 1
  pgrep -f "[v]ideohub_pc4 /dev/video4" >/dev/null 2>&1 || { echo "[camera] 已释放相机"; return 0; }

  sudo -n pkill -f "[v]ideohub_pc4 /dev/video4" 2>/dev/null || true
  sleep 1
  pgrep -f "[v]ideohub_pc4 /dev/video4" >/dev/null 2>&1 || { echo "[camera] 已释放相机"; return 0; }

  echo "[camera] 错误：videohub_pc4 仍占用相机（该进程属 root，普通用户杀不掉）" >&2
  echo "[camera]       请先执行：" >&2
  echo "[camera]         sudo pkill -f 'videohub_pc4 /dev/video4'" >&2
  echo "[camera]       然后重新运行本脚本。这是机器人官方相机服务，一相机只能喂一路。" >&2
  return 1
}

# ---------------------------------------------------------------- 重复启动保护
# 匹配 teleimager-server 的真实 cmdline，不会误伤本脚本或外层 shell。
if pgrep -f "teleimager-server --no-affinity" >/dev/null 2>&1; then
  if [[ -n "$RESTART" ]]; then
    echo "[camera] 检测到已在运行（--restart）：停止旧实例..."
    pkill -f "teleimager-server --no-affinity" || true
    sleep 2
  else
    echo "[camera] 错误：teleimager-server 已在运行。" >&2
    echo "[camera]       如需重启：bash ${BASH_SOURCE[0]} --restart" >&2
    exit 1
  fi
fi

# ---------------------------------------------------------------- 运行环境
# glibc-2.35 包装：SCAM SDK 需要比系统更新的 glibc，只对采集进程注入。
# 判定用 -f 而不是 -x：部署包在 Windows 上打包，NTFS 没有 unix 权限位，
# 包内文件解压后一律没有执行位，用 -x 会把「明明有」误判成「不存在」。
GLIBC235_DIR="${GLIBC235_DIR:-$VENDOR_DIR/glibc235/lib/aarch64-linux-gnu}"
if [[ ! -f "$GLIBC235_DIR/ld-linux-aarch64.so.1" ]]; then
  GLIBC235_DIR="/home/unitree/glibc235/lib/aarch64-linux-gnu"
elif [[ ! -x "$GLIBC235_DIR/ld-linux-aarch64.so.1" ]]; then
  chmod +x "$GLIBC235_DIR/ld-linux-aarch64.so.1" 2>/dev/null || true
fi

export XR_TELEOP_CERT="${XR_TELEOP_CERT:-${HOME}/.config/xr_teleoperate/cert.pem}"
export XR_TELEOP_KEY="${XR_TELEOP_KEY:-${HOME}/.config/xr_teleoperate/key.pem}"
export LD_PRELOAD="${CONDA_PREFIX}/lib/libgomp.so.1"
export GI_TYPELIB_PATH=/usr/lib/aarch64-linux-gnu/girepository-1.0

# ---- 启动（前台 exec 或 DAEMON=1 后台）----
launch() {
  if [[ "$DAEMON" == "1" ]]; then
    nohup "$@" >> "$LOG_FILE" 2>&1 &
    echo $! > "$PIDFILE"
    echo "[camera] 已后台启动 pid=$(cat "$PIDFILE")  log=$LOG_FILE"
  else
    exec "$@"
  fi
}

# 用 RealSense 前先确认 UVC 节点没被官方相机服务占着
if [[ "$DETECTED_KIND" == "realsense" ]]; then
  if ! release_camera; then
    exit 1
  fi
fi

if [[ "$CAM_BACKEND" == "scam" ]]; then
  if [[ ! -f "$GLIBC235_DIR/ld-linux-aarch64.so.1" ]]; then
    echo "[camera] 错误：未找到 glibc-2.35 加载器 $GLIBC235_DIR/ld-linux-aarch64.so.1" >&2
    echo "[camera]       SCAM 双目相机必须用它启动；缺文件请解压 Ubuntu 22.04 libc6 到 vendor/glibc235" >&2
    exit 1
  fi
  [[ -x "$GLIBC235_DIR/ld-linux-aarch64.so.1" ]] || chmod +x "$GLIBC235_DIR/ld-linux-aarch64.so.1" 2>/dev/null || true
  # teleimager-server 是 Python 脚本（非 ELF），必须由加载器先起 python，脚本作参数。
  launch env \
    LD_LIBRARY_PATH="$GLIBC235_DIR:$CONDA_PREFIX/lib" \
    LD_PRELOAD="$CONDA_PREFIX/lib/libgomp.so.1" \
    "$GLIBC235_DIR/ld-linux-aarch64.so.1" \
    "$CONDA_PREFIX/bin/python3.10" \
    "$CONDA_PREFIX/bin/teleimager-server" --no-affinity --config "$ACTIVE_CONFIG" $EXTRA_ARGS
else
  launch "$CONDA_PREFIX/bin/teleimager-server" --no-affinity --config "$ACTIVE_CONFIG" $EXTRA_ARGS
fi
