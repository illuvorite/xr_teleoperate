#!/usr/bin/env bash
# start_teleimager.sh — teleimager 图像服务启动脚本（2026-08-28 优化版）
#
# 主要变化 vs 旧版：
#   P0 重复启动保护：已在运行则拒绝；--restart 先停旧实例再启
#   P0 删除与注释自相矛盾的 v4l2-ctl 硬编码段（Camera2201 在 /dev/video6，且 SDK 路径不需要）
#   P1 uvcvideo 重载改为可选（RELOAD_UVC=1 才重载，默认不重载；用 sudo -n 非交互）
#   P2 后台模式 DAEMON=1（nohup + pid 文件）；日志路径可配 LOG_FILE
#   P2 支持 CAM_BACKEND=scam：启用进程级 glibc-2.35 包装加载器（方案文档 §1.4B）
#
# 用法：
#   bash scripts/start_teleimager.sh                 # 前台启动（默认，与原行为一致）
#   bash scripts/start_teleimager.sh --restart       # 停掉旧实例后启动
#   RELOAD_UVC=1 bash scripts/start_teleimager.sh    # 启动前重载 uvcvideo 驱动
#   DAEMON=1 bash scripts/start_teleimager.sh        # nohup 后台启动
#   CAM_BACKEND=scam bash scripts/start_teleimager.sh  # 用 glibc-2.35 包装运行（SCAM SDK）
#   bash scripts/start_teleimager.sh                 # 默认按 cam_config_server.yaml 的 type 自动选择
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
TEL_DIR="$REPO_ROOT/teleop/teleimager"
PIDFILE="$TEL_DIR/.teleimager.pid"
LOG_FILE="${LOG_FILE:-$TEL_DIR/teleimager_auto.log}"

# ---- 参数 / 环境变量默认值 ----
RESTART=""
[[ "${1:-}" == "--restart" ]] && RESTART=1
DAEMON="${DAEMON:-0}"               # 1 => nohup 后台
RELOAD_UVC="${RELOAD_UVC:-0}"       # 1 => modprobe -r + 重载 uvcvideo（默认不重载）
CAM_BACKEND="${CAM_BACKEND:-auto}" # auto | opencv | scam

# 默认根据当前 teleimager 配置选择后端，避免 type: scam 时误用系统 glibc 启动。
# 显式 CAM_BACKEND=opencv/scam 仍可覆盖自动判断。
if [[ "$CAM_BACKEND" == "auto" ]]; then
  CONFIG_FILE="$TEL_DIR/cam_config_server.yaml"
  if [[ ! -f "$CONFIG_FILE" ]]; then
    echo "[teleimager] 错误：配置文件不存在：$CONFIG_FILE" >&2
    exit 1
  fi
  if grep -Eq '^[[:space:]]*type:[[:space:]]*scam[[:space:]]*(#.*)?$' "$CONFIG_FILE"; then
    CAM_BACKEND=scam
  else
    CAM_BACKEND=opencv
  fi
fi

# ---- 相机可用性检测：SCAM 立体相机不可用时自动回退到 RealSense 深度相机 ----
#
# ⚠ 必须在当前 shell 直接调用，不要写成 ACTIVE_CONFIG=$(detect_camera_fallback ...)：
#   命令替换会让整个函数跑在子 shell 里，函数内对 CAM_BACKEND / EXTRA_ARGS / ACTIVE_CONFIG
#   的赋值不会传回父 shell —— 表现是：日志已经打了"自动回退到 RealSense 配置"，但父 shell
#   里 backend 仍是 scam、配置路径仍是空的，teleimager 最终按内置默认的 SCAM 配置启动而反复崩溃。
#   结果通过 ACTIVE_CONFIG / CAM_BACKEND / EXTRA_ARGS 三个全局变量返回。
#
# 另外这里刻意不用 python3+pyyaml 解析：conda 环境在本步之后才激活，系统 python3 不一定装了
# yaml，解析失败会静默跳过检测（cam_type 取到空串即当成"非 scam"直接返回），改用 sed 读取。
detect_camera_fallback() {
  local config_file="$1"
  ACTIVE_CONFIG="$config_file"

  local cam_type
  cam_type=$(sed -n 's/^[[:space:]]*type:[[:space:]]*\([A-Za-z0-9_]*\).*/\1/p' "$config_file" | head -n1 | tr '[:upper:]' '[:lower:]')

  if [[ "$cam_type" != "scam" ]]; then
    return 0
  fi

  local video_id physical_path
  video_id=$(sed -n 's/^[[:space:]]*video_id:[[:space:]]*\([0-9]\+\).*/\1/p' "$config_file" | head -n1)
  physical_path=$(sed -n 's/^[[:space:]]*physical_path:[[:space:]]*\([^[:space:]]*\).*/\1/p' "$config_file" | head -n1 | tr -d "'\"")

  local stereo_available=0
  if [[ -n "$video_id" && -e "/dev/video${video_id}" ]]; then
    stereo_available=1
  fi
  if [[ "$stereo_available" -eq 0 && -n "$physical_path" && -e "$physical_path" ]]; then
    stereo_available=1
  fi

  if [[ "$stereo_available" -eq 1 ]]; then
    return 0
  fi

  echo "[teleimager] 警告：SCAM 立体相机未检测到 (video_id=${video_id:-null}, physical_path=${physical_path:-null})" >&2
  local rs_config="$TEL_DIR/cam_config_server_realsense.yaml"
  if [[ ! -f "$rs_config" ]]; then
    echo "[teleimager] 错误：未找到回退配置 $rs_config" >&2
    return 1
  fi

  echo "[teleimager] 自动回退到 RealSense 深度相机配置: $rs_config" >&2
  ACTIVE_CONFIG="$rs_config"
  CAM_BACKEND=opencv
  EXTRA_ARGS="--rs"
  return 0
}

EXTRA_ARGS=""
ACTIVE_CONFIG="$TEL_DIR/cam_config_server.yaml"
if ! detect_camera_fallback "$ACTIVE_CONFIG"; then
  exit 1
fi
echo "[teleimager] 使用配置: $ACTIVE_CONFIG  backend=$CAM_BACKEND ${EXTRA_ARGS}" >&2

if [[ "$CAM_BACKEND" != "opencv" && "$CAM_BACKEND" != "scam" ]]; then
  echo "[teleimager] 错误：CAM_BACKEND 仅支持 auto|opencv|scam（当前 $CAM_BACKEND）" >&2
  exit 1
fi

# ---- 重复启动保护（匹配真实服务进程 cmdline，避免匹配到本脚本/外层 shell 自身的命令串）----
if pgrep -f "teleimager-server --no-affinity" >/dev/null 2>&1; then
  if [[ -n "$RESTART" ]]; then
    echo "[teleimager] 检测到已在运行（--restart）：停止旧实例..."
    pkill -f "teleimager-server --no-affinity" || true
    sleep 2
  else
    echo "[teleimager] 错误：teleimager-server 已在运行。" >&2
    echo "[teleimager] 如需重启请用: bash ${BASH_SOURCE[0]} --restart（或先 pkill -f 'teleimager-server --no-affinity'）" >&2
    exit 1
  fi
fi

# ---- Vendor 路径（优先使用项目内 vendor，环境变量可覆盖）----
VENDOR_DIR="${VENDOR_DIR:-$TEL_DIR/vendor}"
CAMERA_SDK_DIR="${CAMERA_SDK_DIR:-$VENDOR_DIR/camera_sdk_test}"
GLIBC235_DIR="${GLIBC235_DIR:-$VENDOR_DIR/glibc235/lib/aarch64-linux-gnu}"
export VENDOR_DIR CAMERA_SDK_DIR GLIBC235_DIR

# 回退：若 vendor 内缺失，仍尝试系统路径（兼容旧部署）
if [[ ! -d "$CAMERA_SDK_DIR/build" ]]; then
  echo "[teleimager] 警告：项目内 vendor/camera_sdk_test 不存在，回退到 /home/unitree/camera_sdk_test" >&2
  CAMERA_SDK_DIR="/home/unitree/camera_sdk_test"
fi
# 判定"包内是否带了 glibc235"必须用 -f，不能用 -x：
# 部署包在 Windows 上打包，NTFS 没有 unix 权限位，包内文件一律是 666（目录 777），
# 解压后 ld-linux 没有执行位 —— 用 -x 判断会把"包内明明有"误判成"不存在"而回退。
# 位不对时自己补上（文件属主是部署用户，chmod 不需要 root）。
if [[ ! -f "$GLIBC235_DIR/ld-linux-aarch64.so.1" ]]; then
  echo "[teleimager] 警告：项目内 vendor/glibc235 不存在，回退到 /home/unitree/glibc235" >&2
  GLIBC235_DIR="/home/unitree/glibc235/lib/aarch64-linux-gnu"
elif [[ ! -x "$GLIBC235_DIR/ld-linux-aarch64.so.1" ]]; then
  chmod +x "$GLIBC235_DIR/ld-linux-aarch64.so.1" 2>/dev/null || true
fi

# ---- conda 环境（多候选路径发现，不依赖交互 shell 的 PATH）----
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
  echo "[teleimager] 错误：找不到 conda（已尝试 PATH/miniconda3//opt/miniconda3），可用 CONDA_BASE= 指定" >&2
  exit 1
fi
source "$CONDA_BASE/etc/profile.d/conda.sh"
conda activate tv
echo "[teleimager] conda env: ${CONDA_DEFAULT_ENV:-unknown}  backend=$CAM_BACKEND"

export PYTHONPATH="$REPO_ROOT/teleop/teleimager/src:$REPO_ROOT${PYTHONPATH:+:$PYTHONPATH}"
cd "$REPO_ROOT"

export XR_TELEOP_CERT="${XR_TELEOP_CERT:-${HOME}/.config/xr_teleoperate/cert.pem}"
export XR_TELEOP_KEY="${XR_TELEOP_KEY:-${HOME}/.config/xr_teleoperate/key.pem}"

# ---- uvcvideo 重载（默认不重载；重载会重排 /dev/videoN 节点并瞬间断流）----
if [[ "$RELOAD_UVC" == "1" ]]; then
  if command -v sudo >/dev/null 2>&1; then
    echo "[teleimager] 重载 uvcvideo 驱动 (RELOAD_UVC=1)"
    sudo -n modprobe -r uvcvideo || echo "[teleimager] 警告：sudo modprobe -r uvcvideo 失败（忽略）"
    sudo -n modprobe uvcvideo debug=0 || echo "[teleimager] 警告：sudo modprobe uvcvideo 失败（忽略）"
  else
    echo "[teleimager] 警告：无 sudo，跳过 uvcvideo 重载"
  fi
fi

export LD_PRELOAD="${CONDA_PREFIX}/lib/libgomp.so.1"
export GI_TYPELIB_PATH=/usr/lib/aarch64-linux-gnu/girepository-1.0

# ---- 启动（前台 exec 或 DAEMON=1 后台）----
launch() {
  if [[ "$DAEMON" == "1" ]]; then
    nohup "$@" >> "$LOG_FILE" 2>&1 &
    echo $! > "$PIDFILE"
    echo "[teleimager] 已后台启动 pid=$(cat "$PIDFILE")  log=$LOG_FILE"
  else
    exec "$@"
  fi
}

if [[ "$CAM_BACKEND" == "scam" ]]; then
  # SCAM SDK 需要 glibc-2.35 包装（方案文档 §1.4B）：仅对捕获进程注入，勿 export 到整个脚本。
  # teleimager-server 是 Python 脚本（非 ELF），必须由加载器先启动 python3.10、脚本作参数。
  GLIBC_235="${GLIBC_235:-$GLIBC235_DIR}"
  # 同前：包内文件没有执行位，判定用 -f，然后自己补 +x。
  if [[ ! -f "$GLIBC_235/ld-linux-aarch64.so.1" ]]; then
    echo "[teleimager] 错误：未找到 glibc-2.35 包装 $GLIBC_235/ld-linux-aarch64.so.1" >&2
    echo "[teleimager] 请先按方案文档 §1.4B 解压 Ubuntu 22.04 libc6 到项目 vendor 目录或 ~/glibc235" >&2
    exit 1
  fi
  [[ -x "$GLIBC_235/ld-linux-aarch64.so.1" ]] || chmod +x "$GLIBC_235/ld-linux-aarch64.so.1" 2>/dev/null || true
  launch env \
    LD_LIBRARY_PATH="$GLIBC_235:$CONDA_PREFIX/lib" \
    LD_PRELOAD="$CONDA_PREFIX/lib/libgomp.so.1" \
    "$GLIBC_235/ld-linux-aarch64.so.1" \
    "$CONDA_PREFIX/bin/python3.10" \
    "$CONDA_PREFIX/bin/teleimager-server" --no-affinity --config "$ACTIVE_CONFIG" $EXTRA_ARGS
else
  launch "$CONDA_PREFIX/bin/teleimager-server" --no-affinity --config "$ACTIVE_CONFIG" $EXTRA_ARGS
fi