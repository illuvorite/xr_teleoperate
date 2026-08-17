#!/usr/bin/env bash
set -euo pipefail

TELEIMAGER_DIR=~/xr_teleoperate-main/teleop/teleimager
CONFIG_FILE="$TELEIMAGER_DIR/cam_config_server.yaml"
CONFIG_RS="$TELEIMAGER_DIR/cam_config_server_realsense.yaml"
CONFIG_OCV="$TELEIMAGER_DIR/cam_config_server_opencv.yaml"
CONFIG_UVC="$TELEIMAGER_DIR/cam_config_server_uvc.yaml"
LOG="$TELEIMAGER_DIR/teleimager_auto.log"
CACHE="$TELEIMAGER_DIR/.teleimager_last_type"

timestamp() {
  date '+%F %T'
}

log() {
  local msg="$1"
  echo "[$(timestamp)] $msg"
  echo "[$(timestamp)] $msg" >> "$LOG"
}

if [[ ! -f "$CONFIG_RS" || ! -f "$CONFIG_OCV" || ! -f "$CONFIG_UVC" ]]; then
  log "[错误] 缺少分流配置模板：$CONFIG_RS / $CONFIG_OCV / $CONFIG_UVC"
  exit 1
fi

save_type() {
  local type="$1"
  echo "$type" > "$CACHE"
}

read_type() {
  local type="unknown"
  if [[ -f "$CACHE" ]]; then
    type=$(cat "$CACHE" 2>/dev/null || echo unknown)
  fi
  echo "${type:-unknown}"
}

light_probe_realsense() {
  python3 -c "import pyrealsense2" 2>/dev/null && \
  ls /dev/video* >/dev/null 2>&1 && \
  teleimager-server --cf --rs 2>/dev/null | grep -q "RealSense serial numbers:"
}

light_probe_opencv() {
  teleimager-server --cf 2>/dev/null | grep -q "Found RGB video devices:"
}

light_probe_uvc() {
  teleimager-server --cf 2>/dev/null | grep -q "Found video devices:"
}

pick_best() {
  if light_probe_realsense; then
    echo realsense
  elif light_probe_opencv; then
    echo opencv
  elif light_probe_uvc; then
    echo uvc
  else
    echo unknown
  fi
}

apply_config() {
  local type="$1"
  case "$type" in
    realsense) cp "$CONFIG_RS" "$CONFIG_FILE" ;;
    opencv) cp "$CONFIG_OCV" "$CONFIG_FILE" ;;
    uvc) cp "$CONFIG_UVC" "$CONFIG_FILE" ;;
    *) return 1 ;;
  esac
  save_type "$type"
  log "[配置] 已切换到 $type"
}

stop_teleimager() {
  pkill -f "teleimager-server" || true
  sleep 1
}

start_teleimager() {
  log "[启动] 正在启动 teleimager-server --rs"
  (cd "$TELEIMAGER_DIR" && nohup teleimager-server --rs >> "$LOG" 2>&1 &)
}

current_type() {
  grep -E '^[[:space:]]*type:' "$CONFIG_FILE" | head -n1 | awk '{print $2}' || echo unknown
}

handle_switch() {
  local target="$1"
  local current
  current=$(current_type)

  if [[ "$target" == "$current" ]]; then
    log "[切换] 当前已经是 $target，无需切换"
    return
  fi

  log "[切换] 手动切换到 $target"
  apply_config "$target"
  stop_teleimager
  start_teleimager
}

main_loop() {
  local last_type
  last_type=$(read_type)

  while true; do
    sleep 15
    best=$(pick_best || echo unknown)
    current=$(current_type)

    if [[ "$best" == unknown ]]; then
      continue
    fi

    if [[ "$best" != "$current" ]]; then
      log "[自动] $current -> $best"
      apply_config "$best"
      stop_teleimager
      start_teleimager
    fi
  done
}

log "[调试] 分流监控已启动"
best=$(pick_best || echo unknown)
log "[调试] 启动时检测结果：$best"

if [[ "$best" != unknown ]]; then
  apply_config "$best"
else
  log "[警告] 未检测到可用相机，保持现有配置"
fi

start_teleimager

case "${1:-}" in
  switch)
    if [[ $# -ge 2 ]]; then
      handle_switch "$2"
    else
      log "[用法] 缺少切换目标：realsense | opencv | uvc"
    fi
    ;;
  *)
    main_loop
    ;;
esac