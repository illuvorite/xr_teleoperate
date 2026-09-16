#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TEL_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
VENDOR_DIR="$TEL_DIR/vendor"

echo "[sync_vendor] 目标目录: $VENDOR_DIR"

CAMERA_SDK_SRC="${CAMERA_SDK_SRC:-/home/unitree/camera_sdk_test}"
if [[ -d "$CAMERA_SDK_SRC" ]]; then
  echo "[sync_vendor] 复制相机 SDK: $CAMERA_SDK_SRC -> $VENDOR_DIR/camera_sdk_test/"
  mkdir -p "$VENDOR_DIR/camera_sdk_test"
  cp -a "$CAMERA_SDK_SRC/." "$VENDOR_DIR/camera_sdk_test/"
else
  echo "[sync_vendor] 警告：未找到源目录 $CAMERA_SDK_SRC，跳过相机 SDK 复制"
fi

GLIBC235_SRC="${GLIBC235_SRC:-/home/unitree/glibc235}"
if [[ -d "$GLIBC235_SRC" ]]; then
  echo "[sync_vendor] 复制 glibc2.35: $GLIBC235_SRC -> $VENDOR_DIR/glibc235/"
  mkdir -p "$VENDOR_DIR/glibc235"
  cp -a "$GLIBC235_SRC/." "$VENDOR_DIR/glibc235/"
else
  echo "[sync_vendor] 警告：未找到源目录 $GLIBC235_SRC，跳过 glibc235 复制"
fi

echo "[sync_vendor] 完成。"
