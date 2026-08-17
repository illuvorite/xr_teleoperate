#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

conda activate tv
echo "[teleimager] activated conda env: $(conda info --envs | grep '\*' | awk '{print $1}')"

cd "$REPO_ROOT/teleop/teleimager"

export XR_TELEOP_CERT="${XR_TELEOP_CERT:-${HOME}/.config/xr_teleoperate/cert.pem}"
export XR_TELEOP_KEY="${XR_TELEOP_KEY:-${HOME}/.config/xr_teleoperate/key.pem}"

teleimager-server --rs &
echo '123' | sudo -S modprobe -r uvcvideo || true
echo '123' | sudo -S modprobe uvcvideo debug=0
wait
