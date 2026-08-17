#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

conda activate tv
echo "[xr_teleop] activated conda env: $(conda info --envs | grep '\*' | awk '{print $1}')"

cd "$REPO_ROOT"

python teleop/teleop_hand_and_arm.py \
  --arm G1_29 \
  --input-mode controller \
  --display-mode immersive \
  --network-interface eth0 \
  --img-server-ip 192.168.2.203 \
  --motion \
  --static-dashboard
