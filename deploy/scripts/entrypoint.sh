#!/bin/bash
set -euo pipefail

echo "=========================================="
echo " xr_teleoperate Docker Entrypoint"
echo "=========================================="

# ---- Auto-detect host IP if IMG_SERVER_IP is not set ----
if [ -z "${IMG_SERVER_IP:-}" ]; then
    # Try to detect the primary IP address
    DETECTED_IP=$(ip -4 addr show | grep -oP '(?<=inet\s)\d+(\.\d+){3}' | grep -v '127.0.0.1' | head -n 1)
    if [ -n "$DETECTED_IP" ]; then
        echo "[entrypoint] Auto-detected host IP: ${DETECTED_IP}"
        export IMG_SERVER_IP="$DETECTED_IP"
    else
        echo "[entrypoint] WARNING: Could not auto-detect IP, using fallback 192.168.2.203"
        export IMG_SERVER_IP="192.168.2.203"
    fi
else
    echo "[entrypoint] Using configured IMG_SERVER_IP: ${IMG_SERVER_IP}"
fi

# Ensure certificate symlinks exist (project-builtin certs)
mkdir -p /root/.config/xr_teleoperate
if [ -f /workspace/certs/cert.pem ] && [ -f /workspace/certs/key.pem ]; then
    ln -sf /workspace/certs/cert.pem /root/.config/xr_teleoperate/cert.pem
    ln -sf /workspace/certs/key.pem /root/.config/xr_teleoperate/key.pem
    echo "[entrypoint] Certificates linked to ~/.config/xr_teleoperate/"
else
    echo "[entrypoint] WARNING: /workspace/certs/ missing; relying on module fallback"
fi

# Ensure runtime directories exist
mkdir -p /workspace/data /workspace/guidelogs

# Show GPU status
if command -v nvidia-smi >/dev/null 2>&1; then
    echo "[entrypoint] GPU detected:"
    nvidia-smi -L || true
else
    echo "[entrypoint] WARNING: nvidia-smi not found; GPU may not be available"
fi

# Show certificate info
echo "[entrypoint] XR_TELEOP_CERT=${XR_TELEOP_CERT:-<not set>}"
echo "[entrypoint] XR_TELEOP_KEY=${XR_TELEOP_KEY:-<not set>}"
echo "[entrypoint] IMG_SERVER_IP=${IMG_SERVER_IP}"

echo "=========================================="
echo " Starting xr_teleoperate..."
echo "=========================================="

# Execute the command passed to the container
exec "$@"
