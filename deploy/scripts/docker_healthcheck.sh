#!/bin/bash
set -e

echo "=== xr_teleoperate Docker Health Check ==="

echo "[1] Python environment:"
python --version
pip list | grep -E 'torch|vuer|teleimager|televuer|unitree|pinocchio' || true

echo "[2] GPU:"
nvidia-smi || echo "WARNING: nvidia-smi failed"
python -c "import torch; print(f'CUDA: {torch.cuda.is_available()}')" || true

echo "[3] Certificates (project-builtin):"
test -f /workspace/certs/cert.pem && echo "cert.pem: OK" || echo "cert.pem: MISSING"
test -f /workspace/certs/key.pem && echo "key.pem: OK" || echo "key.pem: MISSING"
test -f /root/.config/xr_teleoperate/cert.pem && echo "user config cert: OK" || echo "user config cert: MISSING"

echo "[4] Network ports:"
netstat -tulpn 2>/dev/null | grep -E '8012|60001' || ss -tulpn | grep -E '8012|60001' || echo "Ports not yet listening (normal before startup)"

echo "[5] Video devices:"
ls -la /dev/video* 2>/dev/null || echo "No video devices"

echo "[6] Data directories:"
ls -la /workspace/data/ 2>/dev/null || echo "data/: missing"
ls -la /workspace/.config/xr_teleoperate/ 2>/dev/null || echo "config: missing"

echo "[7] Environment:"
echo "IMG_SERVER_IP=$IMG_SERVER_IP"
echo "XR_TELEOP_ARM=$XR_TELEOP_ARM"
echo "XR_TELEOP_CERT=$XR_TELEOP_CERT"

echo "=== Health check complete ==="
