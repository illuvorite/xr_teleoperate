#!/bin/bash
# ============================================================
# Generate Apple Vision Pro CA certificate chain
# ============================================================
# This script generates a CA-signed certificate for AVP.
# After running, copy rootCA.pem to AVP via AirDrop and install it.
# ============================================================

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CERT_DIR="${REPO_ROOT}/teleop/televuer"

echo "[avp-cert] Generating Apple Vision Pro certificate chain..."
echo "[avp-cert] Cert directory: ${CERT_DIR}"

cd "${CERT_DIR}"

# 1. Generate root CA
echo "[avp-cert] Step 1/4: Generating root CA key..."
openssl genrsa -out rootCA.key 2048

echo "[avp-cert] Step 2/4: Generating root CA certificate..."
openssl req -x509 -new -nodes -key rootCA.key -sha256 -days 365 \
    -out rootCA.pem -subj "/CN=xr-teleoperate"

# 2. Generate server key and CSR
echo "[avp-cert] Step 3/4: Generating server key and CSR..."
openssl genrsa -out key.pem 2048
openssl req -new -key key.pem -out server.csr -subj "/CN=localhost"

# 3. Create SAN config with host IPs
echo "[avp-cert] Step 4/4: Signing server certificate..."
HOST_IP=$(hostname -I | awk '{print $1}')
echo "[avp-cert] Detected host IP: ${HOST_IP}"

cat > server_ext.cnf <<EOF
subjectAltName = @alt_names
[alt_names]
DNS.1 = localhost
IP.1 = ${HOST_IP}
IP.2 = 192.168.123.2
IP.3 = 192.168.123.164
EOF

openssl x509 -req -in server.csr -CA rootCA.pem -CAkey rootCA.key \
    -CAcreateserial -out cert.pem -days 365 -sha256 -extfile server_ext.cnf

echo ""
echo "=========================================="
echo " Certificate generation complete!"
echo "=========================================="
echo ""
echo "Files generated in ${CERT_DIR}:"
ls -la *.pem *.key *.csr *.cnf *.srl 2>/dev/null || true
echo ""
echo "NEXT STEPS:"
echo "1. Copy rootCA.pem to your Apple Vision Pro via AirDrop"
echo "2. On AVP: Settings → General → About → Certificate Trust Settings"
echo "3. Enable full trust for 'xr-teleoperate' root certificate"
echo "4. Restart the Safari browser on AVP"
echo "5. Connect to: https://${HOST_IP}:8012/?ws=wss://${HOST_IP}:8012"
echo ""
echo "NOTE: These certificates are also used by the Docker container."
echo "      The deploy/Dockerfile copies them to /workspace/certs/ automatically."
