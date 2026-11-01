#!/usr/bin/env bash
# Compatibility entry point; camera startup is maintained in scripts/.
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$REPO_ROOT/scripts/start_camera.sh" "$@"
