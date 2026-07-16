#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ROOT="$(cd "$SIM_DIR/../.." && pwd)"
exec "$ROOT/venv/bin/python" "$SIM_DIR/scripts/vision_landing_bridge.py" \
  --config "$SIM_DIR/config/camera_sim.json" \
  --mavlink udpin:127.0.0.1:14551 --rtp-port 5600 \
  --hfov 60 --vfov 45 --rate 10 "$@"

