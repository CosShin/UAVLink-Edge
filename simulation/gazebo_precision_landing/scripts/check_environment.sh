#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ROOT="$(cd "$SIM_DIR/../.." && pwd)"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"
ARDUPILOT_GAZEBO_HOME="${ARDUPILOT_GAZEBO_HOME:-$HOME/gz_ws/src/ardupilot_gazebo}"
failed=0
check_cmd() {
  if command -v "$1" >/dev/null 2>&1; then echo "[OK] $1: $(command -v "$1")";
  else echo "[MISSING] $1"; failed=1; fi
}
check_cmd gz
check_cmd gst-launch-1.0
check_cmd python3
test -x "$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py" \
  && echo "[OK] ArduPilot: $ARDUPILOT_HOME" \
  || { echo "[MISSING] ArduPilot: $ARDUPILOT_HOME"; failed=1; }
test -d "$ARDUPILOT_GAZEBO_HOME/models" \
  && echo "[OK] ardupilot_gazebo: $ARDUPILOT_GAZEBO_HOME" \
  || { echo "[MISSING] ardupilot_gazebo: $ARDUPILOT_GAZEBO_HOME"; failed=1; }
test -f "$SIM_DIR/models/aruco_landing_pad/materials/textures/aruco_board.png" \
  && echo "[OK] ArUco texture" \
  || { echo "[MISSING] chạy scripts/prepare_assets.sh"; failed=1; }
"$ROOT/venv/bin/python" -c "import cv2, pymavlink; print('[OK] Python detector dependencies')" || failed=1
if [ "$failed" -ne 0 ]; then echo "Môi trường chưa sẵn sàng; xem README.md." >&2; exit 1; fi

