#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ARDUPILOT_HOME="${ARDUPILOT_HOME:-$HOME/ardupilot}"
SIM_VEHICLE="$ARDUPILOT_HOME/Tools/autotest/sim_vehicle.py"
test -x "$SIM_VEHICLE" || { echo "Không tìm thấy $SIM_VEHICLE" >&2; exit 1; }
cd "$ARDUPILOT_HOME"
exec "$SIM_VEHICLE" -v ArduCopter -f gazebo-iris --model JSON --console --map \
  --add-param-file="$SIM_DIR/config/precision_landing.parm"

