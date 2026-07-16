#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ARDUPILOT_GAZEBO_HOME="${ARDUPILOT_GAZEBO_HOME:-$HOME/gz_ws/src/ardupilot_gazebo}"
command -v gz >/dev/null 2>&1 || {
  echo "Không tìm thấy 'gz'. Hãy cài Gazebo Harmonic trên Ubuntu 22.04 trước." >&2
  exit 1
}
export GZ_VERSION="${GZ_VERSION:-harmonic}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_HOME/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_SIM_RESOURCE_PATH="$SIM_DIR/models:$SIM_DIR/worlds:$ARDUPILOT_GAZEBO_HOME/models:$ARDUPILOT_GAZEBO_HOME/worlds:${GZ_SIM_RESOURCE_PATH:-}"
exec gz sim -v4 -r "$SIM_DIR/worlds/precision_landing.sdf"

