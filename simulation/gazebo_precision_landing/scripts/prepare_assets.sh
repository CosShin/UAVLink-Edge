#!/usr/bin/env bash
set -euo pipefail
SIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
ROOT="$(cd "$SIM_DIR/../.." && pwd)"
SRC="$ROOT/Find_landing/templates/aruco_board_dict_4x4_50_0-11.png"
DST="$SIM_DIR/models/aruco_landing_pad/materials/textures/aruco_board.png"
test -f "$SRC" || { echo "Không tìm thấy board: $SRC" >&2; exit 1; }
cp "$SRC" "$DST"
echo "Đã chuẩn bị texture: $DST"

