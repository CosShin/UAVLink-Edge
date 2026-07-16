#!/usr/bin/env python3
"""Offline smoke test for the exact ArUco texture used by Gazebo."""
from __future__ import annotations
import json
import sys
from pathlib import Path
import cv2

SIM_DIR = Path(__file__).resolve().parents[1]
ROOT = SIM_DIR.parents[1]
FIND_LANDING = ROOT / "Find_landing"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(FIND_LANDING))
from processing.base import FrameMeta  # noqa: E402
from processing.detectors.aruco import create_processor  # noqa: E402


def main() -> int:
    config = json.loads((SIM_DIR / "config/camera_sim.json").read_text(encoding="utf-8"))
    texture = SIM_DIR / "models/aruco_landing_pad/materials/textures/aruco_board.png"
    frame = cv2.imread(str(texture))
    if frame is None:
        raise SystemExit(f"Không đọc được texture: {texture}")
    processor = create_processor(config, str(FIND_LANDING))
    result: dict = {}
    height, width = frame.shape[:2]
    for frame_id in range(1, int(config.get("aruco_acquire_frames", 5)) + 1):
        state = {"detection_result": {"detected": False}}
        processor.process(frame, FrameMeta(frame_id, (width, height)), state)
        result = state["detection_result"]
    expected = int(config["aruco_board_cols"]) * int(config["aruco_board_rows"])
    if not result.get("detected") or int(result.get("aruco_marker_count", 0)) != expected:
        raise SystemExit(f"FAIL: detector không thấy đủ board: {result}")
    if not result.get("control_valid"):
        raise SystemExit(f"FAIL: board chưa control_valid: {result}")
    print(f"PASS: texture Gazebo -> detector production | markers={expected} quality={float(result.get('quality', 0.0)):.3f} state={result.get('tracking_state')} control_valid=true")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

