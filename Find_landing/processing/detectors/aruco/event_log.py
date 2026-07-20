"""Small rotating JSONL event log for landing vision diagnostics."""

from __future__ import annotations

import json
import os
import time
from pathlib import Path


class LandingEventLogger:
    def __init__(self, camera_id: int, *, max_bytes: int = 5_000_000, sample_hz: float = 2.0):
        self.path = Path(f"/tmp/camera_landing_events_{int(camera_id)}.jsonl")
        self.max_bytes = max(100_000, int(max_bytes))
        self.sample_interval = 1.0 / max(float(sample_hz), 0.1)
        self._last_sample = 0.0
        self._last_state = ""

    def write(self, detection: dict, frame_id: int) -> None:
        now = time.time()
        state = str(detection.get("tracking_state") or "")
        if state == self._last_state and now - self._last_sample < self.sample_interval:
            return
        self._last_state = state
        self._last_sample = now
        try:
            if self.path.exists() and self.path.stat().st_size >= self.max_bytes:
                rotated = self.path.with_suffix(self.path.suffix + ".1")
                try:
                    os.replace(self.path, rotated)
                except OSError:
                    pass
            payload = {
                "timestamp": now,
                "frame_id": int(frame_id),
                "state": state,
                "detected": bool(detection.get("detected")),
                "control_valid": bool(detection.get("control_valid")),
                "ambiguous": bool(detection.get("ambiguous")),
                "reason": detection.get("tracking_reason") or detection.get("reason"),
                "target_key": detection.get("target_key"),
                "locked_target": detection.get("locked_target"),
                "visible_ids": detection.get("aruco_visible_ids"),
                "duplicate_ids": detection.get("duplicate_ids"),
                "quality": detection.get("quality"),
                "reprojection_error_px": detection.get("reprojection_error_px"),
                "offset_x": detection.get("offset_x"),
                "offset_y": detection.get("offset_y"),
                "pose_camera_m": detection.get("pose_camera_m"),
                "target_center_camera_m": detection.get("target_center_camera_m"),
                "camera_to_target_distance_m": detection.get("camera_to_target_distance_m"),
                "camera_to_target_depth_m": detection.get("camera_to_target_depth_m"),
            }
            with self.path.open("a", encoding="utf-8") as handle:
                handle.write(json.dumps(payload, separators=(",", ":")) + "\n")
        except OSError:
            pass
