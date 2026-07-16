"""Publisher stats — /tmp/camera_stream_stats_{id}.json + landing telemetry."""

import json
import os
import time


def stats_path(camera_id: int) -> str:
    return f"/tmp/camera_stream_stats_{camera_id}.json"


def landing_path(camera_id: int) -> str:
    return f"/tmp/camera_landing_{camera_id}.json"


def write_stats(config: dict, frames_sent: int, start_time: float,
                capture_fps: float, encode_drops: int, window_fps: float):
    try:
        payload = {
            "camera_id": config.get("camera_id", 0),
            "fps_sent": round(frames_sent / max(time.time() - start_time, 0.001), 1),
            "fps_window": round(window_fps, 1),
            "fps_capture": round(capture_fps, 1),
            "frames_sent": frames_sent,
            "encode_drops": encode_drops,
            "updated_at": time.time(),
        }
        path = stats_path(int(config.get("camera_id", 0)))
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f)
        os.replace(tmp, path)
    except Exception:
        pass


def write_landing_telemetry(
    camera_id: int,
    detection: dict,
    detections_count: int,
    frame_size: tuple[int, int] | list[int] | None = None,
):
    """Expose Hướng 2 snapshot for REST / MAVLink bridge (IV-1, IV-2)."""
    try:
        det = detection or {"detected": False}
        payload = {
            "camera_id": camera_id,
            "detected": bool(det.get("detected")),
            "hold": bool(det.get("hold")),
            "hold_age_ms": det.get("hold_age_ms"),
            "offset_x": det.get("offset_x"),
            "offset_y": det.get("offset_y"),
            "h_size": det.get("h_size"),
            "direction": det.get("direction"),
            "similarity": det.get("similarity"),
            "quality": det.get("quality"),
            "quality_threshold": det.get("quality_threshold"),
            "quality_details": det.get("quality_details"),
            "control_valid": bool(det.get("control_valid")),
            "tracking_state": det.get("tracking_state"),
            "tracking_reason": det.get("tracking_reason"),
            "target_key": det.get("target_key"),
            "locked_target": det.get("locked_target"),
            "ambiguous": bool(det.get("ambiguous")),
            "duplicate_ids": det.get("duplicate_ids"),
            "aruco_id": det.get("aruco_id"),
            "aruco_visible_ids": det.get("aruco_visible_ids"),
            "aruco_marker_count": det.get("aruco_marker_count"),
            "reprojection_error_px": det.get("reprojection_error_px"),
            "pose_valid": bool(det.get("pose_valid")),
            "pose_camera_m": det.get("pose_camera_m"),
            "rvec": det.get("rvec"),
            "frame_id": det.get("frame_id"),
            "measurement_monotonic_ms": det.get("measurement_monotonic_ms"),
            "detections_count": detections_count,
            "updated_at": time.time(),
        }
        if frame_size and len(frame_size) >= 2:
            payload["frame_width"] = int(frame_size[0])
            payload["frame_height"] = int(frame_size[1])
        path = landing_path(camera_id)
        tmp = path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(payload, f)
        os.replace(tmp, path)
    except Exception:
        pass
