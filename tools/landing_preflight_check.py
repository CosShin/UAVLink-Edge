#!/usr/bin/env python3
"""Read-only preflight checks for the USB-camera precision-landing pipeline."""

from __future__ import annotations

import argparse
import json
import time
from pathlib import Path

import yaml


ROOT = Path(__file__).resolve().parents[1]


def main() -> int:
    parser = argparse.ArgumentParser(description="Landing vision preflight checker")
    parser.add_argument("--config", default=str(ROOT / "config.yaml"))
    parser.add_argument("--camera-id", type=int, default=0)
    parser.add_argument("--require-live", action="store_true")
    parser.add_argument(
        "--probe-camera",
        action="store_true",
        help="Open the webcam briefly; stop the streamer first to avoid device contention",
    )
    args = parser.parse_args()

    config_path = Path(args.config).resolve()
    cfg = yaml.safe_load(config_path.read_text(encoding="utf-8")) or {}
    streams = cfg.get("camera", {}).get("streams", [])
    stream = next(
        (item for item in streams if int(item.get("camera_id", -1)) == args.camera_id),
        None,
    )
    checks: list[tuple[str, str, str]] = []

    def add(level: str, name: str, detail: str):
        checks.append((level, name, detail))

    if stream is None:
        add("ERROR", "camera config", f"camera_id={args.camera_id} not found")
    else:
        calibration_ok = False
        source = str(stream.get("source") or "csi").lower()
        add("OK" if source == "usb" else "ERROR", "camera source", source)
        size = tuple(int(v) for v in (stream.get("size") or []))
        add("OK" if size == (1280, 720) else "ERROR", "stream size", str(size))
        mode = str(stream.get("landing_detection_mode") or "")
        add("OK" if mode == "aruco" else "ERROR", "detector", mode or "unset")
        strategy = str(stream.get("aruco_target_strategy") or "single")
        min_markers = int(stream.get("aruco_board_min_markers", 2) or 2)
        strategy_ok = strategy in ("single", "board") and (
            strategy != "board" or min_markers >= 2
        )
        add(
            "OK" if strategy_ok else "ERROR",
            "target strategy",
            f"{strategy}; board_min_markers={min_markers}",
        )
        add(
            "OK",
            "vision gate",
            f"quality>={float(stream.get('aruco_min_quality', 0.55)):.2f}; "
            f"acquire={int(stream.get('aruco_acquire_frames', 5))} frames",
        )
        device = Path(str(stream.get("device_path") or "/dev/video0"))
        exists = device.exists()
        level = "OK" if exists else ("ERROR" if args.require_live else "WARN")
        add(level, "webcam device", f"{device} {'exists' if exists else 'not found'}")

        calibration_value = str(stream.get("aruco_calibration_file") or "").strip()
        marker_length = float(stream.get("aruco_marker_length_m", 0.0) or 0.0)
        if calibration_value:
            calibration = Path(calibration_value).expanduser()
            if not calibration.is_absolute():
                calibration = ROOT / "Find_landing" / calibration
            add(
                "OK" if calibration.is_file() else "ERROR",
                "camera calibration",
                str(calibration),
            )
            calibration_ok = calibration.is_file()
        else:
            level = "ERROR" if marker_length > 0 else "WARN"
            add(level, "camera calibration", "not configured; angle-only mode")

        if args.probe_camera:
            import cv2

            cap = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
            cap.set(cv2.CAP_PROP_FPS, int(stream.get("framerate", 30) or 30))
            cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
            ok, frame = cap.read() if cap.isOpened() else (False, None)
            cap.release()
            actual = None if frame is None else (frame.shape[1], frame.shape[0])
            add(
                "OK" if ok and actual == (1280, 720) else "ERROR",
                "live webcam frame",
                str(actual or "capture failed"),
            )

    landing = cfg.get("landing", {})
    mavlink_enabled = bool(landing.get("mavlink_enabled", False))
    hfov = float(landing.get("camera_hfov_deg", 0) or 0)
    vfov = float(landing.get("camera_vfov_deg", 0) or 0)
    if mavlink_enabled:
        fov_ok = 1 <= hfov < 179 and 1 <= vfov < 179
        derived = bool(stream is not None and calibration_ok)
        add(
            "OK" if fov_ok or derived else "ERROR",
            "LANDING_TARGET output",
            (
                f"enabled; FOV={hfov}x{vfov} deg"
                if fov_ok
                else "enabled; FOV will be derived from camera calibration"
            ),
        )
    else:
        add("WARN", "LANDING_TARGET output", "disabled (safe bench default)")

    telemetry_path = Path(f"/tmp/camera_landing_{args.camera_id}.json")
    try:
        telemetry = json.loads(telemetry_path.read_text(encoding="utf-8"))
        age = time.time() - float(telemetry.get("updated_at", 0))
        fresh = 0 <= age <= 2.0
        level = "OK" if fresh else ("ERROR" if args.require_live else "WARN")
        add(
            level,
            "vision telemetry",
            f"age={age:.2f}s; state={telemetry.get('tracking_state')}; "
            f"control_valid={bool(telemetry.get('control_valid'))}",
        )
    except (OSError, ValueError, TypeError, json.JSONDecodeError):
        level = "ERROR" if args.require_live else "WARN"
        add(level, "vision telemetry", f"no valid {telemetry_path}")

    icons = {"OK": "[OK]", "WARN": "[WARN]", "ERROR": "[ERROR]"}
    for level, name, detail in checks:
        print(f"{icons[level]:7} {name}: {detail}")
    errors = sum(level == "ERROR" for level, _, _ in checks)
    warnings = sum(level == "WARN" for level, _, _ in checks)
    print(f"Summary: errors={errors}, warnings={warnings}")
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
