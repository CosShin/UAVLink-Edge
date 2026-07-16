#!/usr/bin/env python3
"""Gazebo RTP camera -> production ArUco detector -> SITL LANDING_TARGET."""
from __future__ import annotations

import argparse
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from pymavlink import mavutil

SIM_DIR = Path(__file__).resolve().parents[1]
ROOT = SIM_DIR.parents[1]
FIND_LANDING = ROOT / "Find_landing"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(FIND_LANDING))

from landing_mavlink import _landing_target_from_telemetry  # noqa: E402
from processing.base import FrameMeta  # noqa: E402
from processing.detectors.aruco import create_processor, draw_overlay  # noqa: E402


def loopback_endpoint(value: str) -> str:
    allowed = (
        "udpin:127.0.0.1:", "udpin:localhost:", "udpout:127.0.0.1:",
        "udpout:localhost:", "tcp:127.0.0.1:", "tcp:localhost:",
    )
    if not value.startswith(allowed):
        raise argparse.ArgumentTypeError("SITL bridge chỉ cho phép endpoint loopback")
    return value


def gstreamer_command(port: int, width: int, height: int) -> list[str]:
    caps = "application/x-rtp,media=video,clock-rate=90000,encoding-name=H264"
    return [
        "gst-launch-1.0", "-q", "udpsrc", f"port={port}", f"caps={caps}",
        "!", "rtpjitterbuffer", "latency=20", "drop-on-latency=true",
        "!", "rtph264depay", "!", "avdec_h264", "!", "videoconvert",
        "!", "videoscale", "!", f"video/x-raw,format=BGR,width={width},height={height}",
        "!", "fdsink", "fd=1",
    ]


def read_exact(stream, length: int) -> bytes | None:
    chunks: list[bytes] = []
    remaining = length
    while remaining:
        chunk = stream.read(remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


def main() -> int:
    parser = argparse.ArgumentParser(description="Gazebo camera precision landing bridge")
    parser.add_argument("--config", default=str(SIM_DIR / "config/camera_sim.json"))
    parser.add_argument("--mavlink", type=loopback_endpoint, default="udpin:127.0.0.1:14551")
    parser.add_argument("--rtp-port", type=int, default=5600)
    parser.add_argument("--hfov", type=float, default=60.0)
    parser.add_argument("--vfov", type=float, default=45.0)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--min-quality", type=float, default=0.55)
    parser.add_argument("--preview", action="store_true", help="Hiện overlay; q/Esc để dừng")
    args = parser.parse_args()

    config = json.loads(Path(args.config).read_text(encoding="utf-8"))
    width, height = (int(v) for v in config.get("size", [1280, 720]))
    if not (1 <= args.hfov < 179 and 1 <= args.vfov < 179):
        raise SystemExit("FOV phải nằm trong khoảng 1..179 độ")

    conn = mavutil.mavlink_connection(args.mavlink, source_system=245, source_component=191)
    print(f"Chờ SITL heartbeat tại {args.mavlink} ...")
    if args.mavlink.startswith(("udpin:", "tcp:")) and conn.wait_heartbeat(timeout=15) is None:
        raise SystemExit("Không nhận được SITL heartbeat trong 15 giây")
    print(f"SITL system={conn.target_system} component={conn.target_component}")

    processor = create_processor(config, str(FIND_LANDING))
    gst = subprocess.Popen(
        gstreamer_command(args.rtp_port, width, height), stdout=subprocess.PIPE,
        stderr=None, bufsize=width * height * 3 * 2,
    )
    if gst.stdout is None:
        raise SystemExit("Không mở được GStreamer stdout")

    frame_bytes = width * height * 3
    frame_id = found = valid = sent = 0
    last_send = 0.0
    last_report = time.monotonic()
    interval = 1.0 / max(1.0, min(50.0, args.rate))
    print(f"RTP :{args.rtp_port} -> detector {width}x{height} -> LANDING_TARGET @ {1/interval:.1f} Hz")
    try:
        while True:
            raw = read_exact(gst.stdout, frame_bytes)
            if raw is None:
                raise RuntimeError("GStreamer camera stream đã dừng")
            frame_id += 1
            if not processor.wants_frame(frame_id):
                continue
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((height, width, 3))
            state = {"detection_result": {"detected": False}}
            processor.process(frame, FrameMeta(frame_id, (width, height)), state)
            result = state.get("detection_result") or {"detected": False}
            if args.preview:
                display = frame.copy()
                if result.get("detected"):
                    draw_overlay(display, result)
                else:
                    cv2.putText(display, f"SEARCH: {result.get('reason', 'target not visible')}",
                                (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                                (0, 165, 255), 2, cv2.LINE_AA)
                cv2.putText(display, f"sent LANDING_TARGET: {sent}", (20, 35),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255),
                            2, cv2.LINE_AA)
                cv2.imshow("Gazebo precision landing - production detector", display)
                if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
                    break
            found += int(bool(result.get("detected")))
            control_valid = bool(
                result.get("detected") and result.get("control_valid")
                and not result.get("hold") and not result.get("ambiguous")
                and float(result.get("quality", 0.0) or 0.0) >= args.min_quality
            )
            valid += int(control_valid)
            now = time.monotonic()
            if control_valid and now - last_send >= interval:
                telemetry = dict(result)
                telemetry.update(frame_width=width, frame_height=height,
                                 measurement_monotonic_ms=int(now * 1000))
                conn.mav.send(_landing_target_from_telemetry(telemetry, args.hfov, args.vfov))
                sent += 1
                last_send = now
            if now - last_report >= 1.0:
                angle_x = math.degrees(math.atan((2.0 * float(result.get("offset_x", 0.0) or 0.0) / width) * math.tan(math.radians(args.hfov) / 2.0)))
                angle_y = math.degrees(math.atan((-2.0 * float(result.get("offset_y", 0.0) or 0.0) / height) * math.tan(math.radians(args.vfov) / 2.0)))
                print(f"state={result.get('tracking_state', 'SEARCH')} detected={bool(result.get('detected'))} valid={control_valid} quality={float(result.get('quality', 0.0) or 0.0):.2f} angle=({angle_x:+.2f},{angle_y:+.2f})deg found={found} valid_count={valid} sent={sent}")
                last_report = now
    except KeyboardInterrupt:
        print("Dừng bridge")
    finally:
        if args.preview:
            cv2.destroyAllWindows()
        gst.terminate()
        try:
            gst.wait(timeout=3)
        except subprocess.TimeoutExpired:
            gst.kill()
        conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

