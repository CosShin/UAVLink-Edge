#!/usr/bin/env python3
"""Receive browser location over HTTP and inject GPS_INPUT into a disarmed Pixhawk."""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import threading
import time
from dataclasses import dataclass
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Optional

from pymavlink import mavutil


PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))


GPS_EPOCH_UNIX = 315964800
GPS_UTC_LEAP_SECONDS = 18


@dataclass(frozen=True)
class BrowserFix:
    latitude: float
    longitude: float
    accuracy_m: float
    altitude_m: Optional[float]
    altitude_accuracy_m: Optional[float]
    speed_m_s: Optional[float]
    heading_deg: Optional[float]
    received_monotonic: float


class FixStore:
    def __init__(self):
        self._lock = threading.Lock()
        self._latest: Optional[BrowserFix] = None

    def set(self, fix: BrowserFix) -> None:
        with self._lock:
            self._latest = fix

    def get(self) -> Optional[BrowserFix]:
        with self._lock:
            return self._latest


def _optional_number(payload: dict, key: str) -> Optional[float]:
    value = payload.get(key)
    if value is None or value == "":
        return None
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"{key} must be finite")
    return result


def parse_browser_fix(payload: dict, now_monotonic: Optional[float] = None) -> BrowserFix:
    latitude = float(payload["latitude"])
    longitude = float(payload["longitude"])
    accuracy_m = float(payload["accuracy_m"])
    if not all(math.isfinite(value) for value in (latitude, longitude, accuracy_m)):
        raise ValueError("latitude, longitude and accuracy must be finite")
    if not -90.0 <= latitude <= 90.0:
        raise ValueError("latitude outside -90..90")
    if not -180.0 <= longitude <= 180.0:
        raise ValueError("longitude outside -180..180")
    if accuracy_m <= 0.0:
        raise ValueError("accuracy_m must be positive")
    speed = _optional_number(payload, "speed_m_s")
    heading = _optional_number(payload, "heading_deg")
    if speed is not None and speed < 0:
        raise ValueError("speed_m_s cannot be negative")
    if heading is not None:
        heading %= 360.0
    return BrowserFix(
        latitude=latitude,
        longitude=longitude,
        accuracy_m=accuracy_m,
        altitude_m=_optional_number(payload, "altitude_m"),
        altitude_accuracy_m=_optional_number(payload, "altitude_accuracy_m"),
        speed_m_s=speed,
        heading_deg=heading,
        received_monotonic=time.monotonic() if now_monotonic is None else now_monotonic,
    )


def gps_week(timestamp_s: float) -> tuple[int, int]:
    seconds = timestamp_s - GPS_EPOCH_UNIX + GPS_UTC_LEAP_SECONDS
    if seconds < 0:
        return 0, 0
    week = int(seconds // 604800)
    return week, int((seconds - week * 604800) * 1000)


def heartbeat_is_armed(heartbeat) -> bool:
    return bool(
        int(getattr(heartbeat, "base_mode", 0) or 0)
        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )


def velocity_ne(fix: BrowserFix) -> tuple[float, float]:
    if fix.speed_m_s is None or fix.heading_deg is None:
        return 0.0, 0.0
    angle = math.radians(fix.heading_deg)
    return fix.speed_m_s * math.cos(angle), fix.speed_m_s * math.sin(angle)


def make_handler(store: FixStore, token: str, max_accuracy_m: float):
    class Handler(BaseHTTPRequestHandler):
        def _json(self, status: int, payload: dict) -> None:
            body = json.dumps(payload).encode("utf-8")
            self.send_response(status)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path == "/health":
                fix = store.get()
                self._json(200, {"ok": True, "has_fix": fix is not None})
            else:
                self._json(404, {"ok": False, "error": "not found"})

        def do_POST(self):
            if self.path != "/gps":
                self._json(404, {"ok": False, "error": "not found"})
                return
            if self.headers.get("X-GPS-Token", "") != token:
                self._json(403, {"ok": False, "error": "invalid token"})
                return
            try:
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 8192:
                    raise ValueError("invalid body length")
                payload = json.loads(self.rfile.read(length).decode("utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("JSON root must be an object")
                fix = parse_browser_fix(payload)
            except (UnicodeDecodeError, json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
                self._json(400, {"ok": False, "error": str(exc)})
                return
            store.set(fix)
            accepted = fix.accuracy_m <= max_accuracy_m
            self._json(200, {
                "ok": True,
                "accepted_for_pixhawk": accepted,
                "accuracy_m": fix.accuracy_m,
                "max_accuracy_m": max_accuracy_m,
            })

        def log_message(self, format, *args):
            pass

    return Handler


def start_http_receiver(bind: str, port: int, store: FixStore, token: str, max_accuracy_m: float):
    server = ThreadingHTTPServer((bind, port), make_handler(store, token, max_accuracy_m))
    thread = threading.Thread(target=server.serve_forever, daemon=True, name="laptop-gps-http")
    thread.start()
    return server, thread


def inject_loop(args, store: FixStore) -> int:
    connection = mavutil.mavlink_connection(
        args.device,
        baud=args.baud,
        source_system=245,
        source_component=191,
    )
    print(f"Waiting for Pixhawk heartbeat on {args.device} @ {args.baud}...")
    heartbeat = connection.wait_heartbeat(timeout=10)
    if heartbeat is None:
        connection.close()
        raise RuntimeError("no Pixhawk heartbeat within 10 seconds")
    if heartbeat_is_armed(heartbeat):
        connection.close()
        raise RuntimeError("Pixhawk is ARMED; refusing laptop GPS injection")

    period = 1.0 / max(1.0, min(10.0, args.rate))
    deadline = time.monotonic() + max(10.0, min(600.0, args.duration))
    last_heartbeat = time.monotonic()
    last_status = 0.0
    sent = 0
    try:
        while time.monotonic() < deadline:
            while True:
                update = connection.recv_match(type="HEARTBEAT", blocking=False)
                if update is None:
                    break
                last_heartbeat = time.monotonic()
                if heartbeat_is_armed(update):
                    raise RuntimeError("Pixhawk became ARMED; GPS_INPUT stopped")
            if time.monotonic() - last_heartbeat > 3.0:
                raise RuntimeError("Pixhawk heartbeat stale; GPS_INPUT stopped")

            fix = store.get()
            now_mono = time.monotonic()
            reason = "waiting for laptop location"
            if fix is not None and now_mono - fix.received_monotonic > args.fix_timeout:
                reason = "laptop location is stale"
            elif fix is not None and fix.accuracy_m > args.max_accuracy_m:
                reason = f"accuracy {fix.accuracy_m:.1f}m exceeds {args.max_accuracy_m:.1f}m"
            elif fix is not None:
                now = time.time()
                week, week_ms = gps_week(now)
                altitude = fix.altitude_m if fix.altitude_m is not None else args.fallback_altitude_m
                altitude_accuracy = (
                    fix.altitude_accuracy_m
                    if fix.altitude_accuracy_m is not None
                    else max(3.0, fix.accuracy_m * 1.5)
                )
                vn, ve = velocity_ne(fix)
                hdop = max(0.8, min(99.0, fix.accuracy_m / 5.0))
                connection.mav.gps_input_send(
                    int(now * 1_000_000),
                    0,
                    0,
                    week_ms,
                    week,
                    3,
                    int(round(fix.latitude * 1e7)),
                    int(round(fix.longitude * 1e7)),
                    altitude,
                    hdop,
                    max(1.2, hdop * 1.5),
                    vn,
                    ve,
                    0.0,
                    0.5,
                    max(1.0, fix.accuracy_m),
                    max(1.0, altitude_accuracy),
                    args.satellites,
                )
                sent += 1
                reason = (
                    f"ACTIVE lat={fix.latitude:.7f} lon={fix.longitude:.7f} "
                    f"accuracy={fix.accuracy_m:.1f}m sent={sent}"
                )
            if now_mono - last_status >= 2.0:
                print(reason)
                last_status = now_mono
            time.sleep(period)
    finally:
        connection.close()
    return sent


def dry_run_loop(args, store: FixStore) -> None:
    deadline = time.monotonic() + max(10.0, min(600.0, args.duration))
    while time.monotonic() < deadline:
        fix = store.get()
        if fix is None:
            print("DRY-RUN: waiting for laptop location")
        else:
            print(
                f"DRY-RUN: lat={fix.latitude:.7f} lon={fix.longitude:.7f} "
                f"accuracy={fix.accuracy_m:.1f}m "
                f"accepted={fix.accuracy_m <= args.max_accuracy_m}"
            )
        time.sleep(2.0)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Laptop browser GPS -> HTTP -> Pixhawk GPS_INPUT")
    parser.add_argument("--bind", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8766)
    parser.add_argument("--token", required=True)
    parser.add_argument("--device", default="/dev/ttyAMA0")
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--fallback-altitude-m", type=float, required=True)
    parser.add_argument("--rate", type=float, default=5.0)
    parser.add_argument("--duration", type=float, default=300.0)
    parser.add_argument("--fix-timeout", type=float, default=3.0)
    parser.add_argument("--max-accuracy-m", type=float, default=100.0)
    parser.add_argument("--satellites", type=int, default=10)
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--bench-confirm", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if len(args.token) < 16:
        print("REFUSED: --token must contain at least 16 characters")
        return 2
    if not args.bench_confirm:
        print("REFUSED: add --bench-confirm for DISARMED bench testing")
        return 2
    if not math.isfinite(args.fallback_altitude_m):
        print("REFUSED: fallback altitude must be finite")
        return 2
    args.rate = max(1.0, min(10.0, float(args.rate)))
    args.duration = max(10.0, min(600.0, float(args.duration)))
    args.fix_timeout = max(1.0, min(10.0, float(args.fix_timeout)))
    args.max_accuracy_m = max(1.0, min(5000.0, float(args.max_accuracy_m)))
    args.satellites = max(6, min(30, int(args.satellites)))

    if not args.dry_run:
        from instance_lock import acquire_instance_lock

        acquire_instance_lock()
    store = FixStore()
    server, thread = start_http_receiver(args.bind, args.port, store, args.token, args.max_accuracy_m)
    print(f"Laptop GPS receiver: http://<PI-IP>:{args.port}/gps")
    print(f"Mode: {'DRY-RUN' if args.dry_run else 'PIXHAWK DISARMED ONLY'}")
    try:
        if args.dry_run:
            dry_run_loop(args, store)
            return 0
        sent = inject_loop(args, store)
        print(f"Laptop GPS finished; packets sent={sent}")
        return 0
    except (OSError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}")
        return 1
    except KeyboardInterrupt:
        print("Stopped")
        return 130
    finally:
        server.shutdown()
        server.server_close()
        thread.join(timeout=2.0)


if __name__ == "__main__":
    raise SystemExit(main())
