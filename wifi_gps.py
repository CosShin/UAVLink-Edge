#!/usr/bin/env python3
"""Receive GPS fixes over Wi-Fi/UDP and inject MAVLink GPS_INPUT into ArduPilot.

Default data path:
    phone/server --JSON UDP :25100--> wifi_gps.py
    wifi_gps.py --MAVLink UDP 127.0.0.1:14600--> main.py --> Pixhawk

This is intended for disarmed bench/SITL tests.  Wi-Fi is only the transport;
it is not a positioning sensor.  Do not use a fixed or phone-grade position as
the sole position source for autonomous indoor flight.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import sys
import time
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml
from pymavlink import mavutil


GPS_EPOCH_UNIX = 315964800
GPS_UTC_LEAP_SECONDS = 18
DEFAULT_LISTEN_PORT = 25100
DEFAULT_INJECT_PORT = 14600
EARTH_RADIUS_M = 6378137.0


@dataclass
class GpsFix:
    lat: float
    lon: float
    alt_m: float
    fix_type: int = 3
    satellites: int = 10
    hdop: float = 1.0
    vdop: float = 1.5
    vn: Optional[float] = None
    ve: Optional[float] = None
    vd: Optional[float] = None
    speed_accuracy: Optional[float] = None
    horiz_accuracy: Optional[float] = None
    vert_accuracy: Optional[float] = None
    timestamp_s: float = 0.0


def global_offset(lat: float, lon: float, north_m: float, east_m: float) -> Tuple[float, float]:
    """Move a WGS84 coordinate by a small local North/East offset."""
    lat_rad = math.radians(lat)
    cos_lat = math.cos(lat_rad)
    if abs(cos_lat) < 1e-6:
        raise ValueError("không hỗ trợ vision GPS gần cực địa lý")
    out_lat = lat + math.degrees(north_m / EARTH_RADIUS_M)
    out_lon = lon + math.degrees(east_m / (EARTH_RADIUS_M * cos_lat))
    return out_lat, out_lon


def board_xy_to_ne(x_m: float, y_m: float, heading_deg: float) -> Tuple[float, float]:
    """Rotate board X/Y into North/East.

    ``heading_deg`` is the compass heading of the board's +X direction.  The
    printed board's +Y direction is 90 degrees clockwise from +X.
    """
    heading = math.radians(heading_deg)
    north = x_m * math.cos(heading) - y_m * math.sin(heading)
    east = x_m * math.sin(heading) + y_m * math.cos(heading)
    return north, east


def camera_position_on_board(pose_camera_m, rvec) -> Tuple[float, float, float]:
    """Invert OpenCV solvePnP output to get camera position in board axes."""
    try:
        import cv2
        import numpy as np

        t = np.asarray(pose_camera_m, dtype=np.float64).reshape(3, 1)
        rotation_vector = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    except (ImportError, TypeError, ValueError) as exc:
        raise ValueError(f"pose camera không hợp lệ: {exc}") from exc
    rotation, _ = cv2.Rodrigues(rotation_vector)
    camera = -rotation.T @ t
    if not np.all(np.isfinite(camera)):
        raise ValueError("pose camera chứa NaN/Inf")
    return tuple(float(value) for value in camera.reshape(3))


class HybridVisionGps:
    """Latch a coarse Wi-Fi anchor and add metric ArUco-board displacement."""

    def __init__(
        self,
        telemetry_path: Path,
        *,
        heading_deg: float,
        timeout_s: float,
        min_quality: float,
        horizontal_accuracy_m: float,
        max_radius_m: float,
        max_step_m: float,
    ):
        self.telemetry_path = telemetry_path
        self.heading_deg = float(heading_deg)
        self.timeout_s = max(0.05, float(timeout_s))
        self.min_quality = max(0.0, min(1.0, float(min_quality)))
        self.horizontal_accuracy_m = max(0.05, float(horizontal_accuracy_m))
        self.max_radius_m = max(0.1, float(max_radius_m))
        self.max_step_m = max(0.01, float(max_step_m))
        self.anchor_fix: Optional[GpsFix] = None
        self.reference_camera: Optional[Tuple[float, float, float]] = None
        self.last_camera: Optional[Tuple[float, float, float]] = None
        self.target_key = ""
        self.reason = "chưa có pose camera"
        self.north_m = 0.0
        self.east_m = 0.0

    def _read_telemetry(self) -> Optional[dict]:
        try:
            data = json.loads(self.telemetry_path.read_text(encoding="utf-8"))
        except (OSError, UnicodeDecodeError, json.JSONDecodeError):
            self.reason = f"chưa đọc được {self.telemetry_path}"
            return None
        if not isinstance(data, dict):
            self.reason = "vision telemetry không phải JSON object"
            return None
        return data

    def _valid_camera_position(self, now_wall: float) -> Optional[Tuple[float, float, float]]:
        data = self._read_telemetry()
        if data is None:
            return None
        updated_at = float(data.get("updated_at", 0.0) or 0.0)
        age = now_wall - updated_at if updated_at > 0 else float("inf")
        if age < -1.0 or age > self.timeout_s:
            self.reason = f"pose camera stale ({age:.2f}s)"
            return None
        if not data.get("detected"):
            self.reason = "camera chưa thấy ArUco board"
            return None
        if data.get("hold") or data.get("ambiguous"):
            self.reason = "pose camera đang hold/ambiguous"
            return None
        if not data.get("control_valid"):
            self.reason = "ArUco chưa đạt trạng thái TRACKING"
            return None
        quality = float(data.get("quality", 0.0) or 0.0)
        if quality < self.min_quality:
            self.reason = f"chất lượng pose thấp ({quality:.2f})"
            return None
        if not data.get("pose_valid"):
            self.reason = "chưa có pose mét; cần calibration + kích thước marker"
            return None
        target_key = str(data.get("target_key") or "")
        if not target_key:
            self.reason = "pose camera thiếu target_key"
            return None
        if self.target_key and target_key != self.target_key:
            self.reason = f"ArUco target đổi từ {self.target_key} sang {target_key}"
            return None
        try:
            camera = camera_position_on_board(data.get("pose_camera_m"), data.get("rvec"))
        except ValueError as exc:
            self.reason = str(exc)
            return None
        self.target_key = target_key
        return camera

    def make_fix(self, wifi_fix: Optional[GpsFix], now_wall: Optional[float] = None) -> Optional[GpsFix]:
        if wifi_fix is None:
            self.reason = "chưa có tọa độ gốc từ Wi-Fi"
            return None
        now_wall = time.time() if now_wall is None else float(now_wall)
        camera = self._valid_camera_position(now_wall)
        if camera is None:
            return None

        if self.reference_camera is None:
            self.anchor_fix = replace(wifi_fix)
            self.reference_camera = camera
            self.last_camera = camera
            self.north_m = self.east_m = 0.0
        else:
            assert self.anchor_fix is not None
            assert self.last_camera is not None
            step = math.hypot(camera[0] - self.last_camera[0], camera[1] - self.last_camera[1])
            if step > self.max_step_m:
                self.reason = f"pose camera nhảy {step:.2f}m > {self.max_step_m:.2f}m"
                return None
            dx = camera[0] - self.reference_camera[0]
            dy = camera[1] - self.reference_camera[1]
            radius = math.hypot(dx, dy)
            if radius > self.max_radius_m:
                self.reason = f"camera ra ngoài bán kính board {radius:.2f}m"
                return None
            self.north_m, self.east_m = board_xy_to_ne(dx, dy, self.heading_deg)
            self.last_camera = camera

        assert self.anchor_fix is not None
        lat, lon = global_offset(
            self.anchor_fix.lat,
            self.anchor_fix.lon,
            self.north_m,
            self.east_m,
        )
        self.reason = "OK"
        return replace(
            self.anchor_fix,
            lat=lat,
            lon=lon,
            fix_type=max(3, self.anchor_fix.fix_type),
            hdop=max(0.01, self.horizontal_accuracy_m),
            vdop=max(0.01, self.anchor_fix.vdop),
            vn=None,
            ve=None,
            vd=None,
            speed_accuracy=None,
            horiz_accuracy=self.horizontal_accuracy_m,
            timestamp_s=now_wall,
        )


def _number(data: Dict[str, Any], *keys: str, default=None):
    for key in keys:
        value = data.get(key)
        if value is not None and value != "":
            return float(value)
    return default


def _degrees(value: float, limit: float, field: str) -> float:
    # Accept either decimal degrees or MAVLink degE7 integers.
    if abs(value) > limit:
        value /= 1e7
    if not -limit <= value <= limit:
        raise ValueError(f"{field} ngoài phạm vi hợp lệ")
    return value


def parse_fix(data: Dict[str, Any]) -> GpsFix:
    lat_raw = _number(data, "lat", "latitude")
    lon_raw = _number(data, "lon", "lng", "longitude")
    if lat_raw is None or lon_raw is None:
        raise ValueError("JSON phải có lat và lon")

    lat = _degrees(lat_raw, 90.0, "lat")
    lon = _degrees(lon_raw, 180.0, "lon")
    alt_m = float(_number(data, "alt_m", "alt", "altitude", default=0.0))
    if not -1000.0 <= alt_m <= 20000.0:
        raise ValueError("altitude ngoài phạm vi -1000..20000 m")

    vn = _number(data, "vn", "velocity_north")
    ve = _number(data, "ve", "velocity_east")
    vd = _number(data, "vd", "velocity_down")
    speed = _number(data, "speed_m_s", "speed")
    course = _number(data, "course_deg", "course", "bearing")
    if (vn is None or ve is None) and speed is not None and course is not None:
        angle = math.radians(course)
        vn = speed * math.cos(angle)
        ve = speed * math.sin(angle)

    timestamp_ms = _number(data, "timestamp_ms", "time_ms")
    timestamp_s = timestamp_ms / 1000.0 if timestamp_ms else float(
        _number(data, "timestamp_s", "timestamp", default=time.time())
    )
    now = time.time()
    if timestamp_s <= 0 or timestamp_s > now + 300:
        timestamp_s = now

    fix = GpsFix(
        lat=lat,
        lon=lon,
        alt_m=alt_m,
        fix_type=max(0, min(6, int(_number(data, "fix_type", default=3)))),
        satellites=max(0, min(255, int(_number(data, "satellites", "satellites_visible", default=10)))),
        hdop=max(0.01, float(_number(data, "hdop", default=1.0))),
        vdop=max(0.01, float(_number(data, "vdop", default=1.5))),
        vn=vn,
        ve=ve,
        vd=vd,
        speed_accuracy=_number(data, "speed_accuracy"),
        horiz_accuracy=_number(data, "horiz_accuracy", "accuracy_m", "accuracy"),
        vert_accuracy=_number(data, "vert_accuracy", "vertical_accuracy_m"),
        timestamp_s=timestamp_s,
    )
    for name in ("speed_accuracy", "horiz_accuracy", "vert_accuracy"):
        value = getattr(fix, name)
        if value is not None and value < 0:
            raise ValueError(f"{name} không được âm")
    return fix


def gps_week(timestamp_s: float) -> Tuple[int, int]:
    gps_seconds = timestamp_s - GPS_EPOCH_UNIX + GPS_UTC_LEAP_SECONDS
    if gps_seconds < 0:
        return 0, 0
    week = int(gps_seconds // 604800)
    week_ms = int((gps_seconds - week * 604800) * 1000)
    return week, week_ms


def gps_ignore_flags(fix: GpsFix) -> int:
    mav = mavutil.mavlink
    flags = 0
    if fix.vn is None or fix.ve is None:
        flags |= mav.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ
    if fix.vd is None:
        flags |= mav.GPS_INPUT_IGNORE_FLAG_VEL_VERT
    if fix.speed_accuracy is None:
        flags |= mav.GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY
    if fix.horiz_accuracy is None:
        flags |= mav.GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY
    if fix.vert_accuracy is None:
        flags |= mav.GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY
    return flags


class GpsSender:
    def __init__(self, endpoint: str, baud: int):
        kwargs = {"source_system": 254, "source_component": 191}
        if endpoint.startswith("/"):
            kwargs["baud"] = baud
        self.connection = mavutil.mavlink_connection(endpoint, **kwargs)

    def send(self, fix: GpsFix) -> None:
        week, week_ms = gps_week(fix.timestamp_s or time.time())
        self.connection.mav.gps_input_send(
            int((fix.timestamp_s or time.time()) * 1_000_000),
            0,
            gps_ignore_flags(fix),
            week_ms,
            week,
            fix.fix_type,
            int(round(fix.lat * 1e7)),
            int(round(fix.lon * 1e7)),
            fix.alt_m,
            fix.hdop,
            fix.vdop,
            float(fix.vn or 0.0),
            float(fix.ve or 0.0),
            float(fix.vd or 0.0),
            float(fix.speed_accuracy or 0.0),
            float(fix.horiz_accuracy or 0.0),
            float(fix.vert_accuracy or 0.0),
            fix.satellites,
        )

    def close(self) -> None:
        self.connection.close()


def load_project_defaults(path: str) -> dict:
    try:
        data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    except (OSError, yaml.YAMLError):
        return {}
    return data


def main_service_reachable(web_port: int) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", web_port), timeout=0.5):
            return True
    except OSError:
        return False


def parse_fixed(value: str) -> GpsFix:
    parts = [part.strip() for part in value.split(",")]
    if len(parts) not in (2, 3):
        raise ValueError("--fixed dùng dạng LAT,LON hoặc LAT,LON,ALT_M")
    return parse_fix({
        "lat": float(parts[0]),
        "lon": float(parts[1]),
        "alt_m": float(parts[2]) if len(parts) == 3 else 0.0,
        "vn": 0.0,
        "ve": 0.0,
        "vd": 0.0,
        "speed_accuracy": 0.2,
        "horiz_accuracy": 1.5,
        "vert_accuracy": 2.5,
        "satellites": 10,
        "fix_type": 3,
    })


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="GPS-over-Wi-Fi JSON UDP → MAVLink GPS_INPUT for ArduPilot"
    )
    parser.add_argument("--config", default="config.yaml")
    parser.add_argument("--listen-host", default="0.0.0.0")
    parser.add_argument("--listen-port", type=int, default=DEFAULT_LISTEN_PORT)
    parser.add_argument("--token", default=os.environ.get("WIFI_GPS_TOKEN", ""))
    parser.add_argument("--allow-unauthenticated", action="store_true")
    parser.add_argument("--inject-host", default="127.0.0.1")
    parser.add_argument("--inject-port", type=int, default=None)
    parser.add_argument("--direct", metavar="DEVICE", help="Open Pixhawk serial directly; stop main.py first")
    parser.add_argument("--baud", type=int, default=None)
    parser.add_argument("--rate", type=float, default=5.0, help="GPS_INPUT rate in Hz (1..10)")
    parser.add_argument("--timeout", type=float, default=2.0, help="Stop output when Wi-Fi fix is stale")
    parser.add_argument("--fixed", metavar="LAT,LON[,ALT_M]", help="Fixed DISARMED bench position")
    parser.add_argument(
        "--vision-camera-id",
        type=int,
        default=None,
        help="Experimental bench mode: Wi-Fi anchor + metric ArUco-board displacement",
    )
    parser.add_argument("--vision-telemetry", default=None, help="Override camera landing JSON path")
    parser.add_argument(
        "--vision-heading-deg",
        type=float,
        default=0.0,
        help="Compass heading of printed board +X axis (0=North, 90=East)",
    )
    parser.add_argument("--vision-timeout", type=float, default=0.35)
    parser.add_argument("--vision-min-quality", type=float, default=0.55)
    parser.add_argument("--vision-horizontal-accuracy", type=float, default=0.5)
    parser.add_argument("--vision-max-radius", type=float, default=10.0)
    parser.add_argument("--vision-max-step", type=float, default=1.0)
    parser.add_argument(
        "--bench-confirm",
        action="store_true",
        help="Confirm fixed/vision mode is for DISARMED bench testing",
    )
    parser.add_argument("--dry-run", action="store_true")
    return parser


def main() -> int:
    args = build_parser().parse_args()
    cfg = load_project_defaults(args.config)
    network = cfg.get("network", {}) if isinstance(cfg, dict) else {}
    forwarding = cfg.get("forwarding", {}) if isinstance(cfg, dict) else {}
    web = cfg.get("web", {}) if isinstance(cfg, dict) else {}
    inject_port = args.inject_port or int(network.get("local_inject_port", DEFAULT_INJECT_PORT))
    baud = args.baud or int(network.get("serial_baud", 921600))
    endpoint = args.direct or f"udpout:{args.inject_host}:{inject_port}"
    server_host = forwarding.get("target_host") or network.get("target_host") or "(chưa cấu hình)"
    server_port = int(forwarding.get("target_port") or network.get("target_port") or 14550)
    web_port = int(web.get("port", 8080) or 8080)
    rate = min(max(float(args.rate), 1.0), 10.0)
    period = 1.0 / rate

    fixed: Optional[GpsFix] = None
    if args.fixed and args.vision_camera_id is not None:
        print("TỪ CHỐI: không dùng đồng thời --fixed và --vision-camera-id.", file=sys.stderr)
        return 2
    if args.fixed:
        if not args.bench_confirm:
            print("TỪ CHỐI: --fixed chỉ dành cho bench DISARMED; thêm --bench-confirm.", file=sys.stderr)
            return 2
        fixed = parse_fixed(args.fixed)
        print("CẢNH BÁO: fixed GPS không phải nguồn vị trí an toàn để bay trong nhà.")

    vision: Optional[HybridVisionGps] = None
    if args.vision_camera_id is not None:
        if not args.bench_confirm:
            print(
                "TỪ CHỐI: vision GPS mới chỉ dành cho bench DISARMED; "
                "thêm --bench-confirm.",
                file=sys.stderr,
            )
            return 2
        telemetry_path = Path(
            args.vision_telemetry or f"/tmp/camera_landing_{args.vision_camera_id}.json"
        )
        vision = HybridVisionGps(
            telemetry_path,
            heading_deg=args.vision_heading_deg,
            timeout_s=args.vision_timeout,
            min_quality=args.vision_min_quality,
            horizontal_accuracy_m=args.vision_horizontal_accuracy,
            max_radius_m=args.vision_max_radius,
            max_step_m=args.vision_max_step,
        )
        print(
            "CẢNH BÁO: vision GPS là chế độ thử nghiệm DISARMED; "
            "Wi-Fi chỉ làm mốc, ArUco board phải luôn nằm trong ảnh."
        )

    if not fixed and args.listen_host not in ("127.0.0.1", "::1", "localhost"):
        if not args.token and not args.allow_unauthenticated:
            print(
                "TỪ CHỐI bind Wi-Fi không có xác thực. Dùng --token SECRET "
                "hoặc --allow-unauthenticated cho mạng test cô lập.",
                file=sys.stderr,
            )
            return 2

    sender = None if args.dry_run else GpsSender(endpoint, baud)
    sock: Optional[socket.socket] = None
    if not fixed:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((args.listen_host, args.listen_port))
        sock.settimeout(min(period, 0.2))
        print(f"Nhận GPS JSON tại udp://{args.listen_host}:{args.listen_port}")
    print(f"GPS_INPUT → {endpoint} @ {rate:.1f} Hz{' (dry-run)' if args.dry_run else ''}")
    if args.dry_run:
        print("DRY-RUN: chỉ kiểm tra dữ liệu; KHÔNG gửi GPS_INPUT tới main.py/Pixhawk.")
    if vision:
        print(
            f"Vision GPS ← {vision.telemetry_path}; board +X="
            f"{args.vision_heading_deg:.1f}°; quality≥{vision.min_quality:.2f}"
        )
    if not args.direct:
        print(f"Chặng nội bộ: wifi_gps.py → main.py tại 127.0.0.1:{inject_port}")
        print(f"Đích uplink cuối: main.py → udp://{server_host}:{server_port}")
        if not main_service_reachable(web_port):
            print(
                f"CẢNH BÁO: main.py chưa chạy (không thấy localhost:{web_port}). "
                "Hãy mở terminal khác và chạy ./run.sh.",
                file=sys.stderr,
            )

    latest = fixed
    latest_received = time.monotonic() if fixed else 0.0
    next_send = time.monotonic()
    sent = 0
    last_status = 0.0
    try:
        while True:
            if sock is not None:
                try:
                    raw, peer = sock.recvfrom(8192)
                    data = json.loads(raw.decode("utf-8"))
                    if not isinstance(data, dict):
                        raise ValueError("JSON root phải là object")
                    if args.token and data.get("token") != args.token:
                        print(f"Bỏ gói sai token từ {peer[0]}", file=sys.stderr)
                        continue
                    latest = parse_fix(data)
                    latest_received = time.monotonic()
                except socket.timeout:
                    pass
                except (UnicodeDecodeError, json.JSONDecodeError, ValueError) as exc:
                    print(f"Bỏ gói GPS lỗi: {exc}", file=sys.stderr)

            now = time.monotonic()
            if latest is not None and now >= next_send:
                wifi_fresh = fixed is not None or now - latest_received <= max(args.timeout, 0.2)
                output_fix = latest
                if vision and wifi_fresh:
                    output_fix = vision.make_fix(latest)
                fresh = wifi_fresh and output_fix is not None
                if fresh and output_fix is not None:
                    output_fix.timestamp_s = time.time()
                    if sender:
                        sender.send(output_fix)
                    sent += 1
                    if now - last_status >= 5.0:
                        counter_label = "validated" if args.dry_run else "injected"
                        vision_status = (
                            f" vision_N={vision.north_m:+.2f}m vision_E={vision.east_m:+.2f}m"
                            if vision
                            else ""
                        )
                        print(
                            f"GPS OK lat={output_fix.lat:.7f} lon={output_fix.lon:.7f} "
                            f"alt={output_fix.alt_m:.1f}m fix={output_fix.fix_type} "
                            f"{counter_label}={sent}{vision_status}"
                        )
                        last_status = now
                elif now - last_status >= 2.0:
                    reason = (
                        "GPS Wi-Fi stale"
                        if not wifi_fresh
                        else f"Vision GPS chưa sẵn sàng: {vision.reason if vision else 'không có fix'}"
                    )
                    print(f"{reason} — đã dừng GPS_INPUT (failsafe)", file=sys.stderr)
                    last_status = now
                next_send = now + period
    except KeyboardInterrupt:
        print("\nĐã dừng GPS-over-Wi-Fi")
    finally:
        if sock:
            sock.close()
        if sender:
            sender.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
