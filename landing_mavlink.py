"""LANDING_TARGET uplink from vision telemetry — Pi camera/landing_mavlink.go."""

from __future__ import annotations

import logging
import math
import threading
import time
from pathlib import Path
from typing import Optional

import yaml
from pymavlink.dialects.v20 import common as mavlink_common

from mavlink_custom import COMP_ONBOARD
from web.camera_service import read_landing_telemetry

logger = logging.getLogger("LandingMavlink")


def _publish_generated(forwarder, msg) -> tuple[bool, bool]:
    """Send vision target to Pixhawk and mirror it to the UAVLink server."""
    sys_id = int(getattr(forwarder, "_pixhawk_sys_id", 0) or 0) or 1
    mav = mavlink_common.MAVLink(None, srcSystem=sys_id, srcComponent=COMP_ONBOARD)
    buf = msg.pack(mav)

    sent_pixhawk = False
    conn = getattr(forwarder, "_active_conn", None)
    if conn is not None:
        try:
            lock = getattr(forwarder, "_pixhawk_write_lock", None)
            if lock is None:
                conn.write(buf)
            else:
                with lock:
                    conn.write(buf)
            sent_pixhawk = True
        except (OSError, IOError) as exc:
            logger.debug("[LANDING][MAVLINK] Pixhawk write failed: %s", exc)

    sent_server = False
    server_sock = getattr(forwarder, "server_sock", None)
    auth_client = getattr(forwarder, "auth_client", None)
    if server_sock and auth_client and getattr(auth_client, "session_token", None):
        try:
            server_sock.sendto(buf, forwarder.target_addr)
            sent_server = True
        except OSError as exc:
            logger.debug("[LANDING][MAVLINK] server write failed: %s", exc)
    return sent_pixhawk, sent_server


def _pixel_angle(offset_px: float, frame_px: int, fov_deg: float) -> float:
    """Convert an image-plane pixel offset to optical angle (pinhole model)."""
    if frame_px <= 0:
        raise ValueError("frame size must be positive")
    if not 1.0 <= fov_deg < 179.0:
        raise ValueError("camera FOV must be in range 1..179 degrees")
    return math.atan(
        (2.0 * float(offset_px) / float(frame_px))
        * math.tan(math.radians(fov_deg) / 2.0)
    )


def _angular_size(size_px: float, frame_px: int, fov_deg: float) -> float:
    if size_px <= 0:
        return 0.0
    return 2.0 * math.atan(
        (float(size_px) / float(frame_px))
        * math.tan(math.radians(fov_deg) / 2.0)
    )


def _fov_from_camera_matrix(matrix, image_size) -> tuple[float, float]:
    width, height = int(image_size[0]), int(image_size[1])
    fx, fy = float(matrix[0][0]), float(matrix[1][1])
    if width <= 0 or height <= 0 or fx <= 0 or fy <= 0:
        raise ValueError("invalid camera calibration for FOV")
    return (
        math.degrees(2.0 * math.atan(width / (2.0 * fx))),
        math.degrees(2.0 * math.atan(height / (2.0 * fy))),
    )


def _calibrated_fov(cfg, camera_id: int) -> tuple[float, float] | None:
    camera = cfg.camera if hasattr(cfg, "camera") else {}
    streams = camera.get("streams", []) if isinstance(camera, dict) else []
    stream = next(
        (item for item in streams if int(item.get("camera_id", -1)) == int(camera_id)),
        None,
    )
    value = str((stream or {}).get("aruco_calibration_file") or "").strip()
    if not value:
        return None
    path = Path(value).expanduser()
    if not path.is_absolute():
        path = Path(__file__).resolve().parent / "Find_landing" / path
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    return _fov_from_camera_matrix(data["camera_matrix"], data["image_size"])


def _landing_target_from_telemetry(
    lt: dict,
    hfov_deg: float,
    vfov_deg: float,
    *,
    min_distance_m: float = 0.05,
    max_distance_m: float = 30.0,
):
    frame_width = int(lt.get("frame_width") or 0)
    frame_height = int(lt.get("frame_height") or 0)
    offset_x = float(lt.get("offset_x") or 0.0)
    offset_y_up = float(lt.get("offset_y") or 0.0)

    # Detector X is positive right. Detector Y is positive up, while image Y
    # (and LANDING_TARGET angle_y for a normal downward camera image) is down.
    angle_x = _pixel_angle(offset_x, frame_width, hfov_deg)
    angle_y = _pixel_angle(-offset_y_up, frame_height, vfov_deg)

    h_size = lt.get("h_size") or (0, 0)
    try:
        size_x = _angular_size(float(h_size[0]), frame_width, hfov_deg)
        size_y = _angular_size(float(h_size[1]), frame_height, vfov_deg)
    except (IndexError, TypeError, ValueError):
        size_x = size_y = 0.0

    mav = mavlink_common.MAVLink(None)
    x_body, y_body, z_body, distance = _body_frd_position_from_telemetry(
        lt,
        min_distance_m=min_distance_m,
        max_distance_m=max_distance_m,
    )
    try:
        return mav.landing_target_encode(
            time.monotonic_ns() // 1000,
            0,
            mavlink_common.MAV_FRAME_BODY_FRD,
            angle_x,
            angle_y,
            distance,
            size_x,
            size_y,
            x_body,
            y_body,
            z_body,
            q=(1.0, 0.0, 0.0, 0.0),
            type=mavlink_common.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            position_valid=1,
        )
    except TypeError:
        raise ValueError(
            "installed pymavlink does not support metric LANDING_TARGET fields"
        )


def _body_frd_position_from_telemetry(
    lt: dict,
    *,
    min_distance_m: float = 0.05,
    max_distance_m: float = 30.0,
) -> tuple[float, float, float, float]:
    """Convert calibrated OpenCV target pose to MAV_FRAME_BODY_FRD.

    OpenCV camera coordinates are X right, Y down and Z along the optical axis.
    The downward camera is mounted with image-top facing vehicle-forward, so
    BODY_FRD is X=-camera_Y, Y=camera_X, Z=camera_Z.
    """
    if not lt.get("pose_valid"):
        raise ValueError("metric target pose is not valid")
    pose = lt.get("target_center_camera_m") or lt.get("pose_camera_m")
    if not isinstance(pose, (list, tuple)) or len(pose) != 3:
        raise ValueError("metric target pose must contain camera X/Y/Z")
    try:
        camera_x, camera_y, camera_z = (float(value) for value in pose)
    except (TypeError, ValueError) as exc:
        raise ValueError("metric target pose is not numeric") from exc
    if not all(math.isfinite(value) for value in (camera_x, camera_y, camera_z)):
        raise ValueError("metric target pose must be finite")
    if camera_z <= 0:
        raise ValueError("metric target must be in front of the camera")

    x_body = -camera_y
    y_body = camera_x
    z_body = camera_z
    distance = math.sqrt(x_body * x_body + y_body * y_body + z_body * z_body)
    minimum = max(0.001, float(min_distance_m))
    maximum = max(minimum, float(max_distance_m))
    if not minimum <= distance <= maximum:
        raise ValueError(
            f"metric target distance {distance:.3f}m outside "
            f"{minimum:.3f}..{maximum:.3f}m"
        )
    return x_body, y_body, z_body, distance


def _telemetry_control_ready(
    lt: dict | None,
    *,
    min_quality: float,
    max_measurement_age_ms: int,
    require_control_valid: bool,
    require_metric_pose: bool = False,
    min_distance_m: float = 0.05,
    max_distance_m: float = 30.0,
    now_monotonic_ms: int | None = None,
) -> bool:
    """Fail closed when the vision measurement is stale, held or ambiguous."""
    if not lt:
        return False
    measurement_ms = int(lt.get("measurement_monotonic_ms") or 0)
    now_ms = (
        int(time.monotonic() * 1000)
        if now_monotonic_ms is None
        else int(now_monotonic_ms)
    )
    age_ms = now_ms - measurement_ms if measurement_ms > 0 else 10**9
    quality = float(lt.get("quality", 0.0) or 0.0)
    control_ok = bool(lt.get("control_valid")) or not require_control_valid
    base_ready = bool(
        lt.get("detected")
        and not lt.get("hold")
        and not lt.get("ambiguous")
        and control_ok
        and quality >= min_quality
        and 0 <= age_ms <= max_measurement_age_ms
    )
    if not base_ready:
        return False
    if require_metric_pose:
        try:
            _body_frd_position_from_telemetry(
                lt,
                min_distance_m=min_distance_m,
                max_distance_m=max_distance_m,
            )
        except ValueError:
            return False
    return True


def start_landing_mavlink_bridge(cfg, forwarder, stop_event: Optional[threading.Event] = None) -> None:
    landing = cfg.landing if hasattr(cfg, "landing") else {}
    if not landing.get("mavlink_enabled", False):
        return

    camera_id = int(landing.get("mavlink_camera_id", landing.get("mavlink_camera", 0)) or 0)
    hz = float(landing.get("mavlink_hz", 10) or 10)
    if hz <= 0:
        hz = 10
    interval = 1.0 / hz
    hfov_deg = float(landing.get("camera_hfov_deg", 0) or 0)
    vfov_deg = float(landing.get("camera_vfov_deg", 0) or 0)
    fov_source = "config"
    if not (1.0 <= hfov_deg < 179.0 and 1.0 <= vfov_deg < 179.0):
        try:
            calibrated = _calibrated_fov(cfg, camera_id)
            if calibrated is not None:
                hfov_deg, vfov_deg = calibrated
                fov_source = "camera calibration"
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            logger.error("[LANDING][MAVLINK] cannot derive FOV from calibration: %s", exc)
    min_quality = max(0.0, min(1.0, float(landing.get("min_quality", 0.55) or 0.55)))
    max_measurement_age_ms = max(50, int(landing.get("max_measurement_age_ms", 300) or 300))
    require_control_valid = bool(landing.get("require_control_valid", True))
    min_distance_m = max(0.001, float(landing.get("min_pose_distance_m", 0.05) or 0.05))
    max_distance_m = max(
        min_distance_m,
        float(landing.get("max_pose_distance_m", 30.0) or 30.0),
    )
    if not (1.0 <= hfov_deg < 179.0 and 1.0 <= vfov_deg < 179.0):
        logger.error(
            "[LANDING][MAVLINK] disabled: set calibrated camera_hfov_deg and "
            "camera_vfov_deg in config.yaml"
        )
        return
    logger.info(
        "[LANDING][MAVLINK] LANDING_TARGET cam%d @ %.1f Hz, FOV %.1fx%.1f deg, "
        "quality>=%.2f age<=%dms metric=%.2f..%.1fm (%s)",
        camera_id,
        hz,
        hfov_deg,
        vfov_deg,
        min_quality,
        max_measurement_age_ms,
        min_distance_m,
        max_distance_m,
        fov_source,
    )

    def _loop() -> None:
        last_bad_telemetry_log = 0.0
        while stop_event is None or not stop_event.is_set():
            lt = read_landing_telemetry(camera_id, 2.0)
            if _telemetry_control_ready(
                lt,
                min_quality=min_quality,
                max_measurement_age_ms=max_measurement_age_ms,
                require_control_valid=require_control_valid,
                require_metric_pose=True,
                min_distance_m=min_distance_m,
                max_distance_m=max_distance_m,
            ):
                try:
                    # Revalidate the same metric bounds immediately before encode.
                    _body_frd_position_from_telemetry(
                        lt,
                        min_distance_m=min_distance_m,
                        max_distance_m=max_distance_m,
                    )
                    msg = _landing_target_from_telemetry(
                        lt,
                        hfov_deg,
                        vfov_deg,
                        min_distance_m=min_distance_m,
                        max_distance_m=max_distance_m,
                    )
                    _publish_generated(forwarder, msg)
                except (TypeError, ValueError) as exc:
                    now = time.monotonic()
                    if now - last_bad_telemetry_log >= 10.0:
                        logger.warning("[LANDING][MAVLINK] invalid camera telemetry: %s", exc)
                        last_bad_telemetry_log = now
            time.sleep(interval)

    threading.Thread(target=_loop, daemon=True, name="landing-mavlink").start()
