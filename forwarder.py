import os
import socket
import threading
import time
import logging
from datetime import datetime, timezone
from typing import Dict, Optional, Tuple

from pymavlink import mavutil

from mavlink_utils import (
    MAVLINK_PATH_ETHERNET,
    MAVLINK_PATH_SERIAL,
    is_pixhawk_heartbeat,
    normalize_connection_type,
)
from mavlink_custom import (
    COMP_ONBOARD,
    COMPANION_SYS_ID,
    GPS_DIAG_NO_PX4_STREAM,
    GPS_DIAG_PX4_LOCAL_ONLY,
    GPS_DIAG_PX4_NO_FIX,
    GPS_DIAG_PX4_OK,
    build_dronebridge_status_frame,
    build_session_heartbeat_frame,
    build_session_heartbeat_frame_shifted,
    forward_gps_raw_int,
    session_hb_mode,
)
from metrics import global_metrics
from network_utils import detect_network_info, get_local_ip
from telemetry import global_telemetry
from web.mavlink_bridge import bridge

logger = logging.getLogger("Forwarder")

# A practical full telemetry profile for a 921600-baud Pixhawk link.  Forwarding
# remains unfiltered; these requests only make ArduPilot publish streams that are
# otherwise disabled on a quiet TELEM port.
FULL_TELEMETRY_RATES_HZ = {
    "SYS_STATUS": 1.0,
    "BATTERY_STATUS": 1.0,
    "SYSTEM_TIME": 1.0,
    "GPS_RAW_INT": 2.0,
    "RAW_IMU": 10.0,
    "SCALED_PRESSURE": 5.0,
    "ATTITUDE": 10.0,
    "LOCAL_POSITION_NED": 5.0,
    "GLOBAL_POSITION_INT": 5.0,
    "SERVO_OUTPUT_RAW": 5.0,
    "MISSION_CURRENT": 1.0,
    "NAV_CONTROLLER_OUTPUT": 2.0,
    "RC_CHANNELS": 5.0,
    "VFR_HUD": 5.0,
    "POWER_STATUS": 1.0,
    "TERRAIN_REPORT": 1.0,
    "MEMINFO": 0.5,
    "WIND": 1.0,
    "RANGEFINDER": 5.0,
    "EKF_STATUS_REPORT": 2.0,
    "VIBRATION": 2.0,
    "HOME_POSITION": 0.2,
    "EXTENDED_SYS_STATE": 1.0,
}
COPTER_MODE_AUTO = 3


def sys_status_to_battery_status(msg):
    """Build the BATTERY_STATUS packet expected by the fleet server."""
    mav = mavutil.mavlink
    voltage_mv = max(0, min(65534, int(getattr(msg, "voltage_battery", 0) or 0)))
    current_ca = int(getattr(msg, "current_battery", -1))
    remaining = int(getattr(msg, "battery_remaining", -1))
    return mav.MAVLink_battery_status_message(
        0,
        mav.MAV_BATTERY_FUNCTION_ALL,
        mav.MAV_BATTERY_TYPE_UNKNOWN,
        32767,
        [voltage_mv] + [65535] * 9,
        current_ca,
        -1,
        -1,
        max(-1, min(100, remaining)),
    )


def pack_set_mode_command(msg, custom_mode: int | None = None) -> bytes:
    """Repack MAV_CMD_DO_SET_MODE with ArduPilot's required custom-mode flag."""
    mav = mavutil.mavlink
    if custom_mode is None:
        mode = int(round(float(msg.param2)))
        legacy_param1 = int(round(float(msg.param1)))
        # Compatibility with older backends that put the Copter mode number in
        # param1 instead of the required custom-mode flag and param2.
        if mode == 0 and legacy_param1 not in (0, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED):
            mode = legacy_param1
    else:
        mode = int(custom_mode)
    encoder = mav.MAVLink(
        None,
        srcSystem=int(msg.get_srcSystem() or 255),
        srcComponent=int(msg.get_srcComponent() or 190),
    )
    corrected = mav.MAVLink_command_long_message(
        int(getattr(msg, "target_system", 0) or 0),
        int(getattr(msg, "target_component", 0) or 0),
        mav.MAV_CMD_DO_SET_MODE,
        int(getattr(msg, "confirmation", 0) or 0),
        float(mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED),
        float(mode),
        0.0,
        0.0,
        0.0,
        0.0,
        0.0,
    )
    return corrected.pack(encoder)


class Forwarder:
    def __init__(self, config, auth_client, vpn_manager=None):
        self.config = config
        self.auth_client = auth_client
        self.vpn_manager = vpn_manager
        self.network = config.network
        self.ethernet = config.ethernet or {}
        self.running = False
        self.server_sock = None
        self.target_addr = (
            config.forwarding.get("target_host"),
            config.forwarding.get("target_port"),
        )
        self.tcp_host = self.network.get("tcp_host", "0.0.0.0")
        self.tcp_port = self.network.get("local_listen_port", self.network.get("tcp_port", 14540))
        self.connection_type = normalize_connection_type(self.network.get("connection_type", "serial"))

        self._connections: Dict[str, object] = {}
        self._path_lock = threading.RLock()
        self._pixhawk_write_lock = threading.Lock()
        self._active_path = ""
        self._active_conn = None
        self._eth_heartbeat_at: Optional[datetime] = None
        self._serial_heartbeat_at: Optional[datetime] = None

        self._pixhawk_connected = threading.Event()
        self._pixhawk_sys_id = 0
        self._pixhawk_armed = False
        self._pixhawk_custom_mode = 0
        self._is_healthy = True
        self._previous_ip = ""

        self.stats_lock = threading.Lock()
        self.stats = {
            "rawIn": 0,
            "accepted": 0,
            "outServer": 0,
            "dropErr": 0,
            "dropNoPixhawk": 0,
            "dropUnhealthy": 0,
            "dropAuthNotReady": 0,
            "dropVpnNotReady": 0,
        }
        self._rate_lock = threading.Lock()
        self._rate_raw_in = 0
        self._rate_accepted = 0
        self._rate_out_server = 0
        self._rate_bytes_out = 0

        self._gps_last_at: Optional[datetime] = None
        self._battery_status_last_at: Optional[datetime] = None
        self._message_last_at: Dict[str, datetime] = {}
        self._gps_fix_type = 0
        self._gps_satellites = 0
        self._local_pos_last_at: Optional[datetime] = None
        self._hb_seq = 0
        self._companion_seq = 0
        self._mavlink_ka_seq = 0
        self._downlink_error_at = 0.0
        self._downlink_parser = mavutil.mavlink.MAVLink(None)
        self._mission_command_lock = threading.Lock()
        self._pending_mission_start: Optional[Tuple[bytes, float]] = None
        # Mission metadata observed on the MAVLink bridge.  The route drawn in
        # the web UI is not necessarily stored in ArduPilot; keeping this
        # separate lets the logs explain that very common failure clearly.
        self._mission_item_count: Optional[int] = None
        self._mission_upload_count: Optional[int] = None
        self._mission_first_command: Optional[int] = None

    def _fallback_timeout_sec(self) -> float:
        timeout = self.ethernet.get("pixhawk_connection_timeout", 30)
        sec = max(3, int(timeout) // 2)
        return float(sec)

    def _listen_port(self) -> int:
        return int(self.network.get("local_listen_port") or self.network.get("tcp_port") or 14550)

    def _ethernet_udpin_spec(self) -> str:
        """Bind fixed UDP on ethernet.local_ip (PX4 sends unicast to CM5 IP:port)."""
        port = self._listen_port()
        local_ip = str(self.ethernet.get("local_ip") or "").strip()
        if local_ip:
            return f"udpin:{local_ip}:{port}"
        return f"udpin:0.0.0.0:{port}"

    def _pixhawk_udp_target(self) -> Optional[Tuple[str, int]]:
        pixhawk_ip = str(self.ethernet.get("pixhawk_ip") or "").strip()
        if not pixhawk_ip:
            return None
        port = int(self.ethernet.get("pixhawk_port") or 0)
        if port <= 0:
            port = self._listen_port()
        return pixhawk_ip, port

    def _create_connection(self, path: str):
        if path == MAVLINK_PATH_SERIAL:
            device = self.network.get("serial_port", "/dev/ttyAMA2")
            baud = int(self.network.get("serial_baud", 57600))
            if not os.path.exists(device):
                raise FileNotFoundError(f"serial device {device} not available")
            logger.info("[MAVLINK] Serial listener enabled on %s @ %d baud", device, baud)
            return mavutil.mavlink_connection(device, baud=baud)

        if self.connection_type == "tcp_listen":
            logger.info("[MAVLINK] Listening for Pixhawk via TCP port %s", self.tcp_port)
            return mavutil.mavlink_connection(f"tcpin:{self.tcp_host}:{self.tcp_port}")
        if self.connection_type == "tcp_client":
            logger.info("[MAVLINK] Connecting to Pixhawk via TCP: %s:%s", self.tcp_host, self.tcp_port)
            return mavutil.mavlink_connection(f"tcp:{self.tcp_host}:{self.tcp_port}")

        spec = self._ethernet_udpin_spec()
        logger.info("[MAVLINK] Pixhawk UDP listener %s (fixed port for ETH partner)", spec)
        try:
            return mavutil.mavlink_connection(spec)
        except OSError as exc:
            local_ip = str(self.ethernet.get("local_ip") or "").strip()
            if not local_ip:
                raise
            fallback = f"udpin:0.0.0.0:{self._listen_port()}"
            logger.warning("[MAVLINK] bind %s failed (%s) — retry %s", spec, exc, fallback)
            return mavutil.mavlink_connection(fallback)

    def start_listener(self) -> bool:
        paths = []
        if self.connection_type == MAVLINK_PATH_SERIAL:
            paths = [MAVLINK_PATH_SERIAL]
        elif self.connection_type == "prefer_ethernet":
            paths = [MAVLINK_PATH_ETHERNET, MAVLINK_PATH_SERIAL]
        else:
            paths = [MAVLINK_PATH_ETHERNET]

        for path in paths:
            try:
                self._connections[path] = self._create_connection(path)
            except Exception as exc:
                if self.connection_type == "prefer_ethernet":
                    if path == MAVLINK_PATH_ETHERNET:
                        logger.warning("[MAVLINK] Ethernet listener failed, trying serial backup: %s", exc)
                        continue
                    if path == MAVLINK_PATH_SERIAL:
                        logger.warning("[MAVLINK] Serial backup disabled: %s", exc)
                        continue
                logger.error("[MAVLINK] Failed to open %s listener: %s", path, exc)
                return False

        if not self._connections:
            return False

        self._refresh_active_path(datetime.now(timezone.utc))
        return True

    def _vpn_ready(self) -> bool:
        if not self.vpn_manager or not self.vpn_manager.is_enabled():
            return True
        return self.vpn_manager.is_running() and bool(self.vpn_manager.get_assigned_ip())

    def is_pixhawk_connected(self) -> bool:
        return self._pixhawk_connected.is_set()

    def _create_server_socket(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        vpn_ip = self.vpn_manager.get_assigned_ip() if self.vpn_manager else ""
        if vpn_ip:
            sock.bind((vpn_ip, 0))
            logger.info("[FORWARDER] MAVLink uplink via VPN %s → %s", vpn_ip, self.target_addr)
        elif self.vpn_manager and self.vpn_manager.is_enabled():
            logger.warning(
                "[FORWARDER] VPN chưa sẵn sàng — gói tới %s sẽ không tới server",
                self.target_addr,
            )
        return sock

    def start(self) -> bool:
        if not self.start_listener():
            return False

        self.server_sock = self._create_server_socket()
        self.running = True

        for path, conn in self._connections.items():
            threading.Thread(
                target=self._uplink_loop,
                args=(path, conn),
                daemon=True,
                name=f"forwarder-uplink-{path}",
            ).start()

        threading.Thread(target=self._downlink_loop, daemon=True, name="forwarder-downlink").start()
        threading.Thread(target=self._heartbeat_loop, daemon=True, name="forwarder-heartbeat").start()
        threading.Thread(
            target=self._telemetry_stream_request_loop,
            daemon=True,
            name="forwarder-telemetry-stream-request",
        ).start()
        auth_cfg = getattr(self.config, "auth", {}) or {}
        if float(auth_cfg.get("session_heartbeat_frequency", 1.0) or 0) > 0:
            threading.Thread(
                target=self._mavlink_keepalive_loop,
                daemon=True,
                name="forwarder-mavlink-ka",
            ).start()
        threading.Thread(
            target=self._companion_status_loop,
            daemon=True,
            name="forwarder-companion-status",
        ).start()
        if self.network.get("forward_all_mavlink", False):
            logger.info("[MAVLINK] Full Pixhawk MAVLink uplink enabled")
        elif not forward_gps_raw_int(self.network):
            logger.info("[MAVLINK] GPS_RAW_INT uplink OFF — server uses EKF/global position only")
        self._start_partner_heartbeat()
        threading.Thread(target=self._path_watchdog_loop, daemon=True, name="forwarder-path-watchdog").start()
        threading.Thread(target=self._ip_monitor_loop, daemon=True, name="forwarder-ip-monitor").start()
        threading.Thread(target=self._rate_reporter_loop, daemon=True, name="forwarder-rate-reporter").start()

        logger.info("Forwarder started. Target: %s, mode=%s", self.target_addr, self.connection_type)
        global_metrics.add_log("INFO", f"Forwarder started -> {self.target_addr}")
        return True

    def _note_heartbeat_path(self, path: str) -> str:
        now = datetime.now(timezone.utc)
        with self._path_lock:
            if path == MAVLINK_PATH_ETHERNET:
                self._eth_heartbeat_at = now
            elif path == MAVLINK_PATH_SERIAL:
                self._serial_heartbeat_at = now
            self._refresh_active_path(now)
            return self._active_path

    def _refresh_active_path(self, now: datetime) -> None:
        timeout = self._fallback_timeout_sec()
        eth_fresh = self._eth_heartbeat_at and (now - self._eth_heartbeat_at).total_seconds() <= timeout
        serial_fresh = self._serial_heartbeat_at and (now - self._serial_heartbeat_at).total_seconds() <= timeout

        preferred = self.connection_type
        new_path = self._active_path

        if preferred == MAVLINK_PATH_SERIAL:
            if serial_fresh:
                new_path = MAVLINK_PATH_SERIAL
        elif preferred == "prefer_ethernet":
            if eth_fresh:
                new_path = MAVLINK_PATH_ETHERNET
            elif serial_fresh:
                new_path = MAVLINK_PATH_SERIAL
        else:
            if eth_fresh:
                new_path = MAVLINK_PATH_ETHERNET
            elif serial_fresh:
                new_path = MAVLINK_PATH_SERIAL

        if not new_path and self._connections:
            new_path = next(iter(self._connections.keys()))

        changed = new_path and new_path != self._active_path
        if new_path:
            self._active_path = new_path
            self._active_conn = self._connections.get(new_path)

        eth_ok = self._eth_heartbeat_at is not None
        serial_ok = self._serial_heartbeat_at is not None
        bridge.set_mavlink_path(self._active_path, eth_ok, serial_ok)
        if self._active_conn is not None:
            bridge.set_connection(self._active_conn)
        if changed:
            logger.info("[MAVLINK] Active PX4 path switched to %s", self._active_path)
            global_metrics.add_log("INFO", f"Active PX4 path switched to {self._active_path}")

    def _path_watchdog_loop(self) -> None:
        while self.running:
            with self._path_lock:
                self._refresh_active_path(datetime.now(timezone.utc))
            time.sleep(1)

    def _note_raw_in(self, msg) -> None:
        with self._rate_lock:
            self._rate_raw_in += 1

    def _note_out_server(self, buf: bytes) -> None:
        with self._rate_lock:
            self._rate_out_server += 1
            self._rate_bytes_out += len(buf)

    def _rate_reporter_loop(self) -> None:
        while self.running:
            time.sleep(1)
            with self._rate_lock:
                raw = self._rate_raw_in
                accepted = self._rate_accepted
                out = self._rate_out_server
                bytes_out = self._rate_bytes_out
                self._rate_raw_in = 0
                self._rate_accepted = 0
                self._rate_out_server = 0
                self._rate_bytes_out = 0
            if bytes_out == 0 and out > 0:
                bytes_out = int(out * 120)
            global_metrics.set_udp_rates(raw, accepted, out, bytes_out)

    def _process_uplink_message(self, msg, path: str) -> None:
        msg_type = msg.get_type()
        sys_id = msg.get_srcSystem()
        self._message_last_at[msg_type] = datetime.now(timezone.utc)

        if msg_type in (
            "HEARTBEAT",
            "VFR_HUD",
            "GLOBAL_POSITION_INT",
            "GPS_RAW_INT",
            "SYS_STATUS",
            "BATTERY_STATUS",
        ):
            global_telemetry.feed(msg)

        if msg_type == "GPS_RAW_INT":
            self._gps_last_at = datetime.now(timezone.utc)
            self._gps_fix_type = int(getattr(msg, "fix_type", 0) or 0)
            self._gps_satellites = int(getattr(msg, "satellites_visible", 0) or 0)
        elif msg_type == "BATTERY_STATUS":
            self._battery_status_last_at = datetime.now(timezone.utc)
        elif msg_type == "LOCAL_POSITION_NED":
            self._local_pos_last_at = datetime.now(timezone.utc)

        if msg_type == "COMMAND_ACK":
            logger.info(
                "[VEHICLE_ACK] command=%s result=%s progress=%s result_param2=%s",
                getattr(msg, "command", None),
                getattr(msg, "result", None),
                getattr(msg, "progress", None),
                getattr(msg, "result_param2", None),
            )
        elif msg_type == "STATUSTEXT":
            text = getattr(msg, "text", "")
            if isinstance(text, bytes):
                text = text.decode("utf-8", errors="replace")
            text = str(text).rstrip("\x00")
            if text:
                logger.warning(
                    "[VEHICLE_TEXT] severity=%s %s",
                    getattr(msg, "severity", None),
                    text,
                )
        elif msg_type == "MISSION_COUNT":
            self._mission_item_count = int(getattr(msg, "count", 0) or 0)
            logger.info(
                "[MISSION] Vehicle reports %d stored mission item(s)",
                self._mission_item_count,
            )
        elif msg_type == "MISSION_ACK":
            ack_type = int(getattr(msg, "type", -1))
            logger.info("[MISSION] Vehicle upload ACK type=%d", ack_type)
            if (
                ack_type == mavutil.mavlink.MAV_MISSION_ACCEPTED
                and self._mission_upload_count is not None
            ):
                self._mission_item_count = self._mission_upload_count

        if msg_type == "HEARTBEAT":
            if not is_pixhawk_heartbeat(msg):
                logger.debug(
                    "[REJECT] Non-Pixhawk heartbeat (SysID: %s, Type: %s, Autopilot: %s)",
                    sys_id,
                    getattr(msg, "type", None),
                    getattr(msg, "autopilot", None),
                )
                return

            self._pixhawk_armed = bool(
                int(getattr(msg, "base_mode", 0) or 0)
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self._pixhawk_custom_mode = int(getattr(msg, "custom_mode", 0) or 0)

            active_path = self._note_heartbeat_path(path)
            if not self._pixhawk_connected.is_set():
                self._pixhawk_connected.set()
                self._pixhawk_sys_id = sys_id
                logger.info(
                    "[PIXHAWK_CONNECTED] First heartbeat from Pixhawk (SysID: %s, path: %s)",
                    sys_id,
                    active_path,
                )
                global_metrics.add_log("INFO", f"Pixhawk connected via {active_path}")
            bridge.handle_heartbeat(sys_id, active_path)
            self._flush_pending_mission_start()
            # Continue into the normal uplink path.  A remote GCS/server needs
            # the real ArduPilot heartbeat for vehicle discovery and commands.

        if msg_type == "PARAM_VALUE":
            bridge.handle_param_value(msg)

        if not self._pixhawk_connected.is_set():
            with self.stats_lock:
                self.stats["dropNoPixhawk"] += 1
            return

        with self.stats_lock:
            self.stats["rawIn"] += 1
        with self._rate_lock:
            self._rate_accepted += 1

        if not self._is_healthy:
            with self.stats_lock:
                self.stats["dropUnhealthy"] += 1
            global_metrics.inc_failed_unhealthy(msg_type)
            return

        if not self._vpn_ready():
            with self.stats_lock:
                self.stats["dropVpnNotReady"] += 1
            return

        if not self.auth_client.session_token:
            with self.stats_lock:
                # MAVLink starts before cloud authentication on purpose so the
                # local Pixhawk/GCS link is immediately available. Packets seen
                # in that short startup window are deferred, not failed sends.
                self.stats["dropAuthNotReady"] += 1
            return

        if (
            not self.network.get("forward_all_mavlink", False)
            and msg_type == "GPS_RAW_INT"
            and not forward_gps_raw_int(self.network)
        ):
            return

        try:
            buf = msg.get_msgbuf()
            self.server_sock.sendto(buf, self.target_addr)
            with self.stats_lock:
                self.stats["accepted"] += 1
                self.stats["outServer"] += 1
            self._note_out_server(buf)
            global_metrics.inc_sent(msg_type)
            global_metrics.inc_sent("outServer")

            # ArduPilot commonly exposes pack voltage in SYS_STATUS while the
            # fleet UI consumes BATTERY_STATUS. Preserve the original packet
            # above, then add a compatibility packet only when Pixhawk is not
            # already publishing BATTERY_STATUS itself.
            if msg_type == "SYS_STATUS" and self.network.get("forward_all_mavlink", False):
                now = datetime.now(timezone.utc)
                battery_fresh = (
                    self._battery_status_last_at is not None
                    and (now - self._battery_status_last_at).total_seconds() <= 1.5
                )
                if not battery_fresh:
                    battery_msg = sys_status_to_battery_status(msg)
                    encoder = mavutil.mavlink.MAVLink(
                        None,
                        srcSystem=int(self._pixhawk_sys_id or 1),
                        srcComponent=1,
                    )
                    battery_msg.pack(encoder)
                    self._process_uplink_message(battery_msg, "sys_status_fallback")
        except OSError as exc:
            with self.stats_lock:
                self.stats["dropErr"] += 1
            global_metrics.inc_failed_send(msg_type)
            global_metrics.add_log("ERROR", f"Forward send failed: {exc}")

    def _uplink_loop(self, path: str, conn) -> None:
        while self.running:
            try:
                msg = conn.recv_match(blocking=True, timeout=1.0)
                if msg is None:
                    continue
                self._note_raw_in(msg)
                self._process_uplink_message(msg, path)
            except Exception as exc:
                logger.error("Uplink error on %s: %s", path, exc)
                global_metrics.add_log("ERROR", f"Uplink error on {path}: {exc}")
                time.sleep(1)

    def _downlink_loop(self) -> None:
        while self.running:
            try:
                data, addr = self.server_sock.recvfrom(4096)
                if addr != self.target_addr:
                    continue
                conn = self._active_conn
                if conn is None:
                    continue
                decoded = None
                try:
                    decoded = self._downlink_parser.parse_char(data)
                except Exception:
                    decoded = None
                decoded_type = decoded.get_type() if decoded is not None else None
                if decoded_type == "MISSION_CLEAR_ALL":
                    self._mission_item_count = 0
                    self._mission_upload_count = 0
                    self._mission_first_command = None
                    logger.info("[MISSION] Server requested clearing the vehicle mission")
                elif decoded_type == "MISSION_COUNT":
                    self._mission_upload_count = int(getattr(decoded, "count", 0) or 0)
                    self._mission_first_command = None
                    logger.info(
                        "[MISSION] Server started upload of %d mission item(s)",
                        self._mission_upload_count,
                    )
                elif decoded_type in ("MISSION_ITEM", "MISSION_ITEM_INT"):
                    if int(getattr(decoded, "seq", -1)) == 0:
                        self._mission_first_command = int(
                            getattr(decoded, "command", -1)
                        )
                        logger.info(
                            "[MISSION] First uploaded item command=%d",
                            self._mission_first_command,
                        )

                if decoded_type == "COMMAND_LONG":
                    command = int(getattr(decoded, "command", -1))
                    if command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
                        requested_mode = int(round(float(getattr(decoded, "param2", 0.0))))
                        data = pack_set_mode_command(decoded)
                        logger.info(
                            "[DOWNLINK] Normalized CMD 176: custom mode=%d, param1=1",
                            requested_mode,
                        )
                    elif command == mavutil.mavlink.MAV_CMD_MISSION_START:
                        if self._mission_item_count == 0:
                            logger.error(
                                "[MISSION] START rejected locally: vehicle has zero mission items. "
                                "Upload the route before START MISSION."
                            )
                            continue
                        if self._pixhawk_custom_mode != COPTER_MODE_AUTO:
                            with self._mission_command_lock:
                                self._pending_mission_start = (bytes(data), time.monotonic() + 8.0)
                            mode_frame = pack_set_mode_command(decoded, COPTER_MODE_AUTO)
                            with self._pixhawk_write_lock:
                                conn.write(mode_frame)
                            logger.warning(
                                "[MISSION] Queued CMD 300; requesting AUTO and waiting for heartbeat"
                            )
                            continue
                with self._pixhawk_write_lock:
                    conn.write(data)
            except Exception as exc:
                now = time.time()
                if now - self._downlink_error_at >= 10.0:
                    logger.warning("[DOWNLINK] Server → Pixhawk failed: %s", exc)
                    self._downlink_error_at = now
            time.sleep(0.01)

    def _flush_pending_mission_start(self) -> None:
        with self._mission_command_lock:
            pending = self._pending_mission_start
            if pending is None:
                return
            frame, deadline = pending
            if time.monotonic() > deadline:
                self._pending_mission_start = None
                logger.error(
                    "[MISSION] AUTO not accepted within 8s; CMD 300 cancelled. "
                    "Verify mission upload and first NAV_TAKEOFF item."
                )
                return
            if self._pixhawk_custom_mode != COPTER_MODE_AUTO:
                return
            self._pending_mission_start = None

        conn = self._active_conn
        if conn is None:
            logger.error("[MISSION] AUTO accepted but Pixhawk connection disappeared")
            return
        try:
            with self._pixhawk_write_lock:
                conn.write(frame)
            logger.info("[MISSION] AUTO confirmed by heartbeat; forwarded queued CMD 300")
        except Exception as exc:
            logger.error("[MISSION] Failed to forward queued CMD 300: %s", exc)

    def _start_partner_heartbeat(self) -> None:
        target = self._pixhawk_udp_target()
        conn = self._connections.get(MAVLINK_PATH_ETHERNET)
        if not target or conn is None or not hasattr(conn, "port"):
            if target and conn is None:
                logger.warning("[PARTNER_HB] No ethernet MAVLink listener — partner heartbeat skipped")
            return
        threading.Thread(
            target=self._partner_heartbeat_loop,
            args=(conn.port, target[0], target[1]),
            daemon=True,
            name="partner-heartbeat",
        ).start()

    def _partner_heartbeat_loop(self, sock: socket.socket, pixhawk_ip: str, pixhawk_port: int) -> None:
        """Share pymavlink UDP socket — PX4 unicast goes to same bind as listener (Pi pixhawk_udp.go)."""
        from pymavlink import mavutil as pm

        mav = pm.mavlink.MAVLink(None, srcSystem=255, srcComponent=190)
        target = (pixhawk_ip, pixhawk_port)
        sent = 0
        first = False
        last_log = time.time()
        last_warn = 0.0
        logger.info("[PARTNER_HB] HEARTBEAT 1 Hz via %s → %s:%d", sock.getsockname(), pixhawk_ip, pixhawk_port)
        while self.running:
            try:
                msg = mav.heartbeat_encode(
                    type=pm.mavlink.MAV_TYPE_GCS,
                    autopilot=pm.mavlink.MAV_AUTOPILOT_INVALID,
                    base_mode=0,
                    custom_mode=0,
                    system_status=pm.mavlink.MAV_STATE_ACTIVE,
                )
                sock.sendto(msg.pack(mav), target)
                sent += 1
                if not first:
                    logger.info("[PARTNER_HB] ✓ First HEARTBEAT sent → %s:%d", pixhawk_ip, pixhawk_port)
                    first = True
                elif time.time() - last_log >= 60:
                    logger.info("[PARTNER_HB] active → %s:%d (sent %d)", pixhawk_ip, pixhawk_port, sent)
                    last_log = time.time()
            except OSError as exc:
                now = time.time()
                if sent == 0 or now - last_warn >= 30:
                    logger.warning("[PARTNER_HB] send failed: %s", exc)
                    last_warn = now
            time.sleep(1)

    def _gps_diagnosis(self) -> tuple:
        now = datetime.now(timezone.utc)
        stale_after = 5.0
        if self._gps_last_at and (now - self._gps_last_at).total_seconds() <= stale_after:
            if self._gps_fix_type >= 3 and self._gps_satellites > 0:
                return self._gps_fix_type, self._gps_satellites, 1, GPS_DIAG_PX4_OK
            return self._gps_fix_type, self._gps_satellites, 1, GPS_DIAG_PX4_NO_FIX
        if self._local_pos_last_at and (now - self._local_pos_last_at).total_seconds() <= stale_after:
            return 255, 0, 0, GPS_DIAG_PX4_LOCAL_ONLY
        return 255, 0, 0, GPS_DIAG_NO_PX4_STREAM

    def _telemetry_stream_request_loop(self) -> None:
        """Request server-critical streams while forwarding every received packet."""
        last_requested = {}
        while self.running:
            if not self._pixhawk_connected.wait(timeout=1.0):
                continue

            now = datetime.now(timezone.utc)
            conn = self._active_conn
            target_sys = int(self._pixhawk_sys_id or 1)
            rates = FULL_TELEMETRY_RATES_HZ if self.network.get("forward_all_mavlink", False) else {
                "GPS_RAW_INT": 2.0,
                "BATTERY_STATUS": 1.0,
            }
            monotonic_now = time.monotonic()
            for label, rate_hz in rates.items():
                message_id = getattr(mavutil.mavlink, f"MAVLINK_MSG_ID_{label}", None)
                if message_id is None:
                    continue
                interval_us = max(1, int(round(1_000_000.0 / rate_hz)))
                last_seen = self._message_last_at.get(label)
                stale_after = max(5.0, 3.0 / rate_hz)
                fresh = last_seen is not None and (now - last_seen).total_seconds() <= stale_after
                if fresh or monotonic_now - last_requested.get(message_id, -1.0e9) < 30.0:
                    continue
                if conn is None or not hasattr(conn, "mav"):
                    continue
                try:
                    with self._pixhawk_write_lock:
                        conn.mav.command_long_send(
                            target_sys,
                            1,
                            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                            0,
                            message_id,
                            interval_us,
                            0,
                            0,
                            0,
                            0,
                            0,
                        )
                    last_requested[message_id] = monotonic_now
                    logger.info(
                        "[MAVLINK] Requested %s from Pixhawk at %.1f Hz",
                        label,
                        1_000_000.0 / interval_us,
                    )
                except Exception as exc:
                    logger.warning("[MAVLINK] %s request failed: %s", label, exc)
            for _ in range(5):
                if not self.running:
                    return
                time.sleep(1.0)

    def _camera_live_flags(self) -> tuple:
        try:
            from web.camera_service import read_stream_stats

            cam0 = 1 if read_stream_stats(0, 5.0) else 0
            cam1 = 1 if read_stream_stats(1, 5.0) else 0
            return cam0, cam1
        except Exception:
            return 0, 0

    def _mavlink_keepalive_loop(self) -> None:
        auth_cfg = getattr(self.config, "auth", {}) or {}
        interval = float(auth_cfg.get("session_heartbeat_frequency", 1.0) or 1.0)
        if interval <= 0:
            return
        hb_mode = session_hb_mode()
        sequence = 0
        sys_id = self._pixhawk_sys_id or 1
        while self.running:
            token = self.auth_client.session_token
            expires_at = int(getattr(self.auth_client, "expires_at", 0) or 0)
            pixhawk_active = 1 if self._pixhawk_connected.is_set() else 0
            if token and self.server_sock:
                try:
                    if hb_mode == "shifted" and len(token) == 64:
                        frame = build_session_heartbeat_frame_shifted(
                            sys_id,
                            COMP_ONBOARD,
                            self._mavlink_ka_seq,
                            token,
                            expires_at,
                            sequence,
                            pixhawk_active,
                        )
                    else:
                        frame = build_session_heartbeat_frame(
                            sys_id,
                            COMP_ONBOARD,
                            self._mavlink_ka_seq,
                            token,
                            expires_at,
                            sequence,
                            pixhawk_active,
                        )
                    self.server_sock.sendto(frame, self.target_addr)
                    self._mavlink_ka_seq = (self._mavlink_ka_seq + 1) & 0xFF
                    sequence = (sequence + 1) & 0xFFFF
                except OSError as exc:
                    global_metrics.add_log("WARN", f"MAVLink session keepalive failed: {exc}")
            time.sleep(interval)

    def _companion_status_loop(self) -> None:
        while self.running:
            if self.server_sock and self.auth_client.session_token:
                fix, sats, px4_stream, diag = self._gps_diagnosis()
                cam0, cam1 = self._camera_live_flags()
                try:
                    frame = build_dronebridge_status_frame(
                        COMPANION_SYS_ID,
                        COMP_ONBOARD,
                        self._companion_seq,
                        timestamp_ms=int(time.time() * 1000) & 0xFFFFFFFF,
                        gps_fix_type=fix,
                        gps_satellites=sats,
                        gps_px4_streaming=px4_stream,
                        gps_diagnosis=diag,
                        camera0_live=cam0,
                        camera1_live=cam1,
                    )
                    self.server_sock.sendto(frame, self.target_addr)
                    self._companion_seq = (self._companion_seq + 1) & 0xFF
                except OSError:
                    pass
            time.sleep(1)

    def _heartbeat_loop(self) -> None:
        while self.running:
            packet = self.auth_client.get_session_refresh_packet()
            if packet:
                try:
                    self.server_sock.sendto(packet, self.target_addr)
                except OSError as exc:
                    global_metrics.inc_failed_send("session_refresh")
                    global_metrics.add_log("WARN", f"Session refresh send failed: {exc}")
            time.sleep(1)

    def _ip_monitor_loop(self) -> None:
        network_was_down = False
        while self.running:
            current_ip = get_local_ip()
            network_type, network_speed = detect_network_info()
            global_metrics.set_network_info(network_type, network_speed)

            if not current_ip:
                if not network_was_down:
                    logger.warning("[IP_MONITOR] Network lost (no valid IP)")
                    global_metrics.add_log("WARN", "Network lost - no valid IP")
                    self._is_healthy = False
                    network_was_down = True
            else:
                if network_was_down:
                    logger.info("[IP_MONITOR] Network restored: IP=%s", current_ip)
                    global_metrics.add_log("INFO", f"Network restored: IP={current_ip}")
                    global_metrics.set_ip(current_ip)
                    self._previous_ip = current_ip
                    self._is_healthy = True
                    network_was_down = False
                    if self.auth_client:
                        self.auth_client.force_reconnect()
                elif not self._previous_ip:
                    self._previous_ip = current_ip
                    global_metrics.set_ip(current_ip)
                    global_metrics.add_log("INFO", f"Initial IP: {current_ip}")
                    self._is_healthy = True
                elif self._previous_ip != current_ip:
                    logger.warning("[IP_MONITOR] IP changed: %s -> %s", self._previous_ip, current_ip)
                    global_metrics.add_log("WARN", f"IP changed: {self._previous_ip} -> {current_ip}")
                    global_metrics.set_ip(current_ip)
                    self._previous_ip = current_ip
                    self._is_healthy = True
                    if self.auth_client:
                        self.auth_client.force_reconnect()
                else:
                    global_metrics.set_ip(current_ip)
                    self._is_healthy = True

            time.sleep(5)

    def get_active_connection(self):
        return self._active_conn

    def rebind_vpn_socket(self) -> None:
        if not self.running:
            return
        try:
            new_sock = self._create_server_socket()
            old = self.server_sock
            self.server_sock = new_sock
            if old:
                old.close()
            logger.info("[FORWARDER] UDP sender rebound after VPN up")
        except Exception as exc:
            logger.error("[FORWARDER] VPN socket rebind failed: %s", exc)

    def stop(self) -> None:
        self.running = False
        for conn in self._connections.values():
            try:
                conn.close()
            except Exception:
                pass
        if self.server_sock:
            self.server_sock.close()
