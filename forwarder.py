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
from metrics import global_metrics
from network_utils import detect_network_info, get_local_ip
from telemetry import global_telemetry
from web.mavlink_bridge import bridge

logger = logging.getLogger("Forwarder")


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
        self._active_path = ""
        self._active_conn = None
        self._eth_heartbeat_at: Optional[datetime] = None
        self._serial_heartbeat_at: Optional[datetime] = None

        self._pixhawk_connected = threading.Event()
        self._pixhawk_sys_id = 0
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
            "dropVpnNotReady": 0,
        }
        self._rate_lock = threading.Lock()
        self._rate_raw_in = 0
        self._rate_accepted = 0
        self._rate_out_server = 0
        self._rate_bytes_out = 0

    def _fallback_timeout_sec(self) -> float:
        timeout = self.ethernet.get("pixhawk_connection_timeout", 30)
        sec = max(3, int(timeout) // 2)
        return float(sec)

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

        logger.info("[MAVLINK] Running with UDP Server only on 0.0.0.0:%s", self.tcp_port)
        return mavutil.mavlink_connection(f"udpin:0.0.0.0:{self.tcp_port}")

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
                if path == MAVLINK_PATH_SERIAL and self.connection_type == "prefer_ethernet":
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

        if msg_type in ("HEARTBEAT", "VFR_HUD", "GLOBAL_POSITION_INT", "GPS_RAW_INT", "SYS_STATUS"):
            global_telemetry.feed(msg)

        if msg_type == "HEARTBEAT":
            if not is_pixhawk_heartbeat(msg):
                logger.debug(
                    "[REJECT] Non-Pixhawk heartbeat (SysID: %s, Type: %s, Autopilot: %s)",
                    sys_id,
                    getattr(msg, "type", None),
                    getattr(msg, "autopilot", None),
                )
                return

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
            return

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
                self.stats["dropErr"] += 1
            global_metrics.inc_failed_unhealthy(msg_type)
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
                conn.write(data)
            except Exception:
                pass
            time.sleep(0.01)

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
