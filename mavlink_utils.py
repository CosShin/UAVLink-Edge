from pymavlink import mavutil

MAVLINK_PATH_ETHERNET = "ethernet"
MAVLINK_PATH_SERIAL = "serial"


def normalize_connection_type(conn_type: str) -> str:
    value = (conn_type or "").strip().lower()
    if value in ("", "ethernet", "udp", "udp_listen", "tcp_listen", "tcp_client"):
        return MAVLINK_PATH_ETHERNET
    if value in ("serial", "uart"):
        return MAVLINK_PATH_SERIAL
    if value in ("prefer_ethernet", "dual", "auto"):
        return "prefer_ethernet"
    return value


def is_pixhawk_heartbeat(msg) -> bool:
    if msg is None or msg.get_type() != "HEARTBEAT":
        return False

    mav = mavutil.mavlink
    mav_type = getattr(msg, "type", None)
    autopilot = getattr(msg, "autopilot", None)

    if mav_type in (mav.MAV_TYPE_GCS, mav.MAV_TYPE_ONBOARD_CONTROLLER):
        return False

    if autopilot in (
        mav.MAV_AUTOPILOT_PX4,
        mav.MAV_AUTOPILOT_ARDUPILOTMEGA,
        mav.MAV_AUTOPILOT_GENERIC,
    ):
        return True

    if autopilot == mav.MAV_AUTOPILOT_INVALID:
        return False

    if mav_type in (
        mav.MAV_TYPE_QUADROTOR,
        mav.MAV_TYPE_VTOL_TAILSITTER_QUADROTOR,
        mav.MAV_TYPE_FIXED_WING,
        mav.MAV_TYPE_HELICOPTER,
        mav.MAV_TYPE_VTOL_TILTROTOR,
    ):
        return True

    return False
