#!/usr/bin/env python3
"""Generate controlled LANDING_TARGET scenarios for ArduCopter SITL only."""

from __future__ import annotations

import argparse
import math
import random
import time

from pymavlink import mavutil


def main() -> int:
    parser = argparse.ArgumentParser(description="LANDING_TARGET scenario generator for SITL")
    parser.add_argument(
        "--endpoint",
        default="udpin:127.0.0.1:14550",
        help="Loopback link receiving SITL/MAVProxy output; replies go back to SITL",
    )
    parser.add_argument("--pattern", choices=("center", "step", "sine"), default="center")
    parser.add_argument("--amplitude-deg", type=float, default=8.0)
    parser.add_argument("--frequency", type=float, default=0.2)
    parser.add_argument("--rate", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=60.0)
    parser.add_argument("--noise-deg", type=float, default=0.0)
    parser.add_argument("--packet-loss", type=float, default=0.0)
    parser.add_argument("--dropout-start", type=float, default=-1.0)
    parser.add_argument("--dropout-duration", type=float, default=0.0)
    parser.add_argument("--sitl-confirm", action="store_true")
    args = parser.parse_args()
    if not args.sitl_confirm:
        raise SystemExit("refusing to send: add --sitl-confirm and use only with SITL")
    allowed = (
        "udpin:127.0.0.1:",
        "udpin:localhost:",
        "udpout:127.0.0.1:",
        "udpout:localhost:",
        "tcp:127.0.0.1:",
        "tcp:localhost:",
    )
    if not args.endpoint.startswith(allowed):
        raise SystemExit("SITL tool only permits loopback MAVLink endpoints")
    rate = max(1.0, min(50.0, args.rate))
    packet_loss = max(0.0, min(1.0, args.packet_loss))
    conn = mavutil.mavlink_connection(args.endpoint, source_system=245, source_component=191)
    if args.endpoint.startswith(("udpin:", "tcp:")):
        print(f"waiting for SITL heartbeat on {args.endpoint} ...")
        if conn.wait_heartbeat(timeout=10) is None:
            raise SystemExit("no SITL heartbeat received within 10 seconds")
        print(f"SITL heartbeat: system={conn.target_system} component={conn.target_component}")
    started = time.monotonic()
    sent = dropped = 0
    while time.monotonic() - started < args.duration:
        t = time.monotonic() - started
        in_dropout = args.dropout_start >= 0 and args.dropout_start <= t < args.dropout_start + args.dropout_duration
        if args.pattern == "step":
            x_deg = args.amplitude_deg if t >= args.duration / 2 else -args.amplitude_deg
            y_deg = 0.0
        elif args.pattern == "sine":
            x_deg = args.amplitude_deg * math.sin(2 * math.pi * args.frequency * t)
            y_deg = args.amplitude_deg * math.cos(2 * math.pi * args.frequency * t)
        else:
            x_deg = y_deg = 0.0
        x_deg += random.gauss(0.0, max(args.noise_deg, 0.0))
        y_deg += random.gauss(0.0, max(args.noise_deg, 0.0))
        if in_dropout or random.random() < packet_loss:
            dropped += 1
        else:
            conn.mav.landing_target_send(
                time.monotonic_ns() // 1000,
                0,
                mavutil.mavlink.MAV_FRAME_BODY_FRD,
                math.radians(x_deg),
                math.radians(y_deg),
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                0.0,
                [1.0, 0.0, 0.0, 0.0],
                mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
                0,
            )
            sent += 1
        time.sleep(1.0 / rate)
    conn.close()
    print(f"scenario complete: sent={sent}, dropped={dropped}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
