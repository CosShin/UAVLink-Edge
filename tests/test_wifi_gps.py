from __future__ import annotations

import json
import math
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace

from pymavlink import mavutil

from forwarder import gps_input_to_raw_int, pack_set_mode_command, sys_status_to_battery_status
from wifi_gps import (
    GpsFix,
    HybridVisionGps,
    board_xy_to_ne,
    camera_position_on_board,
    global_offset,
)


class HybridVisionMathTests(unittest.TestCase):
    def test_global_offset_one_meter(self):
        lat, lon = global_offset(0.0, 0.0, 1.0, 1.0)
        expected = math.degrees(1.0 / 6378137.0)
        self.assertAlmostEqual(lat, expected, places=12)
        self.assertAlmostEqual(lon, expected, places=12)

    def test_board_heading_rotation(self):
        self.assertEqual(board_xy_to_ne(2.0, 3.0, 0.0), (2.0, 3.0))
        north, east = board_xy_to_ne(2.0, 3.0, 90.0)
        self.assertAlmostEqual(north, -3.0, places=9)
        self.assertAlmostEqual(east, 2.0, places=9)

    def test_solvepnp_pose_is_inverted(self):
        x, y, z = camera_position_on_board([1.0, 2.0, 3.0], [0.0, 0.0, 0.0])
        self.assertEqual((x, y, z), (-1.0, -2.0, -3.0))


class WifiGpsServerRelayTests(unittest.TestCase):
    def test_gps_input_is_converted_for_server_ui(self):
        source = SimpleNamespace(
            time_usec=123456,
            ignore_flags=0,
            fix_type=3,
            lat=107626220,
            lon=1066601720,
            alt=12.5,
            hdop=0.8,
            vdop=1.2,
            vn=3.0,
            ve=4.0,
            satellites_visible=10,
        )
        output = gps_input_to_raw_int(source)
        self.assertEqual(output.get_type(), "GPS_RAW_INT")
        self.assertEqual((output.lat, output.lon, output.alt), (107626220, 1066601720, 12500))
        self.assertEqual((output.eph, output.epv), (80, 120))
        self.assertEqual(output.vel, 500)
        self.assertEqual(output.cog, 5313)
        self.assertEqual(output.satellites_visible, 10)

    def test_unknown_velocity_stays_unknown(self):
        source = SimpleNamespace(
            ignore_flags=mavutil.mavlink.GPS_INPUT_IGNORE_FLAG_VEL_HORIZ,
            fix_type=3,
            lat=1,
            lon=2,
            alt=0.0,
            hdop=1.0,
            vdop=1.0,
            satellites_visible=8,
        )
        output = gps_input_to_raw_int(source)
        self.assertEqual(output.vel, 65535)
        self.assertEqual(output.cog, 65535)


class BatteryServerRelayTests(unittest.TestCase):
    def test_sys_status_voltage_is_preserved_for_server(self):
        source = SimpleNamespace(
            voltage_battery=11800,
            current_battery=245,
            battery_remaining=76,
        )
        output = sys_status_to_battery_status(source)
        self.assertEqual(output.get_type(), "BATTERY_STATUS")
        self.assertEqual(output.voltages[0], 11800)
        self.assertTrue(all(value == 65535 for value in output.voltages[1:]))
        self.assertEqual(output.current_battery, 245)
        self.assertEqual(output.battery_remaining, 76)


class MissionCommandCompatibilityTests(unittest.TestCase):
    def test_set_mode_command_is_normalized_for_arducopter(self):
        mav = mavutil.mavlink
        source_encoder = mav.MAVLink(None, srcSystem=42, srcComponent=190)
        original = mav.MAVLink_command_long_message(
            1,
            1,
            mav.MAV_CMD_DO_SET_MODE,
            0,
            0.0,
            3.0,
            99.0,
            99.0,
            99.0,
            99.0,
            99.0,
        )
        original.pack(source_encoder)
        parser = mav.MAVLink(None)
        normalized = parser.parse_char(pack_set_mode_command(original))
        self.assertEqual(normalized.command, mav.MAV_CMD_DO_SET_MODE)
        self.assertEqual(normalized.target_system, 1)
        self.assertEqual(normalized.param1, mav.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED)
        self.assertEqual(normalized.param2, 3.0)
        self.assertEqual(normalized.param3, 0.0)

    def test_mission_start_can_generate_auto_mode_request(self):
        mav = mavutil.mavlink
        source_encoder = mav.MAVLink(None, srcSystem=42, srcComponent=190)
        mission_start = mav.MAVLink_command_long_message(
            1,
            1,
            mav.MAV_CMD_MISSION_START,
            0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        mission_start.pack(source_encoder)
        parser = mav.MAVLink(None)
        request = parser.parse_char(pack_set_mode_command(mission_start, 3))
        self.assertEqual(request.command, mav.MAV_CMD_DO_SET_MODE)
        self.assertEqual(request.param1, 1.0)
        self.assertEqual(request.param2, 3.0)

    def test_legacy_mode_in_param1_is_moved_to_param2(self):
        mav = mavutil.mavlink
        encoder = mav.MAVLink(None, srcSystem=42, srcComponent=190)
        original = mav.MAVLink_command_long_message(
            1, 1, mav.MAV_CMD_DO_SET_MODE, 0,
            3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        )
        original.pack(encoder)
        normalized = mav.MAVLink(None).parse_char(pack_set_mode_command(original))
        self.assertEqual(normalized.param1, 1.0)
        self.assertEqual(normalized.param2, 3.0)


class HybridVisionGpsTests(unittest.TestCase):
    def setUp(self):
        self.tempdir = tempfile.TemporaryDirectory()
        self.path = Path(self.tempdir.name) / "camera.json"
        self.anchor = GpsFix(lat=10.0, lon=106.0, alt_m=12.0, horiz_accuracy=20.0)
        self.hybrid = HybridVisionGps(
            self.path,
            heading_deg=0.0,
            timeout_s=0.5,
            min_quality=0.55,
            horizontal_accuracy_m=0.5,
            max_radius_m=5.0,
            max_step_m=2.0,
        )

    def tearDown(self):
        self.tempdir.cleanup()

    def write_pose(self, tvec, **updates):
        payload = {
            "updated_at": time.time(),
            "detected": True,
            "hold": False,
            "ambiguous": False,
            "control_valid": True,
            "quality": 0.9,
            "pose_valid": True,
            "pose_camera_m": tvec,
            "rvec": [0.0, 0.0, 0.0],
            "target_key": "board:0-11",
        }
        payload.update(updates)
        self.path.write_text(json.dumps(payload), encoding="utf-8")

    def test_latches_wifi_anchor_and_adds_camera_displacement(self):
        self.write_pose([0.0, 0.0, 2.0])
        first = self.hybrid.make_fix(self.anchor)
        self.assertIsNotNone(first)
        self.assertAlmostEqual(first.lat, self.anchor.lat)
        self.assertEqual(first.horiz_accuracy, 0.5)

        # For an identity rotation C=-t. Moving tvec X from 0 to -1 means
        # the camera moved +1 m on board X, configured here as North.
        self.write_pose([-1.0, 0.0, 2.0])
        second = self.hybrid.make_fix(self.anchor)
        self.assertIsNotNone(second)
        expected_lat, expected_lon = global_offset(10.0, 106.0, 1.0, 0.0)
        self.assertAlmostEqual(second.lat, expected_lat, places=10)
        self.assertAlmostEqual(second.lon, expected_lon, places=10)
        self.assertAlmostEqual(self.hybrid.north_m, 1.0)

    def test_fails_closed_for_stale_or_invalid_pose(self):
        self.write_pose([0.0, 0.0, 2.0], updated_at=time.time() - 2.0)
        self.assertIsNone(self.hybrid.make_fix(self.anchor))
        self.assertIn("stale", self.hybrid.reason)

        self.write_pose([0.0, 0.0, 2.0], control_valid=False)
        self.assertIsNone(self.hybrid.make_fix(self.anchor))
        self.assertIn("TRACKING", self.hybrid.reason)

    def test_rejects_pose_jump(self):
        self.write_pose([0.0, 0.0, 2.0])
        self.assertIsNotNone(self.hybrid.make_fix(self.anchor))
        self.write_pose([-3.0, 0.0, 2.0])
        self.assertIsNone(self.hybrid.make_fix(self.anchor))
        self.assertIn("nhảy", self.hybrid.reason)


if __name__ == "__main__":
    unittest.main()
