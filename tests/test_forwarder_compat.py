from __future__ import annotations

import unittest
from types import SimpleNamespace

from pymavlink import mavutil

from forwarder import pack_set_mode_command, sys_status_to_battery_status


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
            1,
            1,
            mav.MAV_CMD_DO_SET_MODE,
            0,
            3.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        )
        original.pack(encoder)
        normalized = mav.MAVLink(None).parse_char(pack_set_mode_command(original))
        self.assertEqual(normalized.param1, 1.0)
        self.assertEqual(normalized.param2, 3.0)


if __name__ == "__main__":
    unittest.main()
