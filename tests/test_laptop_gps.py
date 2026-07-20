import unittest

from pymavlink import mavutil

from tools.laptop_gps_receiver import (
    heartbeat_is_armed,
    parse_browser_fix,
    velocity_ne,
)


class LaptopGpsTests(unittest.TestCase):
    def test_browser_fix_and_velocity(self):
        fix = parse_browser_fix({
            "latitude": 21.0285,
            "longitude": 105.8542,
            "accuracy_m": 12.0,
            "altitude_m": None,
            "speed_m_s": 2.0,
            "heading_deg": 90.0,
        }, now_monotonic=123.0)
        self.assertEqual(fix.received_monotonic, 123.0)
        north, east = velocity_ne(fix)
        self.assertAlmostEqual(north, 0.0, places=7)
        self.assertAlmostEqual(east, 2.0, places=7)

    def test_invalid_accuracy_is_rejected(self):
        with self.assertRaises(ValueError):
            parse_browser_fix({"latitude": 1, "longitude": 2, "accuracy_m": -1})

    def test_armed_heartbeat_is_detected(self):
        heartbeat = type("Heartbeat", (), {
            "base_mode": mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED,
        })()
        self.assertTrue(heartbeat_is_armed(heartbeat))


if __name__ == "__main__":
    unittest.main()
