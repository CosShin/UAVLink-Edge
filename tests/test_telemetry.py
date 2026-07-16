import unittest
from types import SimpleNamespace

from telemetry import _flight_mode


class FlightModeDecodeTests(unittest.TestCase):
    def test_arducopter_stabilize(self):
        msg = SimpleNamespace(autopilot=3, custom_mode=0)
        self.assertEqual(_flight_mode(msg), "STABILIZE")

    def test_arducopter_auto(self):
        msg = SimpleNamespace(autopilot=3, custom_mode=3)
        self.assertEqual(_flight_mode(msg), "AUTO")

    def test_px4_position_mode(self):
        msg = SimpleNamespace(autopilot=12, custom_mode=3 << 16)
        self.assertEqual(_flight_mode(msg), "POSCTL")


if __name__ == "__main__":
    unittest.main()
