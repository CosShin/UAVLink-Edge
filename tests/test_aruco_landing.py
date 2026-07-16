from __future__ import annotations

import sys
import unittest
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
FIND_LANDING = ROOT / "Find_landing"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(FIND_LANDING))

from landing_mavlink import _fov_from_camera_matrix, _telemetry_control_ready  # noqa: E402
from processing.detectors.aruco.board import duplicate_ids  # noqa: E402
from processing.detectors.aruco.calibration import matrix_for_size  # noqa: E402
from processing.detectors.aruco.compat import create_aruco_detector  # noqa: E402
from processing.detectors.aruco.detect import detect_frame, detect_frame_multiscale  # noqa: E402
from processing.detectors.aruco.marker import load_dictionary  # noqa: E402
from processing.detectors.aruco.track_state import TargetTrackState  # noqa: E402


TEMPLATES = FIND_LANDING / "templates"


def marker_with_quiet_border(marker_id: int, size: int = 180) -> np.ndarray:
    raw = cv2.imread(str(TEMPLATES / f"aruco_dict_4x4_50_id{marker_id}.png"))
    if raw is None:
        raise RuntimeError(f"missing marker template {marker_id}")
    raw = cv2.resize(raw, (size, size), interpolation=cv2.INTER_NEAREST)
    canvas = np.full((size + 80, size + 80, 3), 255, dtype=np.uint8)
    canvas[40 : 40 + size, 40 : 40 + size] = raw
    return canvas


class DetectorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.detector = create_aruco_detector(load_dictionary("DICT_4X4_50"))

    def detect(self, image, **kwargs):
        return detect_frame(
            image,
            (1280, 720),
            self.detector,
            marker_id=5,
            detect_size=(320, 240),
            **kwargs,
        )

    def test_single_target_id_is_selected(self):
        result = self.detect(marker_with_quiet_border(5), target_strategy="single")
        self.assertTrue(result["detected"])
        self.assertEqual(result["aruco_id"], 5)
        self.assertEqual(result["target_key"], "marker:5")
        self.assertGreater(result["quality"], 0.55)

    def test_wrong_id_is_not_a_target(self):
        result = self.detect(marker_with_quiet_border(4), target_strategy="single")
        self.assertFalse(result["detected"])
        self.assertIn("not visible", result["reason"])

    def test_duplicate_target_id_is_ambiguous(self):
        marker = marker_with_quiet_border(5, 140)
        canvas = np.full((300, 600, 3), 255, dtype=np.uint8)
        canvas[20:240, 20:240] = marker
        canvas[20:240, 360:580] = marker
        result = self.detect(canvas, target_strategy="single")
        self.assertFalse(result["detected"])
        self.assertTrue(result["ambiguous"])
        self.assertEqual(result["duplicate_ids"], [5])

    def test_board_fuses_all_visible_markers(self):
        board = cv2.imread(str(TEMPLATES / "aruco_board_dict_4x4_50_0-11.png"))
        self.assertIsNotNone(board)
        result = self.detect(board, target_strategy="board", board_min_markers=2)
        self.assertTrue(result["detected"])
        self.assertEqual(result["aruco_marker_count"], 12)
        self.assertGreater(result["quality"], 0.90)
        self.assertAlmostEqual(result["h_position"][0], 640, delta=8)
        self.assertAlmostEqual(result["h_position"][1], 360, delta=20)

    def test_board_rejects_one_marker(self):
        result = self.detect(
            marker_with_quiet_border(5),
            target_strategy="board",
            board_min_markers=2,
            board_close_single_marker_area_ratio=0.0,
        )
        self.assertFalse(result["detected"])
        self.assertIn("at least 2", result["reason"])

    def test_board_accepts_one_large_close_marker(self):
        result = self.detect(
            marker_with_quiet_border(7, 180),
            target_strategy="board",
            board_min_markers=2,
            board_close_single_marker_area_ratio=0.08,
        )
        self.assertTrue(result["detected"])
        self.assertTrue(result["close_single_marker_fallback"])
        self.assertEqual(result["aruco_visible_ids"], [7])

    def test_multiscale_recovers_small_far_marker(self):
        raw = cv2.imread(str(TEMPLATES / "aruco_dict_4x4_50_id5.png"))
        canvas = np.full((720, 1280, 3), 255, dtype=np.uint8)
        marker = cv2.resize(raw, (20, 20), interpolation=cv2.INTER_NEAREST)
        canvas[350:370, 630:650] = marker
        result = detect_frame_multiscale(
            canvas,
            (1280, 720),
            self.detector,
            detect_sizes=[(320, 180), (960, 540)],
            marker_id=5,
            target_strategy="single",
        )
        self.assertTrue(result["detected"])
        self.assertEqual(result["detection_size"], [960, 540])
        self.assertEqual(result["multiscale_attempts"], 2)

    def test_duplicate_id_helper(self):
        self.assertEqual(duplicate_ids([{"id": 2}, {"id": 3}, {"id": 2}]), [2])


class TrackingStateTests(unittest.TestCase):
    @staticmethod
    def measurement(key="marker:5", quality=0.9):
        return {
            "detected": True,
            "target_key": key,
            "quality": quality,
            "hold": False,
        }

    def test_requires_consecutive_acquisition_frames(self):
        gate = TargetTrackState(min_quality=0.55, acquire_frames=3, reset_ms=1000)
        first = gate.accept(self.measurement(), now=10.0)
        second = gate.accept(self.measurement(), now=10.1)
        third = gate.accept(self.measurement(), now=10.2)
        self.assertEqual(first["tracking_state"], "ACQUIRING")
        self.assertEqual(second["tracking_state"], "ACQUIRING")
        self.assertFalse(second["control_valid"])
        self.assertEqual(third["tracking_state"], "TRACKING")
        self.assertTrue(third["control_valid"])

    def test_loss_blocks_control_and_fast_recovery_preserves_lock(self):
        gate = TargetTrackState(min_quality=0.55, acquire_frames=2, reset_ms=1000)
        gate.accept(self.measurement(), now=20.0)
        gate.accept(self.measurement(), now=20.1)
        lost = gate.accept({"detected": False}, now=20.3)
        recovered = gate.accept(self.measurement(), now=20.4)
        self.assertEqual(lost["tracking_state"], "LOST")
        self.assertFalse(lost["control_valid"])
        self.assertEqual(recovered["tracking_state"], "TRACKING")
        self.assertTrue(recovered["control_valid"])

    def test_low_quality_and_held_frames_never_control(self):
        gate = TargetTrackState(min_quality=0.55, acquire_frames=1, reset_ms=1000)
        low = gate.accept(self.measurement(quality=0.2), now=1.0)
        held = self.measurement()
        held["hold"] = True
        held_result = gate.accept(held, now=1.1)
        self.assertFalse(low["control_valid"])
        self.assertFalse(held_result["control_valid"])

    def test_duplicate_or_target_change_is_ambiguous(self):
        gate = TargetTrackState(min_quality=0.55, acquire_frames=1, reset_ms=1000)
        gate.accept(self.measurement(), now=1.0)
        changed = gate.accept(self.measurement("marker:6"), now=1.1)
        self.assertEqual(changed["tracking_state"], "AMBIGUOUS")
        self.assertFalse(changed["control_valid"])


class CalibrationAndMavlinkGateTests(unittest.TestCase):
    def test_camera_matrix_scales_to_output_resolution(self):
        calibration = {
            "camera_matrix": np.asarray(
                [[1000.0, 0.0, 640.0], [0.0, 900.0, 360.0], [0.0, 0.0, 1.0]]
            ),
            "image_size": (1280, 720),
        }
        scaled = matrix_for_size(calibration, (640, 360))
        self.assertEqual(scaled[0, 0], 500.0)
        self.assertEqual(scaled[1, 1], 450.0)
        self.assertEqual(scaled[0, 2], 320.0)
        self.assertEqual(scaled[1, 2], 180.0)

    def test_fov_is_derived_from_camera_matrix(self):
        hfov, vfov = _fov_from_camera_matrix(
            [[640.0, 0.0, 640.0], [0.0, 360.0, 360.0], [0.0, 0.0, 1.0]],
            [1280, 720],
        )
        self.assertAlmostEqual(hfov, 90.0, places=5)
        self.assertAlmostEqual(vfov, 90.0, places=5)

    def test_mavlink_gate_fails_closed(self):
        base = {
            "detected": True,
            "hold": False,
            "ambiguous": False,
            "control_valid": True,
            "quality": 0.9,
            "measurement_monotonic_ms": 1000,
        }
        kwargs = {
            "min_quality": 0.55,
            "max_measurement_age_ms": 300,
            "require_control_valid": True,
            "now_monotonic_ms": 1200,
        }
        self.assertTrue(_telemetry_control_ready(base, **kwargs))
        for unsafe in (
            {"hold": True},
            {"ambiguous": True},
            {"control_valid": False},
            {"quality": 0.2},
            {"measurement_monotonic_ms": 800},
        ):
            candidate = dict(base)
            candidate.update(unsafe)
            self.assertFalse(_telemetry_control_ready(candidate, **kwargs), unsafe)


if __name__ == "__main__":
    unittest.main()
