import time

from processing.base import FrameMeta, FrameProcessor
from processing.detect_config import lost_hold_ms, reacquire_ms

from .compat import create_aruco_detector
from .calibration import load_calibration, matrix_for_size
from .detect import detect_frame_multiscale
from .event_log import LandingEventLogger
from .marker import ensure_v2_templates, load_dictionary
from .stability import StableTracker
from .track_state import TargetTrackState


class ArucoProcessor(FrameProcessor):
    def __init__(
        self,
        find_landing_dir: str,
        enabled: bool = True,
        frame_skip: int = 3,
        marker_id: int = 0,
        dictionary: str = "DICT_4X4_50",
        detect_size: tuple[int, int] | None = None,
        lost_hold_ms: int = 1500,
        reacquire_ms: int = 2500,
        camera_id: int = 0,
        target_strategy: str = "single",
        board_first_id: int = 0,
        board_cols: int = 3,
        board_rows: int = 4,
        board_gap_x_ratio: float = 0.16,
        board_gap_y_ratio: float = 0.34,
        board_ransac_threshold_px: float = 3.0,
        board_min_markers: int = 2,
        board_close_single_marker_area_ratio: float = 0.08,
        reacquire_detect_width: int = 960,
        marker_length_m: float = 0.0,
        calibration_file: str = "",
        min_quality: float = 0.55,
        acquire_frames: int = 5,
    ):
        self.enabled = enabled
        self.frame_skip = max(int(frame_skip), 1)
        self.marker_id = int(marker_id)
        if self.marker_id < 0 or self.marker_id > 11:
            raise ValueError(f"aruco_marker_id must be 0–11, got {self.marker_id}")
        self.dictionary = str(dictionary or "DICT_4X4_50").upper()
        self.detect_size = detect_size or (320, 240)
        self.camera_id = int(camera_id)
        self.target_strategy = str(target_strategy or "single").strip().lower()
        if self.target_strategy not in ("single", "board"):
            raise ValueError("aruco_target_strategy must be single or board")
        self.board_first_id = int(board_first_id)
        self.board_cols = max(1, int(board_cols))
        self.board_rows = max(1, int(board_rows))
        self.board_gap_x_ratio = max(0.0, float(board_gap_x_ratio))
        self.board_gap_y_ratio = max(0.0, float(board_gap_y_ratio))
        self.board_ransac_threshold_px = max(0.5, float(board_ransac_threshold_px))
        self.board_min_markers = max(2, int(board_min_markers))
        self.board_close_single_marker_area_ratio = max(
            0.0, min(0.5, float(board_close_single_marker_area_ratio))
        )
        self.reacquire_detect_width = max(0, int(reacquire_detect_width))
        self.marker_length_m = max(0.0, float(marker_length_m))
        self._reacquire_sec = max(int(reacquire_ms), 0) / 1000.0
        self._boost_until = 0.0
        self._stable = StableTracker(lost_hold_ms=lost_hold_ms)
        self._tracking = TargetTrackState(
            min_quality=min_quality,
            acquire_frames=acquire_frames,
            reset_ms=lost_hold_ms,
        )
        self._events = LandingEventLogger(self.camera_id)
        self._calibration_raw = load_calibration(find_landing_dir, calibration_file)
        self._calibration_by_size: dict[tuple[int, int], dict] = {}

        self._detector = create_aruco_detector(load_dictionary(self.dictionary))
        templates = ensure_v2_templates(find_landing_dir, self.dictionary)
        print(
            f" [aruco v3] strategy={self.target_strategy} target ID={self.marker_id} | "
            f"board: {templates['board']} | markers 0–11: {len(templates['markers'])} files | "
            f"quality>={self._tracking.min_quality:.2f} acquire={self._tracking.acquire_frames} | "
            f"reacquire {int(reacquire_ms)}ms | "
            f"calibration={'ON' if self._calibration_raw else 'OFF'}"
        )

    def _calibration_for_size(self, output_size: tuple[int, int]) -> dict | None:
        if self._calibration_raw is None:
            return None
        key = (int(output_size[0]), int(output_size[1]))
        cached = self._calibration_by_size.get(key)
        if cached is not None:
            return cached
        cached = dict(self._calibration_raw)
        cached["camera_matrix"] = matrix_for_size(self._calibration_raw, key)
        cached["image_size"] = key
        self._calibration_by_size[key] = cached
        return cached

    def wants_frame(self, frame_id: int) -> bool:
        if self._reacquire_sec > 0 and time.monotonic() < self._boost_until:
            return True
        return super().wants_frame(frame_id)

    def process(self, frame_bgr, meta: FrameMeta, state: dict) -> None:
        if not self.enabled or not self.wants_frame(meta.frame_id):
            return
        try:
            detect_sizes = [self.detect_size]
            frame_h, frame_w = frame_bgr.shape[:2]
            retry_w = min(self.reacquire_detect_width, frame_w)
            if retry_w > self.detect_size[0]:
                retry_h = max(1, int(round(frame_h * retry_w / max(frame_w, 1))))
                detect_sizes.append((retry_w, retry_h))
            raw = detect_frame_multiscale(
                frame_bgr,
                meta.output_size,
                self._detector,
                marker_id=self.marker_id,
                detect_sizes=detect_sizes,
                target_strategy=self.target_strategy,
                board_first_id=self.board_first_id,
                board_cols=self.board_cols,
                board_rows=self.board_rows,
                board_gap_x_ratio=self.board_gap_x_ratio,
                board_gap_y_ratio=self.board_gap_y_ratio,
                board_ransac_threshold_px=self.board_ransac_threshold_px,
                board_min_markers=self.board_min_markers,
                board_close_single_marker_area_ratio=self.board_close_single_marker_area_ratio,
                calibration=self._calibration_for_size(meta.output_size),
                marker_length_m=self.marker_length_m,
            )
            if raw.get("ambiguous"):
                self._stable.reset()
                stable = raw
            else:
                stable = self._stable.accept(raw, meta.output_size)
            stable = self._tracking.accept(stable or {"detected": False})
            stable["frame_id"] = int(meta.frame_id)
            self._events.write(stable, meta.frame_id)
            if stable and stable.get("detected"):
                if not stable.get("hold") and stable.get("control_valid"):
                    self._boost_until = 0.0
                state["detection_result"] = stable
                if not stable.get("hold"):
                    state["detections_count"] = state.get("detections_count", 0) + 1
            else:
                lost = stable if stable is not None else {"detected": False}
                if (
                    not lost.get("hold")
                    and not lost.get("ambiguous")
                    and self._reacquire_sec > 0
                ):
                    self._boost_until = time.monotonic() + self._reacquire_sec
                state["detection_result"] = lost
        except Exception as e:
            print(f" [aruco v3] detection error: {e}")
            failed = self._tracking.accept({"detected": False, "reason": str(e), "quality": 0.0})
            state["detection_result"] = failed
            if self._reacquire_sec > 0:
                self._boost_until = time.monotonic() + self._reacquire_sec
