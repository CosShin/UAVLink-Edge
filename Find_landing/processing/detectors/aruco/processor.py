import time

from processing.base import FrameMeta, FrameProcessor
from processing.detect_config import lost_hold_ms, reacquire_ms

from .compat import create_aruco_detector
from .detect import detect_frame
from .marker import ensure_v2_templates, load_dictionary
from .stability import StableTracker


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
    ):
        self.enabled = enabled
        self.frame_skip = max(int(frame_skip), 1)
        self.marker_id = int(marker_id)
        if self.marker_id < 0 or self.marker_id > 11:
            raise ValueError(f"aruco_marker_id must be 0–11, got {self.marker_id}")
        self.dictionary = str(dictionary or "DICT_4X4_50").upper()
        self.detect_size = detect_size or (320, 240)
        self._reacquire_sec = max(int(reacquire_ms), 0) / 1000.0
        self._boost_until = 0.0
        self._stable = StableTracker(lost_hold_ms=lost_hold_ms)

        self._detector = create_aruco_detector(load_dictionary(self.dictionary))
        templates = ensure_v2_templates(find_landing_dir, self.dictionary)
        print(
            f" [aruco v2] track ID={self.marker_id} only | "
            f"board: {templates['board']} | markers 0–11: {len(templates['markers'])} files | "
            f"reacquire {int(reacquire_ms)}ms"
        )

    def wants_frame(self, frame_id: int) -> bool:
        if self._reacquire_sec > 0 and time.monotonic() < self._boost_until:
            return True
        return super().wants_frame(frame_id)

    def process(self, frame_bgr, meta: FrameMeta, state: dict) -> None:
        if not self.enabled or not self.wants_frame(meta.frame_id):
            return
        try:
            raw = detect_frame(
                frame_bgr,
                meta.output_size,
                self._detector,
                marker_id=self.marker_id,
                detect_size=self.detect_size,
            )
            stable = self._stable.accept(raw, meta.output_size)
            if stable and stable.get("detected"):
                if not stable.get("hold"):
                    self._boost_until = 0.0
                state["detection_result"] = stable
                if not stable.get("hold"):
                    state["detections_count"] = state.get("detections_count", 0) + 1
            else:
                lost = stable if stable is not None else {"detected": False}
                if not lost.get("hold") and self._reacquire_sec > 0:
                    self._boost_until = time.monotonic() + self._reacquire_sec
                state["detection_result"] = lost
        except Exception as e:
            print(f" [aruco v2] detection error: {e}")
            state["detection_result"] = {"detected": False}
            if self._reacquire_sec > 0:
                self._boost_until = time.monotonic() + self._reacquire_sec
