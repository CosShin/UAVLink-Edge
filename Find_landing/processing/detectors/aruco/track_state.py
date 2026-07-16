"""Safety-oriented acquisition/target-lock state machine."""

from __future__ import annotations

import time


class TargetTrackState:
    SEARCH = "SEARCH"
    ACQUIRING = "ACQUIRING"
    TRACKING = "TRACKING"
    LOST = "LOST"
    AMBIGUOUS = "AMBIGUOUS"

    def __init__(self, *, min_quality: float = 0.55, acquire_frames: int = 5, reset_ms: int = 1500):
        self.min_quality = max(0.0, min(1.0, float(min_quality)))
        self.acquire_frames = max(1, int(acquire_frames))
        self.reset_sec = max(0, int(reset_ms)) / 1000.0
        self.state = self.SEARCH
        self.consecutive = 0
        self.locked_key: str | None = None
        self.last_valid_at: float | None = None

    def reset(self) -> None:
        self.state = self.SEARCH
        self.consecutive = 0
        self.locked_key = None
        self.last_valid_at = None

    def accept(self, detection: dict, now: float | None = None) -> dict:
        now = time.monotonic() if now is None else float(now)
        out = dict(detection or {})
        target_key = str(out.get("target_key") or "")
        quality = float(out.get("quality", 0.0) or 0.0)
        ambiguous = bool(out.get("ambiguous"))
        held = bool(out.get("hold"))
        detected = bool(out.get("detected")) and not held

        if ambiguous:
            self.state = self.AMBIGUOUS
            self.consecutive = 0
            self.locked_key = None
            self.last_valid_at = None
            reason = str(out.get("reason") or "duplicate target")
            return self._decorate(out, False, reason, now)

        valid_measurement = detected and quality >= self.min_quality and bool(target_key)
        if valid_measurement:
            if self.locked_key and target_key != self.locked_key:
                self.state = self.AMBIGUOUS
                self.consecutive = 0
                self.locked_key = None
                self.last_valid_at = None
                return self._decorate(out, False, "target key changed while locked", now)

            recover_locked = (
                self.state == self.LOST
                and
                self.locked_key == target_key
                and self.last_valid_at is not None
                and now - self.last_valid_at <= self.reset_sec
            )
            if self.state == self.TRACKING or recover_locked:
                self.state = self.TRACKING
                self.consecutive = max(self.consecutive, self.acquire_frames)
            else:
                if self.locked_key != target_key:
                    self.consecutive = 0
                self.locked_key = target_key
                self.consecutive += 1
                self.state = (
                    self.TRACKING if self.consecutive >= self.acquire_frames else self.ACQUIRING
                )
            self.locked_key = target_key
            self.last_valid_at = now
            return self._decorate(
                out,
                self.state == self.TRACKING,
                "tracking" if self.state == self.TRACKING else "confirming target",
                now,
            )

        self.consecutive = 0
        age = None if self.last_valid_at is None else now - self.last_valid_at
        if self.locked_key and age is not None and age <= self.reset_sec:
            self.state = self.LOST
            reason = "target temporarily lost" if not detected else "quality below threshold"
        else:
            self.state = self.SEARCH
            self.locked_key = None
            self.last_valid_at = None
            reason = "searching" if not detected else "quality below threshold"
        return self._decorate(out, False, reason, now)

    def _decorate(self, out: dict, control_valid: bool, reason: str, now: float) -> dict:
        out["tracking_state"] = self.state
        out["control_valid"] = bool(control_valid)
        out["tracking_reason"] = reason
        out["acquire_count"] = int(self.consecutive)
        out["acquire_required"] = int(self.acquire_frames)
        out["locked_target"] = self.locked_key
        out["quality_threshold"] = self.min_quality
        out["measurement_monotonic_ms"] = int(now * 1000)
        return out
