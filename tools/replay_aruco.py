#!/usr/bin/env python3
"""Offline deterministic replay of the production ArUco processor."""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import cv2


ROOT = Path(__file__).resolve().parents[1]
FIND_LANDING = ROOT / "Find_landing"
sys.path.insert(0, str(FIND_LANDING))

from processing.base import FrameMeta  # noqa: E402
from processing.detectors.aruco import create_processor  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description="Replay video through landing detector")
    parser.add_argument("video")
    parser.add_argument("--config", default=str(FIND_LANDING / "camera_config_0.json"))
    parser.add_argument("--jsonl", default="/tmp/aruco_replay.jsonl")
    parser.add_argument("--realtime", action="store_true")
    args = parser.parse_args()

    config = json.loads(Path(args.config).read_text(encoding="utf-8"))
    processor = create_processor(config, str(FIND_LANDING))
    cap = cv2.VideoCapture(args.video)
    if not cap.isOpened():
        raise SystemExit(f"cannot open video: {args.video}")
    width, height = config["size"]
    total = detected = valid = ambiguous = 0
    started = time.monotonic()
    with Path(args.jsonl).open("w", encoding="utf-8") as handle:
        while True:
            ok, frame = cap.read()
            if not ok:
                break
            total += 1
            if processor.wants_frame(total):
                state = {"detection_result": {"detected": False}}
                processor.process(frame, FrameMeta(total, (width, height)), state)
                result = state["detection_result"]
                detected += int(bool(result.get("detected")))
                valid += int(bool(result.get("control_valid")))
                ambiguous += int(bool(result.get("ambiguous")))
                handle.write(json.dumps(result, default=str) + "\n")
            if args.realtime:
                fps = cap.get(cv2.CAP_PROP_FPS) or 30
                due = total / fps - (time.monotonic() - started)
                if due > 0:
                    time.sleep(due)
    cap.release()
    print(
        json.dumps(
            {
                "frames": total,
                "detected_updates": detected,
                "control_valid_updates": valid,
                "ambiguous_updates": ambiguous,
                "jsonl": args.jsonl,
            },
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

