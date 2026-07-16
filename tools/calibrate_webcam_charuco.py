#!/usr/bin/env python3
"""Calibrate the exact webcam mode used by precision landing with a ChArUco board."""

from __future__ import annotations

import argparse
import glob
import time
from pathlib import Path

import cv2
import numpy as np
import yaml


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="ChArUco webcam calibration")
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-length-m", type=float, default=0.04)
    parser.add_argument("--marker-length-m", type=float, default=0.03)
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument("--samples", type=int, default=25)
    parser.add_argument("--images", help="Optional image glob instead of live camera")
    parser.add_argument("--auto", action="store_true", help="Capture a valid sample every 0.7 s")
    parser.add_argument("--generate-board", help="Write a printable board PNG and exit")
    parser.add_argument(
        "--output",
        default="Find_landing/camera_calibration_1280x720.yaml",
    )
    return parser


def create_board(args):
    dictionary_id = getattr(cv2.aruco, args.dictionary.upper(), cv2.aruco.DICT_4X4_50)
    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = cv2.aruco.CharucoBoard(
        (args.squares_x, args.squares_y),
        float(args.square_length_m),
        float(args.marker_length_m),
        dictionary,
    )
    detector = cv2.aruco.CharucoDetector(board)
    return board, detector


def detect_sample(detector, frame):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, marker_corners, marker_ids = detector.detectBoard(gray)
    return corners, ids, marker_corners, marker_ids


def add_sample(board, frame, image_points, object_points) -> bool:
    detector = cv2.aruco.CharucoDetector(board)
    corners, ids, _, _ = detect_sample(detector, frame)
    if ids is None or corners is None or len(ids) < 8:
        return False
    chessboard = np.asarray(board.getChessboardCorners(), dtype=np.float32)
    selected = chessboard[np.asarray(ids, dtype=np.int32).reshape(-1)]
    object_points.append(selected.reshape(-1, 3))
    image_points.append(np.asarray(corners, dtype=np.float32).reshape(-1, 2))
    return True


def collect_from_images(args, board):
    object_points, image_points = [], []
    image_size = None
    paths = sorted(glob.glob(args.images))
    if not paths:
        raise RuntimeError(f"no calibration images match: {args.images}")
    for path in paths:
        frame = cv2.imread(path)
        if frame is None:
            continue
        image_size = (frame.shape[1], frame.shape[0])
        if add_sample(board, frame, image_points, object_points):
            print(f"accepted {path} ({len(image_points)}/{args.samples})")
        if len(image_points) >= args.samples:
            break
    return object_points, image_points, image_size


def collect_live(args, board, detector):
    cap = cv2.VideoCapture(args.device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_FPS, args.fps)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {args.device}")

    object_points, image_points = [], []
    image_size = None
    last_auto = 0.0
    print("SPACE: capture | Q/ESC: finish. Move/tilt board across the whole image.")
    try:
        while len(image_points) < args.samples:
            ok, frame = cap.read()
            if not ok:
                continue
            image_size = (frame.shape[1], frame.shape[0])
            corners, ids, marker_corners, marker_ids = detect_sample(detector, frame)
            preview = frame.copy()
            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)
            if ids is not None and corners is not None:
                cv2.aruco.drawDetectedCornersCharuco(preview, corners, ids)
            cv2.putText(
                preview,
                f"samples {len(image_points)}/{args.samples}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (0, 255, 0),
                2,
            )
            cv2.imshow("ChArUco calibration", preview)
            key = cv2.waitKey(1) & 0xFF
            auto_due = args.auto and time.monotonic() - last_auto >= 0.7
            if key == 32 or auto_due:
                if add_sample(board, frame, image_points, object_points):
                    print(f"accepted sample {len(image_points)}/{args.samples}")
                    last_auto = time.monotonic()
            if key in (27, ord("q")):
                break
    finally:
        cap.release()
        cv2.destroyAllWindows()
    return object_points, image_points, image_size


def main() -> int:
    args = build_parser().parse_args()
    if args.square_length_m <= args.marker_length_m or args.marker_length_m <= 0:
        raise SystemExit("square-length-m must be greater than marker-length-m > 0")
    board, detector = create_board(args)
    if args.generate_board:
        image = board.generateImage((2100, 1500), marginSize=40, borderBits=1)
        path = Path(args.generate_board)
        path.parent.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(path), image)
        print(f"board written: {path}")
        return 0

    if args.images:
        object_points, image_points, image_size = collect_from_images(args, board)
    else:
        object_points, image_points, image_size = collect_live(args, board, detector)
    if image_size is None or len(image_points) < 10:
        raise SystemExit("need at least 10 diverse valid samples")

    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    per_view = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        error = np.linalg.norm(projected.reshape(-1, 2) - img, axis=1)
        per_view.append(float(np.sqrt(np.mean(np.square(error)))))
    payload = {
        "version": 1,
        "device": args.device,
        "image_size": [int(image_size[0]), int(image_size[1])],
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "rms": float(rms),
        "per_view_rmse_px": per_view,
        "samples": len(image_points),
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_m": args.square_length_m,
            "marker_length_m": args.marker_length_m,
            "dictionary": args.dictionary.upper(),
        },
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")
    print(f"calibration written: {output}")
    print(f"RMS: {rms:.4f} px | views: {len(per_view)} | max view RMSE: {max(per_view):.4f} px")
    if rms > 1.0:
        print("WARNING: RMS > 1 px; collect sharper, more diverse views before flight use")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

