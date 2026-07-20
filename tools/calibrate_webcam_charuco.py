#!/usr/bin/env python3
"""Calibrate the exact webcam mode used by precision landing with a ChArUco board."""

from __future__ import annotations

import argparse
import glob
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import os
import sys
import threading
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
    parser.add_argument(
        "--capture-interval",
        type=float,
        default=0.7,
        help="Seconds between automatic samples",
    )
    parser.add_argument("--preview-width", type=int, default=640)
    parser.add_argument("--preview-fps", type=float, default=10.0)
    parser.add_argument("--preview-port", type=int, default=8765)
    parser.add_argument("--capture-dir", help="Save accepted live frames as JPEG files")
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


def highgui_available() -> bool:
    """Return whether this OpenCV build can safely open a preview window."""
    gui_line = next(
        (line for line in cv2.getBuildInformation().splitlines() if "GUI:" in line),
        "",
    )
    if not gui_line or gui_line.split("GUI:", 1)[1].strip().upper() == "NONE":
        return False
    if sys.platform.startswith("linux"):
        return bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY"))
    return True


def draw_charuco_corners(preview, corners, ids) -> None:
    """Normalize OpenCV 5's flat arrays to the shape expected by its drawer."""
    draw_corners = np.asarray(corners, dtype=np.float32).reshape(-1, 1, 2)
    draw_ids = np.asarray(ids, dtype=np.int32).reshape(-1, 1)
    if len(draw_corners) == len(draw_ids):
        cv2.aruco.drawDetectedCornersCharuco(preview, draw_corners, draw_ids)


class BrowserPreview:
    """Small temporary HTTP preview for headless calibration hosts."""

    def __init__(self, port: int):
        self._jpeg = None
        self._lock = threading.Lock()
        self._viewer_ready = threading.Event()
        owner = self

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                if self.path.startswith("/preview.jpg"):
                    with owner._lock:
                        payload = owner._jpeg
                    if payload is None:
                        self.send_error(503, "Camera frame not ready")
                        return
                    self.send_response(200)
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", str(len(payload)))
                    self.end_headers()
                    self.wfile.write(payload)
                    return
                page = b"""<!doctype html>
<meta name=viewport content="width=device-width,initial-scale=1">
<title>ChArUco calibration</title>
<style>body{margin:0;background:#111;color:#eee;font:16px sans-serif;text-align:center}img{max-width:100vw;max-height:92vh}</style>
<p>ChArUco calibration preview</p><img id=p alt="Waiting for camera frame">
<script>const p=document.getElementById('p');setInterval(()=>p.src='/preview.jpg?t='+Date.now(),150)</script>"""
                owner._viewer_ready.set()
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)

            def log_message(self, format, *args):
                pass

        self._server = ThreadingHTTPServer(("0.0.0.0", port), Handler)
        self.port = int(self._server.server_address[1])
        self._thread = threading.Thread(target=self._server.serve_forever, daemon=True)
        self._thread.start()

    def update(self, frame) -> None:
        ok, encoded = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self._lock:
                self._jpeg = encoded.tobytes()

    def wait_for_viewer(self, timeout: float = 60.0) -> bool:
        return self._viewer_ready.wait(timeout)

    def close(self) -> None:
        self._server.shutdown()
        self._server.server_close()
        self._thread.join(timeout=2.0)


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
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open {args.device}")

    object_points, image_points = [], []
    image_size = None
    last_auto = 0.0
    last_visual = 0.0
    preview_fps = max(1.0, float(getattr(args, "preview_fps", 10.0)))
    preview_width = max(320, int(getattr(args, "preview_width", 640)))
    capture_dir_value = getattr(args, "capture_dir", None)
    capture_dir = Path(capture_dir_value) if capture_dir_value else None
    if capture_dir is not None:
        capture_dir.mkdir(parents=True, exist_ok=True)
    show_preview = highgui_available()
    browser_preview = None
    if show_preview:
        print("SPACE: capture | Q/ESC: finish. Move/tilt board across the whole image.")
    elif args.auto:
        preview_port = int(getattr(args, "preview_port", 8765))
        try:
            browser_preview = BrowserPreview(preview_port)
            print(f"Browser preview: http://<IP-CM5>:{browser_preview.port}")
            print("Waiting up to 60 seconds for the preview page to open...")
            if not browser_preview.wait_for_viewer():
                print("No browser connected; starting capture without a viewer.")
        except OSError as exc:
            print(f"Browser preview unavailable on port {preview_port}: {exc}")
        print("OpenCV has no GUI; running auto-capture in headless mode.")
        print("Move/tilt the board after each accepted sample; press Ctrl+C to stop.")
    else:
        cap.release()
        raise RuntimeError("OpenCV has no GUI; rerun with --auto for headless capture")
    try:
        while len(image_points) < args.samples:
            ok, frame = cap.read()
            if not ok:
                continue
            image_size = (frame.shape[1], frame.shape[0])
            key = -1
            if show_preview:
                key = cv2.waitKey(1) & 0xFF
            now = time.monotonic()
            capture_interval = max(0.1, float(getattr(args, "capture_interval", 0.7)))
            auto_due = args.auto and now - last_auto >= capture_interval
            visual_due = (
                (show_preview or browser_preview is not None)
                and now - last_visual >= 1.0 / preview_fps
            )
            if visual_due:
                corners, ids, marker_corners, marker_ids = detect_sample(detector, frame)
                preview = frame.copy()
                if marker_ids is not None:
                    cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)
                if ids is not None and corners is not None:
                    draw_charuco_corners(preview, corners, ids)
                cv2.putText(
                    preview,
                    f"samples {len(image_points)}/{args.samples}",
                    (20, 35),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
                if preview.shape[1] != preview_width:
                    preview_height = round(preview.shape[0] * preview_width / preview.shape[1])
                    preview = cv2.resize(
                        preview,
                        (preview_width, preview_height),
                        interpolation=cv2.INTER_AREA,
                    )
                last_visual = now
            if browser_preview is not None:
                if visual_due:
                    browser_preview.update(preview)
            if show_preview and visual_due:
                cv2.imshow("ChArUco calibration", preview)
            if key == 32 or auto_due:
                if add_sample(board, frame, image_points, object_points):
                    if capture_dir is not None:
                        destination = capture_dir / f"sample_{len(image_points):03d}.jpg"
                        if destination.exists():
                            raise RuntimeError(
                                f"refusing to overwrite existing capture: {destination}"
                            )
                        if not cv2.imwrite(str(destination), frame):
                            raise RuntimeError(f"cannot save capture: {destination}")
                        print(f"saved {destination}")
                    print(f"accepted sample {len(image_points)}/{args.samples}")
                    last_auto = time.monotonic()
                elif key == 32:
                    print("sample rejected: need at least 8 clear ChArUco corners")
            if key in (27, ord("q")):
                break
    finally:
        cap.release()
        if show_preview:
            cv2.destroyAllWindows()
        if browser_preview is not None:
            browser_preview.close()
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
