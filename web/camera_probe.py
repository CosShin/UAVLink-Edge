"""Camera discovery — Picamera2, libcamera (rpicam-hello), V4L2 (aligned with Find_landing/camera_manager.py)."""

from __future__ import annotations

import glob
import json
import os
import re
import shutil
import subprocess
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple


def _probe_picamera2() -> Tuple[List[Dict[str, Any]], Optional[str]]:
    try:
        from picamera2 import Picamera2  # type: ignore
    except ImportError as exc:
        return [], f"picamera2 chưa cài ({exc})"

    try:
        info = Picamera2.global_camera_info()
        cameras = [
            {"id": idx, "info": str(cam), "backend": "picamera2"}
            for idx, cam in enumerate(info)
        ]
        return cameras, None
    except Exception as exc:
        return [], str(exc)


def _parse_libcamera_list(text: str) -> List[Dict[str, Any]]:
    cameras: List[Dict[str, Any]] = []
    for line in text.splitlines():
        match = re.match(r"^\s*(\d+)\s*:\s*(.+?)\s*$", line)
        if not match:
            continue
        cameras.append(
            {
                "id": int(match.group(1)),
                "info": match.group(2).strip(),
                "backend": "libcamera",
            }
        )
    return cameras


def _probe_libcamera() -> Tuple[List[Dict[str, Any]], Optional[str]]:
    for cmd in ("rpicam-hello", "libcamera-hello"):
        path = shutil.which(cmd)
        if not path:
            continue
        try:
            result = subprocess.run(
                [path, "--list-cameras"],
                capture_output=True,
                text=True,
                timeout=15,
            )
        except (subprocess.TimeoutExpired, OSError) as exc:
            return [], str(exc)

        output = (result.stdout or "") + "\n" + (result.stderr or "")
        if "No cameras available" in output:
            return [], "libcamera: No cameras available"

        cameras = _parse_libcamera_list(output)
        if cameras:
            return cameras, None
        if result.returncode != 0:
            return [], output.strip() or f"{cmd} exited {result.returncode}"
    return [], "rpicam-hello/libcamera-hello not found"


def _sorted_video_nodes() -> List[str]:
    nodes = glob.glob("/dev/video*")

    def sort_key(path: str) -> int:
        match = re.search(r"(\d+)$", path)
        return int(match.group(1)) if match else 9999

    return sorted(nodes, key=sort_key)


def _probe_v4l2_opencv() -> Tuple[List[Dict[str, Any]], Optional[str]]:
    try:
        import cv2  # type: ignore
    except ImportError:
        return [], "opencv-python (cv2) chưa cài"

    cameras: List[Dict[str, Any]] = []
    nodes = _sorted_video_nodes()
    high_nodes = [n for n in nodes if int(re.search(r"(\d+)$", n).group(1)) >= 8]
    low_nodes = [n for n in nodes if int(re.search(r"(\d+)$", n).group(1)) < 8]

    for node in high_nodes + low_nodes:
        cap = cv2.VideoCapture(node, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            continue
        ok, frame = cap.read()
        cap.release()
        if not ok or frame is None:
            continue
        cameras.append(
            {
                "id": len(cameras),
                "info": f"V4L2 {node} ({frame.shape[1]}x{frame.shape[0]})",
                "backend": "v4l2",
                "device": node,
            }
        )
    if cameras:
        return cameras, None
    return [], "Không mở được /dev/video* bằng OpenCV"


def _probe_v4l2_devices() -> Tuple[List[Dict[str, Any]], Optional[str]]:
    """Fallback when OpenCV unavailable — list capture nodes from v4l2-ctl."""
    ctl = shutil.which("v4l2-ctl")
    if not ctl:
        nodes = _sorted_video_nodes()
        if not nodes:
            return [], "Không có /dev/video*"
        return [
            {"id": i, "info": node, "backend": "v4l2", "device": node}
            for i, node in enumerate(nodes[:8])
        ], "v4l2-ctl không có — chỉ liệt kê device node"

    try:
        result = subprocess.run(
            [ctl, "--list-devices"],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (subprocess.TimeoutExpired, OSError) as exc:
        return [], str(exc)

    output = result.stdout or ""
    cameras: List[Dict[str, Any]] = []
    current_name = ""
    for line in output.splitlines():
        if not line.strip():
            continue
        if not line.startswith("\t") and not line.startswith(" "):
            current_name = line.strip().rstrip(":")
            continue
        node = line.strip()
        if node.startswith("/dev/video"):
            cameras.append(
                {
                    "id": len(cameras),
                    "info": f"{current_name} ({node})",
                    "backend": "v4l2",
                    "device": node,
                }
            )
    if cameras:
        return cameras, None
    return [], output.strip() or "v4l2-ctl: no devices"


def _v4l2_device_info(ctl: str, node: str) -> str:
    try:
        result = subprocess.run(
            [ctl, "--device", node, "--info"],
            capture_output=True,
            text=True,
            timeout=5,
        )
    except (subprocess.TimeoutExpired, OSError):
        return ""
    return (result.stdout or "") + "\n" + (result.stderr or "")


def _sysfs_video_name(node: str) -> str:
    name_path = Path("/sys/class/video4linux") / Path(node).name / "name"
    try:
        return name_path.read_text(encoding="utf-8").strip()
    except OSError:
        return Path(node).name


def _is_usb_sysfs_node(node: str) -> bool:
    device_path = Path("/sys/class/video4linux") / Path(node).name / "device"
    try:
        return "usb" in os.path.realpath(device_path).lower()
    except OSError:
        return False


def probe_usb_cameras() -> Tuple[List[Dict[str, Any]], Optional[str]]:
    """Return only USB V4L2 capture nodes, without opening an active stream.

    One webcam can expose multiple nodes (video + metadata).  ``v4l2-ctl
    --info`` lets us keep actual Video Capture nodes and ignore codec/ISP nodes.
    Sysfs is used as a fallback when v4l2-ctl is not installed.
    """
    nodes = _sorted_video_nodes()
    if not nodes:
        return [], "Không có /dev/video*"

    ctl = shutil.which("v4l2-ctl")
    cameras: List[Dict[str, Any]] = []
    for node in nodes:
        info = _v4l2_device_info(ctl, node) if ctl else ""
        info_lower = info.lower()
        sysfs_usb = _is_usb_sysfs_node(node)
        is_usb = sysfs_usb or "bus info" in info_lower and "usb" in info_lower
        if not is_usb:
            continue

        # Metadata-only nodes cannot provide image frames.
        if info and "video capture" not in info_lower:
            continue

        name = _sysfs_video_name(node)
        cameras.append(
            {
                "id": len(cameras),
                "info": f"{name} ({node})",
                "name": name,
                "backend": "v4l2",
                "source": "usb",
                "device": node,
            }
        )

    if cameras:
        return cameras, None
    if not ctl:
        return [], "Không tìm thấy webcam USB trong sysfs (v4l2-ctl chưa cài)"
    return [], "Không tìm thấy USB Video Capture device"


def probe_cameras() -> Dict[str, Any]:
    warnings: List[str] = []
    methods: List[str] = []

    for name, probe in (
        ("picamera2", _probe_picamera2),
        ("libcamera", _probe_libcamera),
        ("v4l2_opencv", _probe_v4l2_opencv),
    ):
        methods.append(name)
        cameras, err = probe()
        if cameras:
            return {"cameras": cameras, "warnings": warnings, "methods_tried": methods}
        if err:
            warnings.append(f"{name}: {err}")

    # Diagnostic only — ISP/codec nodes are not capture cameras.
    _, diag_err = _probe_v4l2_devices()
    if diag_err:
        warnings.append(f"v4l2_devices: {diag_err}")

    return {"cameras": [], "warnings": warnings, "methods_tried": methods}


def probe_cameras_json() -> str:
    return json.dumps(probe_cameras())
