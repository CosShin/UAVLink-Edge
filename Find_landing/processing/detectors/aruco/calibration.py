"""Load and scale camera intrinsics for ArUco pose estimation."""

from __future__ import annotations

from pathlib import Path
from typing import Optional

import numpy as np
import yaml


def resolve_calibration_path(find_landing_dir: str, value: str | None) -> Optional[Path]:
    raw = str(value or "").strip()
    if not raw:
        return None
    path = Path(raw).expanduser()
    if not path.is_absolute():
        path = Path(find_landing_dir) / path
    return path.resolve()


def load_calibration(find_landing_dir: str, value: str | None) -> Optional[dict]:
    path = resolve_calibration_path(find_landing_dir, value)
    if path is None:
        return None
    if not path.is_file():
        raise FileNotFoundError(f"camera calibration not found: {path}")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    matrix = np.asarray(data.get("camera_matrix"), dtype=np.float64)
    dist = np.asarray(data.get("dist_coeffs", []), dtype=np.float64).reshape(-1, 1)
    size = data.get("image_size") or []
    if matrix.shape != (3, 3):
        raise ValueError(f"invalid camera_matrix in {path}")
    if len(size) < 2 or int(size[0]) <= 0 or int(size[1]) <= 0:
        raise ValueError(f"invalid image_size in {path}")
    return {
        "path": str(path),
        "camera_matrix": matrix,
        "dist_coeffs": dist,
        "image_size": (int(size[0]), int(size[1])),
        "rms": float(data.get("rms", 0.0) or 0.0),
    }


def matrix_for_size(calibration: dict, output_size: tuple[int, int]) -> np.ndarray:
    """Scale K when output keeps the same optical crop/aspect ratio."""
    matrix = np.array(calibration["camera_matrix"], dtype=np.float64, copy=True)
    src_w, src_h = calibration["image_size"]
    dst_w, dst_h = int(output_size[0]), int(output_size[1])
    sx, sy = dst_w / src_w, dst_h / src_h
    matrix[0, 0] *= sx
    matrix[0, 2] *= sx
    matrix[1, 1] *= sy
    matrix[1, 2] *= sy
    return matrix

