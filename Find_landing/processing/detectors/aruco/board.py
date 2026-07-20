"""Geometry, ambiguity checks and pose/quality helpers for ArUco landing pads."""

from __future__ import annotations

import math
from collections import Counter
from typing import Iterable

import cv2
import numpy as np


def duplicate_ids(markers: Iterable[dict]) -> list[int]:
    counts = Counter(int(marker["id"]) for marker in markers)
    return sorted(mid for mid, count in counts.items() if count > 1)


def marker_model_corners(
    marker_id: int,
    *,
    first_id: int,
    cols: int,
    rows: int,
    gap_x_ratio: float,
    gap_y_ratio: float,
    marker_length: float = 1.0,
) -> np.ndarray | None:
    index = int(marker_id) - int(first_id)
    if index < 0 or index >= int(cols) * int(rows):
        return None
    row, col = divmod(index, int(cols))
    step_x = marker_length * (1.0 + float(gap_x_ratio))
    step_y = marker_length * (1.0 + float(gap_y_ratio))
    x0, y0 = col * step_x, row * step_y
    return np.asarray(
        [
            [x0, y0],
            [x0 + marker_length, y0],
            [x0 + marker_length, y0 + marker_length],
            [x0, y0 + marker_length],
        ],
        dtype=np.float32,
    )


def board_center_model(
    *, cols: int, rows: int, gap_x_ratio: float, gap_y_ratio: float, marker_length: float = 1.0
) -> np.ndarray:
    width = marker_length * (cols + max(cols - 1, 0) * gap_x_ratio)
    height = marker_length * (rows + max(rows - 1, 0) * gap_y_ratio)
    return np.asarray([[[width / 2.0, height / 2.0]]], dtype=np.float32)


def _polygon_area(corners: np.ndarray) -> float:
    return abs(float(cv2.contourArea(np.asarray(corners, dtype=np.float32))))


def _edge_score(corners: np.ndarray, width: int, height: int) -> float:
    pts = np.asarray(corners, dtype=np.float32).reshape(-1, 2)
    margin = min(
        float(pts[:, 0].min()),
        float(pts[:, 1].min()),
        float(width - 1 - pts[:, 0].max()),
        float(height - 1 - pts[:, 1].max()),
    )
    return max(0.0, min(1.0, margin / max(min(width, height) * 0.08, 1.0)))


def single_marker_quality(marker: dict, image_size: tuple[int, int]) -> tuple[float, dict]:
    width, height = image_size
    corners = np.asarray(marker["corners"], dtype=np.float32).reshape(4, 2)
    area_fraction = _polygon_area(corners) / max(width * height, 1)
    area_score = max(0.0, min(1.0, area_fraction / 0.015))
    edge_score = _edge_score(corners, width, height)
    lengths = [
        float(np.linalg.norm(corners[(idx + 1) % 4] - corners[idx]))
        for idx in range(4)
    ]
    shape_score = min(lengths) / max(max(lengths), 1e-6)
    quality = 0.45 * area_score + 0.30 * edge_score + 0.25 * shape_score
    return float(max(0.0, min(1.0, quality))), {
        "area_fraction": area_fraction,
        "area_score": area_score,
        "edge_score": edge_score,
        "shape_score": shape_score,
    }


def estimate_single_marker_pose(
    marker: dict,
    image_size: tuple[int, int],
    *,
    calibration: dict | None,
    output_size: tuple[int, int] | None,
    marker_length_m: float,
) -> dict | None:
    """Estimate metric camera-to-marker pose from one calibrated square marker."""
    if not calibration or not output_size or marker_length_m <= 0:
        return None

    width, height = image_size
    out_w, out_h = output_size
    if width <= 0 or height <= 0 or out_w <= 0 or out_h <= 0:
        return None

    half = float(marker_length_m) / 2.0
    object_points = np.asarray(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float32,
    )
    scale = np.asarray([out_w / width, out_h / height], dtype=np.float32)
    image_points = (
        np.asarray(marker["corners"], dtype=np.float32).reshape(4, 2) * scale
    )
    camera_matrix = calibration["camera_matrix"]
    dist_coeffs = calibration["dist_coeffs"]
    ok, rvec, tvec = cv2.solvePnP(
        object_points,
        image_points,
        camera_matrix,
        dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE_SQUARE,
    )
    if not ok or not np.all(np.isfinite(tvec)) or float(tvec[2, 0]) <= 0:
        return None

    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs,
    )
    errors = np.linalg.norm(projected.reshape(-1, 2) - image_points, axis=1)
    pose = [float(value) for value in tvec.reshape(3)]
    return {
        "pose_valid": True,
        "pose_camera_m": pose,
        "target_center_camera_m": pose,
        "rvec": [float(value) for value in rvec.reshape(3)],
        "pnp_reprojection_error_px": float(
            math.sqrt(float(np.mean(np.square(errors))))
        ),
        "pnp_inliers": 4,
        "camera_to_target_distance_m": float(np.linalg.norm(tvec)),
        "camera_to_target_depth_m": float(tvec[2, 0]),
    }


def estimate_board(
    markers: list[dict],
    image_size: tuple[int, int],
    *,
    first_id: int,
    cols: int,
    rows: int,
    gap_x_ratio: float,
    gap_y_ratio: float,
    ransac_threshold_px: float,
    calibration: dict | None = None,
    output_size: tuple[int, int] | None = None,
    marker_length_m: float = 0.0,
) -> dict | None:
    model_points: list[list[float]] = []
    image_points: list[list[float]] = []
    used_ids: list[int] = []
    for marker in markers:
        model = marker_model_corners(
            int(marker["id"]),
            first_id=first_id,
            cols=cols,
            rows=rows,
            gap_x_ratio=gap_x_ratio,
            gap_y_ratio=gap_y_ratio,
        )
        if model is None:
            continue
        model_points.extend(model.tolist())
        image_points.extend(np.asarray(marker["corners"], dtype=np.float32).reshape(4, 2).tolist())
        used_ids.append(int(marker["id"]))
    if len(model_points) < 4:
        return None

    obj2 = np.asarray(model_points, dtype=np.float32)
    img2 = np.asarray(image_points, dtype=np.float32)
    homography, mask = cv2.findHomography(
        obj2,
        img2,
        cv2.RANSAC if len(obj2) > 4 else 0,
        float(ransac_threshold_px),
    )
    if homography is None:
        return None
    projected = cv2.perspectiveTransform(obj2.reshape(-1, 1, 2), homography).reshape(-1, 2)
    errors = np.linalg.norm(projected - img2, axis=1)
    inliers = np.ones(len(errors), dtype=bool) if mask is None else mask.reshape(-1).astype(bool)
    if not np.any(inliers):
        return None
    reprojection_error = float(math.sqrt(float(np.mean(np.square(errors[inliers])))))
    center = cv2.perspectiveTransform(
        board_center_model(
            cols=cols,
            rows=rows,
            gap_x_ratio=gap_x_ratio,
            gap_y_ratio=gap_y_ratio,
        ),
        homography,
    ).reshape(2)

    width, height = image_size
    total_area = sum(_polygon_area(marker["corners"]) for marker in markers)
    area_score = min(1.0, total_area / max(width * height * 0.03, 1.0))
    count_score = min(1.0, len(set(used_ids)) / 3.0)
    inlier_ratio = float(np.mean(inliers))
    reprojection_score = max(0.0, 1.0 - reprojection_error / max(ransac_threshold_px * 2.0, 1.0))
    edge_score = max(
        (_edge_score(marker["corners"], width, height) for marker in markers),
        default=0.0,
    )
    quality = (
        0.35 * reprojection_score
        + 0.25 * count_score
        + 0.20 * inlier_ratio
        + 0.10 * area_score
        + 0.10 * edge_score
    )
    result = {
        "center": (float(center[0]), float(center[1])),
        "quality": float(max(0.0, min(1.0, quality))),
        "reprojection_error_px": reprojection_error,
        "inlier_ratio": inlier_ratio,
        "used_ids": sorted(set(used_ids)),
        "homography": homography.tolist(),
        "pose_valid": False,
    }

    if calibration and marker_length_m > 0 and output_size:
        scale_x = output_size[0] / image_size[0]
        scale_y = output_size[1] / image_size[1]
        img_full = img2 * np.asarray([scale_x, scale_y], dtype=np.float32)
        obj3 = np.column_stack(
            [obj2 * float(marker_length_m), np.zeros(len(obj2), dtype=np.float32)]
        ).astype(np.float32)
        camera_matrix = calibration["camera_matrix"]
        dist_coeffs = calibration["dist_coeffs"]
        ok, rvec, tvec, pnp_inliers = cv2.solvePnPRansac(
            obj3,
            img_full,
            camera_matrix,
            dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE,
            reprojectionError=max(float(ransac_threshold_px * scale_x), 2.0),
            iterationsCount=100,
            confidence=0.99,
        )
        if ok:
            rep, _ = cv2.projectPoints(obj3, rvec, tvec, camera_matrix, dist_coeffs)
            pnp_err = np.linalg.norm(rep.reshape(-1, 2) - img_full, axis=1)
            center_xy = board_center_model(
                cols=cols,
                rows=rows,
                gap_x_ratio=gap_x_ratio,
                gap_y_ratio=gap_y_ratio,
                marker_length=float(marker_length_m),
            ).reshape(2)
            center_board = np.asarray(
                [[float(center_xy[0])], [float(center_xy[1])], [0.0]],
                dtype=np.float64,
            )
            rotation, _ = cv2.Rodrigues(rvec)
            center_camera = rotation @ center_board + tvec
            result.update(
                {
                    "pose_valid": True,
                    "pose_camera_m": [float(v) for v in tvec.reshape(3)],
                    "rvec": [float(v) for v in rvec.reshape(3)],
                    "pnp_reprojection_error_px": float(
                        math.sqrt(float(np.mean(np.square(pnp_err))))
                    ),
                    "pnp_inliers": int(len(pnp_inliers)) if pnp_inliers is not None else 0,
                    "target_center_camera_m": [
                        float(value) for value in center_camera.reshape(3)
                    ],
                    "camera_to_target_distance_m": float(np.linalg.norm(center_camera)),
                    "camera_to_target_depth_m": float(center_camera.reshape(3)[2]),
                }
            )
    return result
