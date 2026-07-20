"""ArUco landing detection with duplicate safety and optional board fusion."""

from __future__ import annotations

import numpy as np

from .marker import BOARD_MARKER_COUNT
from .board import (
    duplicate_ids,
    estimate_board,
    estimate_single_marker_pose,
    single_marker_quality,
)

BOARD_ID_MAX = BOARD_MARKER_COUNT - 1


def get_direction(offset_x: int, offset_y: int, threshold: int = 20) -> str:
    direction = ""
    if abs(offset_x) > threshold:
        direction += "RIGHT " if offset_x > 0 else "LEFT "
    if abs(offset_y) > threshold:
        direction += "DOWN " if offset_y > 0 else "UP "
    return direction.strip() or "CENTER"


def _board_markers(corners, ids):
    out = []
    for i, mid in enumerate(ids.flatten()):
        mid = int(mid)
        if mid < 0 or mid > BOARD_ID_MAX:
            continue
        pts = corners[i].reshape(4, 2)
        cx = float(pts[:, 0].mean())
        cy = float(pts[:, 1].mean())
        out.append({"id": mid, "center": (cx, cy), "corners": pts})
    return out


def _pick_landing(board_markers: list, marker_id: int):
    """Pick one unique target marker; duplicate target IDs are rejected earlier."""
    if not board_markers:
        return None

    target = int(marker_id)
    if target < 0 or target > BOARD_ID_MAX:
        return None

    chosen = next((m for m in board_markers if m["id"] == target), None)
    if chosen is None:
        return None

    return {
        "id": chosen["id"],
        "center": chosen["center"],
        "corners": chosen["corners"],
        "board_mode": len(board_markers) > 1,
        "marker_count": len(board_markers),
        "visible_ids": [m["id"] for m in board_markers],
    }


def detect_frame(
    frame_bgr,
    output_size: tuple[int, int],
    detector,
    *,
    marker_id: int = 0,
    detect_size: tuple[int, int] | None = None,
    target_strategy: str = "single",
    board_first_id: int = 0,
    board_cols: int = 3,
    board_rows: int = 4,
    board_gap_x_ratio: float = 0.16,
    board_gap_y_ratio: float = 0.34,
    board_ransac_threshold_px: float = 3.0,
    board_min_markers: int = 2,
    board_close_single_marker_area_ratio: float = 0.08,
    calibration: dict | None = None,
    marker_length_m: float = 0.0,
) -> dict:
    import cv2

    det_w, det_h = detect_size or (frame_bgr.shape[1], frame_bgr.shape[0])
    if (frame_bgr.shape[1], frame_bgr.shape[0]) != (det_w, det_h):
        small = cv2.resize(frame_bgr, (det_w, det_h), interpolation=cv2.INTER_AREA)
    else:
        small = frame_bgr

    gray = cv2.cvtColor(small, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None or len(ids) == 0:
        return {"detected": False, "reason": "no markers"}

    board_markers = _board_markers(corners, ids)
    visible_ids = [int(marker["id"]) for marker in board_markers]
    strategy = str(target_strategy or "single").strip().lower()
    if strategy not in ("single", "board"):
        strategy = "single"

    duplicates = duplicate_ids(board_markers)
    ambiguous_duplicates = duplicates if strategy == "board" else [mid for mid in duplicates if mid == int(marker_id)]
    if ambiguous_duplicates:
        return {
            "detected": False,
            "ambiguous": True,
            "reason": f"duplicate marker IDs: {ambiguous_duplicates}",
            "duplicate_ids": ambiguous_duplicates,
            "aruco_visible_ids": visible_ids,
            "aruco_marker_count": len(board_markers),
            "searching_id": int(marker_id),
            "quality": 0.0,
        }

    board_result = None
    landing = None
    if strategy == "board":
        supported = [
            marker
            for marker in board_markers
            if board_first_id <= int(marker["id"]) < board_first_id + board_cols * board_rows
        ]
        required_markers = max(2, int(board_min_markers))
        close_single_fallback = False
        if len(supported) == 1 and float(board_close_single_marker_area_ratio) > 0:
            marker_area = abs(float(cv2.contourArea(supported[0]["corners"])))
            close_single_fallback = (
                marker_area / max(float(det_w * det_h), 1.0)
                >= float(board_close_single_marker_area_ratio)
            )
        if len(supported) < required_markers and not close_single_fallback:
            return {
                "detected": False,
                "reason": (
                    f"board needs at least {required_markers} unique markers "
                    "or one large close marker"
                ),
                "aruco_visible_ids": visible_ids,
                "aruco_marker_count": len(supported),
                "quality": 0.0,
            }
        board_result = estimate_board(
            supported,
            (det_w, det_h),
            first_id=board_first_id,
            cols=board_cols,
            rows=board_rows,
            gap_x_ratio=board_gap_x_ratio,
            gap_y_ratio=board_gap_y_ratio,
            ransac_threshold_px=board_ransac_threshold_px,
            calibration=calibration,
            output_size=output_size,
            marker_length_m=marker_length_m,
        )
        if board_result is None:
            return {
                "detected": False,
                "reason": "board pose unavailable",
                "aruco_visible_ids": visible_ids,
                "aruco_marker_count": len(board_markers),
                "quality": 0.0,
            }
        primary = next((m for m in supported if int(m["id"]) == int(marker_id)), supported[0])
        all_pts = np.concatenate([m["corners"].reshape(4, 2) for m in supported], axis=0)
        landing = {
            "id": int(primary["id"]),
            "center": board_result["center"],
            "corners": primary["corners"],
            "board_mode": True,
            "marker_count": len(supported),
            "visible_ids": [int(m["id"]) for m in supported],
            "extent": all_pts,
            "close_single_marker_fallback": close_single_fallback,
        }
    else:
        landing = _pick_landing(board_markers, marker_id)
        if landing is None:
            return {
                "detected": False,
                "reason": f"target ID {int(marker_id)} not visible",
                "searching_id": int(marker_id),
                "aruco_visible_ids": visible_ids,
                "aruco_marker_count": len(board_markers),
                "quality": 0.0,
            }

    cx, cy = landing["center"]
    pts = landing["corners"]
    extent = np.asarray(landing.get("extent", pts), dtype=np.float32).reshape(-1, 2)
    w = float(extent[:, 0].max() - extent[:, 0].min())
    h = float(extent[:, 1].max() - extent[:, 1].min())

    w_out, h_out = output_size
    sx = w_out / det_w
    sy = h_out / det_h
    h_x = int(round(cx * sx))
    h_y = int(round(cy * sy))
    box_w = max(8, int(round(w * sx)))
    box_h = max(8, int(round(h * sy)))
    center_x, center_y = w_out // 2, h_out // 2

    all_marker_corners = []
    for m in board_markers if board_markers else []:
        scaled = [
            (int(round(p[0] * sx)), int(round(p[1] * sy)))
            for p in m["corners"].reshape(4, 2)
        ]
        all_marker_corners.append(scaled)

    primary_corners = [
        (int(round(p[0] * sx)), int(round(p[1] * sy))) for p in pts.reshape(4, 2)
    ]

    markers_by_id: dict[int, list[tuple[int, int]]] = {}
    marker_instances: list[dict] = []
    for m in board_markers if board_markers else []:
        scaled = [
            (int(round(p[0] * sx)), int(round(p[1] * sy)))
            for p in m["corners"].reshape(4, 2)
        ]
        marker_instances.append({"id": int(m["id"]), "corners": scaled})
        markers_by_id.setdefault(int(m["id"]), scaled)

    single_pose = None
    if board_result is not None:
        quality = float(board_result["quality"])
        quality_details = {
            "reprojection_error_px": board_result["reprojection_error_px"],
            "inlier_ratio": board_result["inlier_ratio"],
            "used_ids": board_result["used_ids"],
        }
    else:
        selected_marker = next(m for m in board_markers if int(m["id"]) == int(marker_id))
        quality, quality_details = single_marker_quality(selected_marker, (det_w, det_h))
        single_pose = estimate_single_marker_pose(
            selected_marker,
            (det_w, det_h),
            calibration=calibration,
            output_size=output_size,
            marker_length_m=marker_length_m,
        )

    result = {
        "detected": True,
        "detector": "aruco",
        "mode": "board" if strategy == "board" else "aruco",
        "version": "v3",
        "has_marker": True,
        "ambiguous": False,
        "reason": "target detected",
        "target_key": (
            f"board:{board_first_id}-{board_first_id + board_cols * board_rows - 1}"
            if strategy == "board"
            else f"marker:{int(marker_id)}"
        ),
        "h_position": (h_x, h_y),
        "h_size": (box_w, box_h),
        "offset_x": h_x - center_x,
        "offset_y": center_y - h_y,
        "similarity": quality,
        "quality": quality,
        "quality_details": quality_details,
        "direction": get_direction(h_x - center_x, center_y - h_y),
        "aruco_id": landing["id"],
        "aruco_corners": primary_corners,
        "aruco_markers": all_marker_corners,
        "aruco_markers_by_id": markers_by_id,
        "aruco_instances": marker_instances,
        "aruco_visible_ids": landing.get("visible_ids", []),
        "aruco_marker_count": landing.get("marker_count", 1),
        "duplicate_ids": [],
        "close_single_marker_fallback": bool(
            landing.get("close_single_marker_fallback", False)
        ),
        "pose_valid": False,
        "in_circle": False,
    }
    if board_result is not None:
        result.update(
            {
                "reprojection_error_px": board_result["reprojection_error_px"],
                "board_inlier_ratio": board_result["inlier_ratio"],
                "board_used_ids": board_result["used_ids"],
                "pose_valid": bool(board_result.get("pose_valid")),
            }
        )
        for key in (
            "pose_camera_m",
            "rvec",
            "pnp_reprojection_error_px",
            "pnp_inliers",
            "target_center_camera_m",
            "camera_to_target_distance_m",
            "camera_to_target_depth_m",
        ):
            if key in board_result:
                result[key] = board_result[key]
    elif single_pose is not None:
        result.update(single_pose)
    return result


def detect_frame_multiscale(
    frame_bgr,
    output_size: tuple[int, int],
    detector,
    *,
    detect_sizes: list[tuple[int, int]],
    **kwargs,
) -> dict:
    """Try the cheap scale first and higher resolution only while target is lost."""
    attempts = []
    seen_sizes = set()
    for size in detect_sizes:
        normalized = (max(1, int(size[0])), max(1, int(size[1])))
        if normalized in seen_sizes:
            continue
        seen_sizes.add(normalized)
        result = detect_frame(
            frame_bgr,
            output_size,
            detector,
            detect_size=normalized,
            **kwargs,
        )
        result["detection_size"] = [normalized[0], normalized[1]]
        attempts.append(result)
        if result.get("detected"):
            result["multiscale_attempts"] = len(attempts)
            return result

    if not attempts:
        return {"detected": False, "reason": "no detection scales configured"}
    best = max(
        attempts,
        key=lambda item: (
            int(item.get("aruco_marker_count", 0) or 0),
            len(item.get("aruco_visible_ids") or []),
            float(item.get("quality", 0.0) or 0.0),
        ),
    )
    best["multiscale_attempts"] = len(attempts)
    return best
