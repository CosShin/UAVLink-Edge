"""Overlay ArUco v3 — vẽ toàn bộ marker trên bảng + tâm đáp."""

from processing.overlay_style import (
    FONT_SCALE_LABEL,
    MARKER_BORDER,
    MARKER_BORDER_HI,
    TARGET_DOT_R,
    put_text_line,
)


def draw(frame, detection_result: dict):
    import cv2
    import numpy as np

    instances = detection_result.get("aruco_instances") or []
    by_id = detection_result.get("aruco_markers_by_id")
    if instances:
        for item in instances:
            corners = item.get("corners") or []
            if len(corners) >= 4:
                pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 200, 0), MARKER_BORDER)
                x, y = corners[0]
                cv2.putText(frame, str(item.get("id")), (x, y - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 0), 1, cv2.LINE_AA)
    elif by_id:
        for corners in by_id.values():
            if len(corners) >= 4:
                pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 200, 0), MARKER_BORDER)
    else:
        for marker_corners in detection_result.get("aruco_markers") or []:
            if len(marker_corners) >= 4:
                pts = np.array(marker_corners, dtype=np.int32).reshape((-1, 1, 2))
                cv2.polylines(frame, [pts], True, (0, 200, 0), MARKER_BORDER)

    corners = detection_result.get("aruco_corners")
    if corners:
        pts = np.array(corners, dtype=np.int32).reshape((-1, 1, 2))
        cv2.polylines(frame, [pts], True, (0, 255, 0), MARKER_BORDER_HI)

    aid = detection_result.get("aruco_id", 0)
    n = detection_result.get("aruco_marker_count", 1)
    ids = detection_result.get("aruco_visible_ids", [])
    if detection_result.get("mode") == "board":
        label = f"ARUCO BOARD ({n} visible: {ids})"
        if detection_result.get("close_single_marker_fallback"):
            label += " [CLOSE-1]"
    else:
        label = f"ARUCO v3 ID={aid}"
    distance_m = detection_result.get("camera_to_target_distance_m")
    if distance_m is not None:
        label += f" | D={float(distance_m):.2f}m"
    if detection_result.get("hold"):
        label += " [HOLD]"

    color = (0, 180, 0) if detection_result.get("hold") else (0, 255, 0)
    put_text_line(frame, 0, label, color, FONT_SCALE_LABEL)
    h_x, h_y = detection_result["h_position"]
    cv2.circle(frame, (h_x, h_y), TARGET_DOT_R, (0, 0, 255), -1, lineType=cv2.LINE_AA)
    return frame
