"""v3: ArUco single target hoặc board fusion an toàn."""

MODE_ID = "aruco"


def needs_template() -> bool:
    return False


def prepare(find_landing_dir: str):
    return None


def create_processor(config: dict, find_landing_dir: str, prepared=None):
    from processing.detect_config import detect_size_from_config, frame_skip, lost_hold_ms, reacquire_ms

    from .processor import ArucoProcessor

    return ArucoProcessor(
        find_landing_dir,
        enabled=True,
        frame_skip=frame_skip(config),
        marker_id=int(config.get("aruco_marker_id", 0) or 0),
        dictionary=str(config.get("aruco_dictionary", "DICT_4X4_50")),
        detect_size=detect_size_from_config(config),
        lost_hold_ms=lost_hold_ms(config),
        reacquire_ms=reacquire_ms(config),
        camera_id=int(config.get("camera_id", 0) or 0),
        target_strategy=str(config.get("aruco_target_strategy", "single") or "single"),
        board_first_id=int(config.get("aruco_board_first_id", 0) or 0),
        board_cols=int(config.get("aruco_board_cols", 3) or 3),
        board_rows=int(config.get("aruco_board_rows", 4) or 4),
        board_gap_x_ratio=float(config.get("aruco_board_gap_x_ratio", 0.16) or 0.16),
        board_gap_y_ratio=float(config.get("aruco_board_gap_y_ratio", 0.34) or 0.34),
        board_ransac_threshold_px=float(config.get("aruco_board_ransac_threshold_px", 3.0) or 3.0),
        board_min_markers=int(config.get("aruco_board_min_markers", 2) or 2),
        board_close_single_marker_area_ratio=float(
            config.get("aruco_board_close_single_marker_area_ratio", 0.08) or 0.0
        ),
        reacquire_detect_width=int(config.get("aruco_reacquire_detect_width", 960) or 0),
        marker_length_m=float(config.get("aruco_marker_length_m", 0.0) or 0.0),
        calibration_file=str(config.get("aruco_calibration_file", "") or ""),
        min_quality=float(config.get("aruco_min_quality", 0.55) or 0.55),
        acquire_frames=int(config.get("aruco_acquire_frames", 5) or 5),
    )


def draw_overlay(frame, detection_result: dict):
    from .overlay import draw

    return draw(frame, detection_result)
