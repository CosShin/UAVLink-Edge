"""Đọc tham số CV chung từ camera_config — không chứa thuật toán detection."""


def detect_size_from_config(config: dict | None) -> tuple[int, int]:
    max_w, max_h = 320, 240
    if not config:
        return max_w, max_h
    lores = config.get("lores_size")
    if isinstance(lores, (list, tuple)) and len(lores) >= 2:
        w, h = int(lores[0]), int(lores[1])
        if w > 0 and h > 0:
            return w, h
    size = config.get("size")
    if isinstance(size, (list, tuple)) and len(size) >= 2:
        w, h = int(size[0]), int(size[1])
        if w > 0 and h > 0 and w * h <= max_w * max_h:
            return w, h
    return max_w, max_h


def frame_skip(config: dict, key: str = "detect_frame_skip", default: int = 3) -> int:
    try:
        return max(int(config.get(key, default)), 1)
    except (TypeError, ValueError):
        return default


def lost_hold_ms(config: dict | None, default: int = 1500) -> int:
    """Thời gian giữ pose sau lần nhìn thấy marker cuối (ms), độc lập detect_frame_skip."""
    if not config:
        return default
    try:
        v = config.get("detection_lost_hold_ms")
        if v is not None:
            return max(int(v), 0)
    except (TypeError, ValueError):
        pass
    return default


def reacquire_ms(config: dict | None, default: int = 2500) -> int:
    """Sau khi mất target, detect mỗi frame trong cửa sổ này để bắt lại nhanh."""
    if not config:
        return default
    try:
        v = config.get("detection_reacquire_ms")
        if v is not None:
            return max(int(v), 0)
    except (TypeError, ValueError):
        pass
    return default
