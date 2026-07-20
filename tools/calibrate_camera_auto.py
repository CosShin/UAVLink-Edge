#!/usr/bin/env python3
"""Trợ lý calibration webcam tự chụp và in thông số cấu hình UAVLink."""

from __future__ import annotations

import argparse
import math
from pathlib import Path

import cv2
import numpy as np
import yaml

from calibrate_webcam_charuco import collect_from_images, collect_live, create_board


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Tự chụp ChArUco, calibration webcam và in thông số UAVLink"
    )
    parser.add_argument("--device", default="/dev/video0")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--samples", type=int, default=30)
    parser.add_argument(
        "--manual",
        action="store_true",
        help="Chỉ chụp khi nhấn Space, không tự động chụp",
    )
    parser.add_argument(
        "--capture-interval",
        type=float,
        default=0.7,
        help="Số giây giữa hai lần tự chụp",
    )
    parser.add_argument("--preview-width", type=int, default=640)
    parser.add_argument("--preview-fps", type=float, default=10.0)
    parser.add_argument("--preview-port", type=int, default=8765)
    source = parser.add_mutually_exclusive_group()
    source.add_argument(
        "--capture-only",
        metavar="DIR",
        help="Chỉ chụp và lưu ảnh vào DIR, chưa tính calibration",
    )
    source.add_argument(
        "--images",
        help="Tính calibration từ glob ảnh đã lưu, không mở camera",
    )
    parser.add_argument("--squares-x", type=int, default=7)
    parser.add_argument("--squares-y", type=int, default=5)
    parser.add_argument("--square-mm", type=float, help="Cạnh ô cờ đo thực tế, mm")
    parser.add_argument(
        "--calibration-marker-mm",
        type=float,
        help="Cạnh marker đen trên bảng ChArUco, mm",
    )
    parser.add_argument(
        "--landing-marker-mm",
        type=float,
        help="Cạnh marker ArUco trên bãi đáp, mm (chỉ để in config)",
    )
    parser.add_argument("--dictionary", default="DICT_4X4_50")
    parser.add_argument(
        "--output",
        default="Find_landing/camera_calibration_1280x720.yaml",
    )
    parser.add_argument(
        "--generate-board",
        metavar="PNG",
        help="Tạo bảng ChArUco chuẩn rồi thoát",
    )
    return parser


def ask_mm(value: float | None, label: str, example: str) -> float:
    if value is None:
        print(f"\n{label}")
        print(f"Ví dụ: {example}")
        value = float(input("Nhập số đo (mm): ").strip().replace(",", "."))
    if not math.isfinite(value) or value <= 0:
        raise SystemExit(f"Số đo không hợp lệ: {value}")
    return float(value)


def calibration_arguments(cli, square_mm: float, marker_mm: float):
    """Tạo namespace tương thích công cụ ChArUco dùng chung của dự án."""
    cli.square_length_m = square_mm / 1000.0
    cli.marker_length_m = marker_mm / 1000.0
    cli.auto = not cli.manual
    cli.capture_dir = cli.capture_only
    return cli


def calibrate(object_points, image_points, image_size):
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        None,
        None,
    )
    per_view = []
    for obj, img, rvec, tvec in zip(object_points, image_points, rvecs, tvecs):
        projected, _ = cv2.projectPoints(
            obj, rvec, tvec, camera_matrix, dist_coeffs,
        )
        errors = np.linalg.norm(projected.reshape(-1, 2) - img, axis=1)
        per_view.append(float(np.sqrt(np.mean(np.square(errors)))))
    return float(rms), camera_matrix, dist_coeffs, per_view


def calibrate_robust(
    object_points,
    image_points,
    image_size,
    max_view_rmse: float = 1.5,
    max_drop_fraction: float = 0.25,
):
    """Iteratively remove a bounded number of reprojection-error outliers."""
    kept = list(range(len(image_points)))
    rejected = []
    max_drop = int(len(kept) * max_drop_fraction)
    while True:
        rms, camera_matrix, dist_coeffs, per_view = calibrate(
            [object_points[index] for index in kept],
            [image_points[index] for index in kept],
            image_size,
        )
        worst_position = int(np.argmax(per_view))
        if per_view[worst_position] <= max_view_rmse or len(rejected) >= max_drop:
            return (
                rms,
                camera_matrix,
                dist_coeffs,
                per_view,
                kept,
                rejected,
            )
        rejected.append(kept.pop(worst_position))


def fov_degrees(camera_matrix, image_size) -> tuple[float, float]:
    width, height = image_size
    fx = float(camera_matrix[0, 0])
    fy = float(camera_matrix[1, 1])
    return (
        math.degrees(2.0 * math.atan(width / (2.0 * fx))),
        math.degrees(2.0 * math.atan(height / (2.0 * fy))),
    )


def quality_label(rms: float) -> str:
    if rms < 0.5:
        return "RẤT TỐT"
    if rms <= 1.0:
        return "ĐẠT"
    return "CHƯA ĐẠT - nên chụp lại"


def main() -> int:
    args = build_parser().parse_args()
    if not math.isfinite(args.capture_interval) or args.capture_interval <= 0:
        raise SystemExit("capture-interval phải lớn hơn 0 giây")
    if args.preview_width < 320:
        raise SystemExit("preview-width phải từ 320 pixel trở lên")
    if not math.isfinite(args.preview_fps) or args.preview_fps <= 0:
        raise SystemExit("preview-fps phải lớn hơn 0")

    if args.generate_board:
        # Tỷ lệ 40/30 mm tạo đúng bảng chuẩn; kích thước thật sẽ được đo lại khi chạy.
        args.square_length_m = 0.04
        args.marker_length_m = 0.03
        board, _ = create_board(args)
        image = board.generateImage((2100, 1500), marginSize=40, borderBits=1)
        destination = Path(args.generate_board)
        destination.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(destination), image):
            raise SystemExit(f"Không ghi được {destination}")
        print(f"Đã tạo bảng: {destination}")
        print("Hãy in phẳng, đo lại cạnh ô cờ và marker bằng thước.")
        return 0

    print("=== UAVLink - CALIBRATION CAMERA TỰ ĐỘNG ===")
    print("Dừng ./run.sh trước khi tiếp tục để giải phóng webcam.")
    square_mm = ask_mm(args.square_mm, "Cạnh một ô cờ ChArUco đo thực tế", "39.5")
    marker_mm = ask_mm(
        args.calibration_marker_mm,
        "Cạnh hình vuông đen của marker trên bảng ChArUco",
        "29.6",
    )
    if marker_mm >= square_mm:
        raise SystemExit("Marker ChArUco phải nhỏ hơn ô cờ")

    args = calibration_arguments(args, square_mm, marker_mm)
    board, detector = create_board(args)
    if args.images:
        print(f"\nĐang đọc ảnh đã lưu: {args.images}")
    else:
        print("\nCamera sắp mở. Di chuyển/nghiêng bảng qua mọi góc ảnh.")
        if args.manual:
            print("Chế độ thủ công: giữ bảng đứng yên rồi nhấn SPACE để chụp.")
        else:
            print(
                f"Chương trình tự chụp mỗi {args.capture_interval:g} giây "
                "khi thấy đủ góc ChArUco."
            )
    if args.images:
        object_points, image_points, image_size = collect_from_images(args, board)
    else:
        object_points, image_points, image_size = collect_live(args, board, detector)
    if image_size is None or len(image_points) < 10:
        raise SystemExit("Chưa đủ 10 ảnh hợp lệ; calibration bị hủy")
    if args.capture_only:
        print(f"\nĐã lưu {len(image_points)} ảnh vào: {args.capture_only}")
        print("Chưa tính calibration. Dùng --images để xử lý các ảnh này sau.")
        return 0

    input_samples = len(image_points)
    rms, camera_matrix, dist_coeffs, per_view, kept, rejected = calibrate_robust(
        object_points, image_points, image_size,
    )
    hfov, vfov = fov_degrees(camera_matrix, image_size)
    payload = {
        "version": 1,
        "device": args.device,
        "image_size": [int(image_size[0]), int(image_size[1])],
        "camera_matrix": camera_matrix.tolist(),
        "dist_coeffs": dist_coeffs.reshape(-1).tolist(),
        "rms": rms,
        "per_view_rmse_px": per_view,
        "input_samples": input_samples,
        "samples": len(kept),
        "rejected_sample_numbers": [index + 1 for index in rejected],
        "fov_deg": {"horizontal": hfov, "vertical": vfov},
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_m": square_mm / 1000.0,
            "marker_length_m": marker_mm / 1000.0,
            "dictionary": args.dictionary.upper(),
        },
    }
    output = Path(args.output)
    calibration_ok = rms <= 1.0 and max(per_view) <= 1.5
    if not calibration_ok:
        output = output.with_name(f"{output.stem}_rejected{output.suffix}")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(yaml.safe_dump(payload, sort_keys=False), encoding="utf-8")

    print("\n=== KẾT QUẢ ===")
    print(f"Ảnh đầu vào      : {input_samples}")
    print(f"Ảnh được sử dụng : {len(kept)}")
    if rejected:
        rejected_text = ", ".join(str(index + 1) for index in rejected)
        print(f"Ảnh outlier bị loại: {rejected_text}")
    print(f"RMS              : {rms:.4f} px ({quality_label(rms)})")
    print(f"View RMSE lớn nhất: {max(per_view):.4f} px")
    print(f"HFOV / VFOV      : {hfov:.2f}° / {vfov:.2f}°")
    print("Camera matrix:")
    print(np.array2string(camera_matrix, precision=6, suppress_small=True))
    print("Distortion:")
    print(np.array2string(dist_coeffs.reshape(-1), precision=8, suppress_small=True))
    print(f"Đã lưu           : {output}")

    landing_mm = args.landing_marker_mm
    if calibration_ok:
        print("\nDán vào camera stream trong config.yaml:")
        print(f"  aruco_calibration_file: {output.name}")
        if landing_mm is not None and landing_mm > 0:
            print(f"  aruco_marker_length_m: {landing_mm / 1000.0:.6f}")
        else:
            print("  aruco_marker_length_m: <cạnh marker bãi đáp đo thực tế, mét>")
        print("\nDán vào phần landing:")
        print("  camera_hfov_deg: 0")
        print("  camera_vfov_deg: 0")
    else:
        print("\nKết quả chưa đạt nên chỉ lưu file _rejected; config cũ không bị ghi đè.")
    return 0 if calibration_ok else 2


if __name__ == "__main__":
    raise SystemExit(main())
