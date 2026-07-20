# Danh mục file UAVLink-Edge-Python

Tài liệu này mô tả chức năng các file còn lại sau khi dọn thư mục dự án.

Các file đã dọn vì là dữ liệu sinh tự động, không cần lưu trong source:

- `__pycache__/` và các file `*.pyc`: cache bytecode Python.
- `data/logs/*.log`, `data/logs/*.log.*`: log runtime, tự sinh lại khi chạy.
- `data/uavlink-edge.lock`: lock chống chạy trùng tiến trình, tự sinh lại khi chạy.
- `_bootstrap/build/`: thư mục build tạm nếu rỗng.

Các thư mục không liệt kê từng file:

- `.git/`: dữ liệu nội bộ của Git.
- `venv/`: môi trường Python cục bộ, có thể tạo lại bằng `python3 install.py`.
- `.agents/`, `.codex/`: dữ liệu nội bộ của công cụ làm việc.
- `how_do_drones_work/.venv/`: môi trường Python riêng của project tham khảo.

## File gốc

| File | Chức năng |
|---|---|
| `.drone_secret` | Secret đăng ký drone với server; file nhạy cảm, đã được `.gitignore` bỏ qua. |
| `.gitattributes` | Quy tắc thuộc tính file cho Git. |
| `.gitignore` | Quy tắc bỏ qua cache, log, secret, venv và dữ liệu runtime. |
| `AUTHENTICATION_PROTOCOL.md` | Tài liệu giao thức xác thực giữa edge và cloud. |
| `BAO_CAO_LUONG_HA_CANH_BANG_CAMERA.md` | Báo cáo luồng xử lý hạ cánh bằng camera. |
| `HUONG_DAN_CAP_NHAT_WEBCAM_GPS_LANDING.md` | Hướng dẫn cập nhật tính năng webcam/GPS cho landing. |
| `HUONG_DAN_SU_DUNG_BAN_HA_CANH_ARUCO_HOAN_THIEN.md` | Hướng dẫn sử dụng bàn hạ cánh ArUco hoàn thiện. |
| `KE_HOACH_THUAT_TOAN_HA_CANH_ARUCO.md` | Kế hoạch thuật toán hạ cánh dùng ArUco. |
| `README.md` | Tài liệu chính: giới thiệu, cài đặt, chạy, cấu hình và troubleshooting. |
| `STARTUP_FLOW.md` | Mô tả luồng khởi động hệ thống. |
| `apply_camera_overlay.sh` | Áp cấu hình overlay camera vào host và lên lịch reboot khi cần. |
| `apply_host_reboot.sh` | Script reboot host sau khi đổi overlay camera. |
| `auth_apikey.py` | Đóng gói/giải mã message API key dùng custom MAVLink. |
| `auth_client.py` | Client xác thực drone với cloud server. |
| `camera_mavlink.py` | Gửi trạng thái camera/stream qua MAVLink custom message. |
| `cloud_egress.py` | Quản lý luồng gửi dữ liệu ra cloud, tránh treo khi thiếu modem/network. |
| `config.py` | Lớp đọc, ghi và truy cập cấu hình `config.yaml`. |
| `config.yaml` | Cấu hình runtime: auth, network, ethernet, VPN, camera, MAVLink. |
| `ethernet_setup.py` | Thiết lập IP tĩnh ethernet trước khi bind MAVLink. |
| `forwarder.py` | Cầu nối MAVLink giữa flight controller, web UI và cloud. |
| `index.html` | Trang HTML độc lập/landing UI cũ hoặc bản tĩnh dùng tham khảo. |
| `install.py` | Cài apt dependency, tạo venv và cài pip packages. |
| `install_camera_sudoers.sh` | Cài sudoers cho các lệnh camera/reboot không cần nhập password. |
| `instance_lock.py` | Chặn chạy nhiều instance `main.py` cùng lúc. |
| `landing_mavlink.py` | Chuyển telemetry landing từ camera thành MAVLink `LANDING_TARGET`. |
| `logging_setup.py` | Cấu hình logging sạch cho console và file log xoay vòng. |
| `main.py` | Entry point chính: load config, khởi động auth, MAVLink, web server, camera, VPN. |
| `mavlink_custom.py` | Định nghĩa custom message/id dùng cho UAVLink. |
| `mavlink_utils.py` | Hàm tiện ích nhận dạng heartbeat Pixhawk và chuẩn hóa kiểu kết nối MAVLink. |
| `metrics.py` | Bộ đếm metric đơn giản cho trạng thái hệ thống. |
| `network_controller.py` | Điều khiển network monitor và ưu tiên WiFi/4G/ethernet. |
| `network_utils.py` | Tiện ích kiểm tra network/IP/route dùng chung. |
| `partner_heartbeat.py` | Phát heartbeat MAVLink cho partner/system liên quan. |
| `paths.py` | Chuẩn hóa đường dẫn project, `Module_4G`, `Find_landing`, file runtime. |
| `requirements.txt` | Danh sách Python package cần cài vào venv. |
| `run.sh` | Lệnh chạy khuyến nghị, dùng `venv/bin/python main.py`. |
| `setup_camera.sh` | Ghi overlay CSI camera vào `/boot/firmware/config.txt`. |
| `telemetry.py` | Cache telemetry MAVLink: mode, GPS, pin, armed state, link status. |
| `video_streamer.py` | Lớp điều phối video streamer ở mức root/legacy. |
| `vpn_manager.py` | Quản lý WireGuard/VPN cho drone edge. |
| `web_server.py` | Wrapper/compat entry cho web server. |

## Auto_landing

| File | Chức năng |
|---|---|
| `Auto_landing/VISION_LANDING_ANALYSIS.md` | Phân tích cũ về vision landing. |
| `Auto_landing/camera_config.json` | Cấu hình camera cho module Auto_landing cũ. |
| `Auto_landing/camera_manager.py` | Quản lý camera Picamera/OpenCV cho pipeline cũ. |
| `Auto_landing/camera_manager.py.broken` | Bản backup bị đánh dấu lỗi; chỉ nên giữ nếu cần so sánh lịch sử. |
| `Auto_landing/camera_streamer.py` | Stream camera và xử lý frame theo pipeline cũ. |
| `Auto_landing/find.py` | Detector ArUco/landing target phiên bản cũ. |
| `Auto_landing/landing_config.json` | Cấu hình nhận diện landing cho module cũ. |
| `Auto_landing/list_temp.json` | Danh sách template/phụ trợ cho detector cũ. |
| `Auto_landing/templates/A.png` | Template chữ A cho nhận diện cũ. |
| `Auto_landing/templates/H.png` | Template chữ H cho nhận diện cũ. |
| `Auto_landing/templates/K.png` | Template chữ K cho nhận diện cũ. |
| `Auto_landing/templates/Y.png` | Template chữ Y cho nhận diện cũ. |
| `Auto_landing/test_find.py` | Script test detector Auto_landing cũ. |

## Find_landing

| File | Chức năng |
|---|---|
| `Find_landing/bench_stream.py` | Benchmark capture, publisher và RTSP stream. |
| `Find_landing/camera_calibration_1280x720.yaml` | Ma trận camera/distortion cho độ phân giải 1280x720. |
| `Find_landing/camera_config_0.json` | Cấu hình stream/camera CAM0. |
| `Find_landing/camera_config_1.json` | Cấu hình stream/camera CAM1. |
| `Find_landing/camera_detected.json` | Kết quả detect camera đang có trên thiết bị. |
| `Find_landing/camera_manager.py` | Quản lý camera cho pipeline landing hiện tại. |
| `Find_landing/camera_registry.json` | Registry sensor camera và thông tin nhận diện phần cứng. |
| `Find_landing/camera_streamer.py` | Stream camera chính, publish video và chạy xử lý landing. |
| `Find_landing/find.py` | Detector contour/H landing target kiểu cũ trong Find_landing. |
| `Find_landing/landing_worker.py` | Worker tách riêng cho xử lý landing trên lores frame. |
| `Find_landing/test_find.py` | Script test detector contour/H. |

### Find_landing/processing

| File | Chức năng |
|---|---|
| `Find_landing/processing/__init__.py` | Đánh dấu package xử lý frame. |
| `Find_landing/processing/base.py` | Kiểu dữ liệu nền: `FrameMeta`, `ProcessResult`, `FrameProcessor`. |
| `Find_landing/processing/detect_config.py` | Đọc tham số detect size, frame skip, hold/reacquire từ config. |
| `Find_landing/processing/overlay.py` | Vẽ overlay detection lên frame. |
| `Find_landing/processing/overlay_style.py` | Helper style chữ/đường vẽ cho overlay. |
| `Find_landing/processing/pipeline.py` | Xây pipeline processor cho từng frame. |
| `Find_landing/processing/registry.py` | Chọn danh sách processor theo config. |
| `Find_landing/processing/smooth_tracker.py` | Làm mượt vị trí/kích thước target qua nhiều frame. |

### Find_landing/processing/detectors

| File | Chức năng |
|---|---|
| `Find_landing/processing/detectors/__init__.py` | Plugin registry cho các mode detector. |
| `Find_landing/processing/detectors/aruco/__init__.py` | Adapter plugin detector ArUco. |
| `Find_landing/processing/detectors/aruco/board.py` | Tính pose/quality cho marker đơn và board nhiều marker. |
| `Find_landing/processing/detectors/aruco/calibration.py` | Load calibration camera và scale matrix theo output size. |
| `Find_landing/processing/detectors/aruco/compat.py` | Tương thích OpenCV ArUco API cũ/mới. |
| `Find_landing/processing/detectors/aruco/detect.py` | Logic detect ArUco marker/board, offset, pose và trạng thái target. |
| `Find_landing/processing/detectors/aruco/event_log.py` | Ghi sự kiện landing target để debug/tracking. |
| `Find_landing/processing/detectors/aruco/marker.py` | Tạo PNG marker ArUco và board sheet. |
| `Find_landing/processing/detectors/aruco/overlay.py` | Vẽ overlay riêng cho kết quả ArUco. |
| `Find_landing/processing/detectors/aruco/processor.py` | Processor class bọc detector ArUco vào pipeline. |
| `Find_landing/processing/detectors/aruco/stability.py` | Đánh giá độ ổn định detection. |
| `Find_landing/processing/detectors/aruco/track_state.py` | Lưu trạng thái target qua frame để chống nhấp nháy/lost. |
| `Find_landing/processing/detectors/contour_h/__init__.py` | Adapter plugin detector contour chữ H. |
| `Find_landing/processing/detectors/contour_h/detect.py` | Detect landing pad bằng contour/template H. |
| `Find_landing/processing/detectors/contour_h/overlay.py` | Vẽ overlay cho detector contour H. |
| `Find_landing/processing/detectors/contour_h/processor.py` | Processor class bọc detector contour H. |
| `Find_landing/processing/detectors/contour_h/stability.py` | Đánh giá ổn định kết quả contour H. |
| `Find_landing/processing/detectors/contour_h/template.py` | Load template H/A/K/Y cho detector contour. |

### Find_landing/stream

| File | Chức năng |
|---|---|
| `Find_landing/stream/__init__.py` | Lazy import các thành phần stream để giảm phụ thuộc lúc import. |
| `Find_landing/stream/capture_loop.py` | Loop capture frame từ camera và ghi sang pipe. |
| `Find_landing/stream/capture_source.py` | Abstraction nguồn frame camera. |
| `Find_landing/stream/encoder.py` | Encode/ghi frame sang sink H264/pipe. |
| `Find_landing/stream/frame_gate.py` | Điều tiết frame để giữ FPS và tránh backlog. |
| `Find_landing/stream/h264_cv_loop.py` | Loop stream H264 kèm xử lý OpenCV/overlay. |
| `Find_landing/stream/metrics.py` | Ghi stats stream và telemetry landing ra `/tmp`. |
| `Find_landing/stream/wire_format.py` | Chuyển pixel format/crop/resize giữa sensor, BGR và wire format. |

### Find_landing/templates

| File | Chức năng |
|---|---|
| `Find_landing/templates/A.png` | Template chữ A cho detector contour. |
| `Find_landing/templates/H.png` | Template chữ H cho detector contour. |
| `Find_landing/templates/K.png` | Template chữ K cho detector contour. |
| `Find_landing/templates/Y.png` | Template chữ Y cho detector contour. |
| `Find_landing/templates/aruco_board_dict_4x4_50_0-11.png` | Board ArUco gồm marker ID 0-11. |
| `Find_landing/templates/aruco_dict_4x4_50_id0.png` đến `id11.png` | Marker ArUco riêng lẻ để in/test. |
| `Find_landing/templates/charuco_calibration_7x5.png` | Board ChArUco dùng hiệu chuẩn camera. |

## Module_4G

| File | Chức năng |
|---|---|
| `Module_4G/4G_control_at_testor.py` | Điều khiển/diagnose module 4G bằng GPIO và AT command. |
| `Module_4G/connection_manager.py` | Network monitor: ưu tiên Ethernet/WiFi/4G, route, fallback, trạng thái mạng. |
| `Module_4G/enable_4g_auto.py` | Tự bật module 4G, chuẩn bị QMI, lấy IP và cấu hình interface. |
| `Module_4G/module_4g_knowledge.md` | Ghi chú kiến thức vận hành module 4G. |
| `Module_4G/set_4g_mode.py` | Đổi chế độ 4G/policy kết nối. |
| `Module_4G/test_4g_diagnose.sh` | Script diagnose nhanh module 4G. |
| `Module_4G/vn_carriers.py` | Mapping nhà mạng Việt Nam theo IMSI/MCC-MNC. |

## _bootstrap

| File | Chức năng |
|---|---|
| `_bootstrap/_apt.py` | Logic cài apt dependency cho project. |
| `_bootstrap/pyproject.toml` | Metadata build package bootstrap. |
| `_bootstrap/setup.py` | Hook setuptools để chạy apt install khi cài package. |
| `_bootstrap/uavlink_apt_bootstrap.egg-info/PKG-INFO` | Metadata package đã sinh bởi setuptools. |
| `_bootstrap/uavlink_apt_bootstrap.egg-info/SOURCES.txt` | Danh sách source package đã sinh. |
| `_bootstrap/uavlink_apt_bootstrap.egg-info/dependency_links.txt` | Metadata dependency links đã sinh. |
| `_bootstrap/uavlink_apt_bootstrap.egg-info/top_level.txt` | Metadata top-level package đã sinh. |

## data

| File/Thư mục | Chức năng |
|---|---|
| `data/` | Thư mục runtime. Hiện chỉ nên chứa file sinh khi chạy như log, lock, connection config. |
| `data/logs/` | Nơi ghi log runtime; log đã được dọn và sẽ tự sinh lại. |

## images

| File | Chức năng |
|---|---|
| `images/pilot-ui.jpg` | Ảnh minh họa giao diện cloud trong README. |
| `images/image.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/image copy.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/image copy 2.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/image copy 3.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/image copy 4.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/image copy 5.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/image copy 6.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/image copy 7.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/image copy 8.png` | Ảnh tài liệu/minh họa thao tác. |
| `images/huong_dan_do_camera_fov.svg` | Sơ đồ hướng dẫn đo FOV camera. |
| `images/calibration/charuco_7x5.png` | Board ChArUco dùng hiệu chuẩn. |
| `images/calibration/session_02/sample_001.jpg` đến `sample_030.jpg` | Bộ ảnh mẫu dùng hiệu chuẩn camera session 02. |

## tests

| File | Chức năng |
|---|---|
| `tests/test_aruco_landing.py` | Unit test detector ArUco, tracking, calibration và MAVLink gate. |
| `tests/test_forwarder_compat.py` | Test tương thích forwarder với battery status và command packing. |
| `tests/test_laptop_gps.py` | Test parse/inject GPS từ laptop/browser. |
| `tests/test_telemetry.py` | Test decode telemetry/flight mode. |

## tools

| File | Chức năng |
|---|---|
| `tools/calibrate_camera_auto.py` | Tool hiệu chuẩn camera tự động từ ảnh ChArUco. |
| `tools/calibrate_webcam_charuco.py` | Tool thu mẫu live/webcam và hiệu chuẩn ChArUco. |
| `tools/landing_preflight_check.py` | Kiểm tra trước khi chạy landing: config, marker, calibration, stream. |
| `tools/landing_target_sitl.py` | Tool phát landing target cho môi trường SITL. |
| `tools/laptop_gps_receiver.py` | Nhận GPS từ browser/laptop và inject vào MAVLink. |
| `tools/laptop_gps_sender.py` | Gửi GPS browser từ laptop đến Pi. |
| `tools/replay_aruco.py` | Replay detector ArUco trên ảnh/video để debug. |

## web

| File | Chức năng |
|---|---|
| `web/__init__.py` | Đánh dấu package web. |
| `web/camera_handlers.py` | Handler API camera tách khỏi server chính. |
| `web/camera_probe.py` | Probe CSI/USB camera bằng Picamera2, libcamera, V4L2/OpenCV. |
| `web/camera_service.py` | Service layer cấu hình camera, stream, landing telemetry và overlay/reboot. |
| `web/landing_handlers.py` | Handler API template/config landing. |
| `web/mavlink_bridge.py` | Bridge API web với MAVLink params/command. |
| `web/network_helpers.py` | Chuẩn hóa trạng thái network và signal 4G cho API/UI. |
| `web/network_mode.py` | Ánh xạ/apply network mode giữa UI và netmon legacy. |
| `web/server.py` | Flask web server chính và các route API. |

### web/static

| File | Chức năng |
|---|---|
| `web/static/PX4ParameterFactMetaData.xml` | Metadata PX4 parameter để hiển thị/chỉnh tham số. |
| `web/static/app-shell.css` | CSS khung giao diện web. |
| `web/static/app-shell.js` | JavaScript khung app shell/navigation. |
| `web/static/common-nav.css` | CSS navigation chung. |
| `web/static/common-nav.js` | JavaScript navigation chung. |
| `web/static/connect.html` | Trang cấu hình/kết nối drone-cloud/network. |
| `web/static/dashboard.html` | Dashboard trạng thái chính. |
| `web/static/embed-themes.css` | CSS theme khi nhúng trang. |
| `web/static/mavlink.html` | Trang trạng thái MAVLink. |
| `web/static/mavlink_settings.html` | Trang cấu hình MAVLink. |
| `web/static/params.html` | Trang xem/sửa MAVLink/PX4 parameters. |
| `web/static/settings.html` | Trang settings: camera, network, hardware. |
| `web/static/tokens.html` | Trang quản lý API key/token. |

## how_do_drones_work

`how_do_drones_work/` là project tham khảo riêng về DroneKit/OpenCV/precise landing. Nó không phải luồng chạy chính của UAVLink-Edge-Python nhưng hữu ích để tham khảo thuật toán.

| File | Chức năng |
|---|---|
| `how_do_drones_work/.gitignore` | Ignore rule của project tham khảo. |
| `how_do_drones_work/LICENSE` | License của project tham khảo. |
| `how_do_drones_work/README.md` | Tài liệu project tham khảo. |
| `how_do_drones_work/libs/__init__.py` | Package marker cho thư viện tham khảo. |
| `how_do_drones_work/libs/plane.py` | Wrapper DroneKit cho thao tác máy bay/drone. |
| `how_do_drones_work/opencv/ChessBoard_9x6.jpg` | Ảnh chessboard để calibration OpenCV. |
| `how_do_drones_work/opencv/OPENCV_INSTALL.txt` | Ghi chú cài OpenCV. |
| `how_do_drones_work/opencv/README.txt` | Tài liệu thư mục OpenCV. |
| `how_do_drones_work/opencv/__init__.py` | Package marker cho module OpenCV. |
| `how_do_drones_work/opencv/aruco_pose_estimation.py` | Demo ước lượng pose ArUco. |
| `how_do_drones_work/opencv/cameraDistortion_raspi.txt` | Distortion coefficients camera Raspberry Pi. |
| `how_do_drones_work/opencv/cameraDistortion_webcam.txt` | Distortion coefficients webcam. |
| `how_do_drones_work/opencv/cameraMatrix_raspi.txt` | Camera matrix Raspberry Pi. |
| `how_do_drones_work/opencv/cameraMatrix_webcam.txt` | Camera matrix webcam. |
| `how_do_drones_work/opencv/cameracalib.py` | Script calibration camera bằng OpenCV. |
| `how_do_drones_work/opencv/common.py` | Helper OpenCV dùng chung trong demo cũ. |
| `how_do_drones_work/opencv/lib_aruco_pose.py` | Class tracking pose ArUco đơn. |
| `how_do_drones_work/opencv/save_snapshots.py` | Tool chụp snapshot camera. |
| `how_do_drones_work/scripts/01_test_connect.py` | Demo kết nối DroneKit và arm/takeoff. |
| `how_do_drones_work/scripts/02_control_with_arrow_keys.py` | Demo điều khiển bằng phím mũi tên. |
| `how_do_drones_work/scripts/03_read_telemetry.py` | Demo đọc telemetry DroneKit. |
| `how_do_drones_work/scripts/04_mission.py` | Demo mission waypoint. |
| `how_do_drones_work/scripts/05_trajectory_tracking.py` | Demo bám trajectory/waypoint. |
| `how_do_drones_work/scripts/06_precise_landing.py` | Demo precise landing bằng ArUco. |
| `how_do_drones_work/scripts/__init__.py` | Package marker cho scripts. |
| `how_do_drones_work/scripts/rcbenchmark/rcbenchmark_udp.js` | Demo UDP RCbenchmark bằng JavaScript. |
| `how_do_drones_work/scripts/rcbenchmark/rcbenchmark_udp.py` | Demo UDP RCbenchmark bằng Python. |
| `how_do_drones_work/tests/__init__.py` | Package marker cho tests. |
| `how_do_drones_work/tests/test_precise_landing_mavlink.py` | Test/gửi message MAVLink precise landing. |

## Ghi chú dọn thêm

- `Auto_landing/camera_manager.py.broken` có vẻ là file backup lỗi. Tôi chưa xóa vì có thể bạn muốn giữ để so sánh.
- `_bootstrap/uavlink_apt_bootstrap.egg-info/` là metadata build sinh ra nhưng đang được Git track, nên tôi chưa xóa tự động.
- `venv/` và `how_do_drones_work/.venv/` chiếm nhiều file nhất nhưng là môi trường chạy. Chỉ xóa khi bạn chắc chắn sẽ tạo lại được dependency.
