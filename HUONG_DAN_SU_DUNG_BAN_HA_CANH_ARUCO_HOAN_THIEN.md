# Hướng dẫn sử dụng bản webcam + hạ cánh ArUco v3

## 1. Trạng thái của bản này

Bản phần mềm đã triển khai các phần có thể kiểm chứng trong repository:

- Webcam USB CAM0 ở `1280×720`, 30 fps; detector chạy ảnh nhỏ `320×240` để giảm tải.
- Hai chế độ mục tiêu: một marker duy nhất (`single`) và hợp nhất bảng 3×4 ID 0–11 (`board`).
- Chặn marker trùng ID, marker sai ID, mục tiêu đổi giữa lúc khóa và board chỉ có một marker.
- Điểm chất lượng thật dựa trên diện tích, mép ảnh, hình dạng, reprojection error, RANSAC inlier và số marker.
- Máy trạng thái `SEARCH → ACQUIRING → TRACKING → LOST/AMBIGUOUS`.
- Chỉ phát `LANDING_TARGET` khi ảnh mới, không phải khung giữ tạm, không ambiguous, đủ chất lượng và đã khóa đủ số khung hình.
- Calibration ChArUco, PnP metric pose cho board, replay video, SITL scenario generator, preflight checker và JSONL event log.
- MAVLink từ server vẫn đi hai chiều qua forwarder; `LANDING_TARGET` được gửi đồng thời về Pixhawk và mirror lên server.

Chưa thể xác nhận bay thật chỉ bằng môi trường phát triển này. Tại thời điểm hoàn thiện code, môi trường không có `/dev/video0`, chưa có calibration của đúng webcam, không có Pixhawk/SITL đang kết nối và `landing.mavlink_enabled` vẫn là `false`. Đây là mặc định an toàn có chủ ý. Phase bench/HIL và bay thật chỉ được đánh dấu đạt sau khi làm checklist ở mục 11–12.

## 2. Kiến trúc và thuật toán bay

Luồng dữ liệu:

```text
Webcam USB 1280×720
  ├─ video encode H.264 → MediaMTX/server
  └─ resize 320×240 → ArUco detector
       → duplicate/quality gate
       → single target hoặc board homography + RANSAC (+ PnP nếu calibrated)
       → smoothing + target lock state machine
       → telemetry /tmp/camera_landing_0.json
       → LANDING_TARGET angle_x/angle_y @ 10 Hz
       → Pixhawk ArduCopter Precision Landing
```

Pi không tự xuất lệnh motor, roll/pitch hay tự viết một flight controller riêng. Pi chỉ đo mục tiêu và gửi `LANDING_TARGET`. ArduCopter trên Pixhawk 2.4.8 mới là bên chạy estimator, điều khiển ngang, điều khiển độ cao, LAND/RTL, retry khi mất mục tiêu và disarm khi chạm đất.

Trong `single`, chỉ ID cấu hình (mặc định ID 5) là bãi đáp. Các ID khác có thể xuất hiện nhưng không được chọn. Hai bản sao ID 5 trong cùng khung hình bị từ chối.

Trong `board`, các ID 0–11 là các phần của **một bãi đáp vật lý duy nhất**. Code khớp góc marker với layout, dùng homography + RANSAC để tìm tâm board. Mặc định phải thấy ít nhất hai ID khác nhau. Nếu có calibration và kích thước marker thật, code chạy `solvePnPRansac` để có pose mét. Không dùng `board` cho một marker ID 5 đứng riêng: tâm suy ra sẽ là tâm của cả board giả định và có thể nằm ngoài marker/khung hình.

## 3. Những file chính đã thêm hoặc nâng cấp

- `Find_landing/processing/detectors/aruco/board.py`: hình học board, RANSAC, quality và PnP.
- `Find_landing/processing/detectors/aruco/calibration.py`: đọc/scaling camera matrix.
- `Find_landing/processing/detectors/aruco/track_state.py`: khóa mục tiêu và trạng thái an toàn.
- `Find_landing/processing/detectors/aruco/event_log.py`: log sự kiện xoay vòng.
- `landing_mavlink.py`: đổi pixel sang góc, tự suy ra FOV từ calibration và fail-closed gate.
- `tools/calibrate_webcam_charuco.py`: tạo bảng và calibration webcam.
- `tools/replay_aruco.py`: replay video qua đúng processor production.
- `tools/landing_target_sitl.py`: phát kịch bản center/step/sine/loss chỉ vào loopback SITL.
- `tools/landing_preflight_check.py`: kiểm tra cấu hình và trạng thái mà không thay đổi hệ thống.
- `tests/test_aruco_landing.py`: unit/synthetic integration tests.

## 4. Cài đặt và kiểm tra webcam 1280×720

Tại thư mục dự án:

```bash
cd /home/minhan/test/tess2/UAVLink-Edge-Python
python3 install.py
```

Cắm webcam và kiểm tra:

```bash
ls -l /dev/video*
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

Phải thấy mode MJPEG `1280x720` gần 30 fps. Nếu camera nằm ở `/dev/video2`, sửa `device_path` trong `config.yaml`. Không dùng số `/dev/videoN` một cách mù quáng sau reboot; kiểm tra lại bằng `v4l2-ctl --list-devices`.

Cấu hình CAM0 hoàn thiện hiện tại:

```yaml
camera:
  streams:
  - camera_id: 0
    source: usb
    device_path: /dev/video0
    usb_input_format: mjpeg
    size: [1280, 720]
    framerate: 30
    bitrate: 2500
    keyframe_interval: 15
    preset: ultrafast
    tune: zerolatency
    usb_direct_mode: false
    detection_enabled: true
    overlay_enabled: true
    overlay_burn_enabled: true
    landing_detection_mode: aruco
    lores_size: [320, 240]
```

`usb_direct_mode` phải là `false` khi cần nhận diện và burn overlay. Chỉ bật `true` khi tắt detection/overlay và ưu tiên đường stream nhẹ nhất.

## 5. Chọn loại bãi đáp

### 5.1 Một marker duy nhất — dùng trước khi bench

Đây là mặc định hiện tại:

```yaml
aruco_dictionary: DICT_4X4_50
aruco_marker_id: 5
aruco_target_strategy: single
aruco_min_quality: 0.55
aruco_acquire_frames: 5
```

In file:

```text
Find_landing/templates/aruco_dict_4x4_50_id5.png
```

Khi in, phải giữ viền trắng quanh marker, không crop sát khung đen và không kéo méo tỉ lệ.

### 5.2 Board 3×4 — khuyến nghị sau khi đã test single

In đúng file:

```text
Find_landing/templates/aruco_board_dict_4x4_50_0-11.png
```

Sau đó đổi:

```yaml
aruco_target_strategy: board
aruco_board_first_id: 0
aruco_board_cols: 3
aruco_board_rows: 4
aruco_board_gap_x_ratio: 0.16
aruco_board_gap_y_ratio: 0.34
aruco_board_ransac_threshold_px: 3.0
aruco_board_min_markers: 2
```

Không tự sắp xếp lại ID, không thay khoảng cách hoặc cắt mất label/margin rồi vẫn giữ các ratio trên. Nếu tự thiết kế board khác, phải cập nhật layout tương ứng.

## 6. Calibration đúng webcam

Calibration phải thực hiện với đúng webcam, đúng focus, đúng mode `1280×720` và sau khi đã cố định camera trên drone. OpenCV khuyến nghị ChArUco cho calibration chính xác.

Tạo bảng calibration:

```bash
venv/bin/python tools/calibrate_webcam_charuco.py \
  --generate-board /tmp/charuco_7x5.png
```

In đúng tỉ lệ; các số `square-length-m` và `marker-length-m` trong lệnh phải khớp kích thước đo thật trên bản in. Thu 25 góc nhìn:

```bash
venv/bin/python tools/calibrate_webcam_charuco.py \
  --device /dev/video0 \
  --width 1280 --height 720 --fps 30 \
  --square-length-m 0.04 \
  --marker-length-m 0.03 \
  --samples 25 \
  --output Find_landing/camera_calibration_1280x720.yaml
```

Nhấn `SPACE` khi ảnh nét; thay đổi vị trí, khoảng cách và góc nghiêng, phủ cả bốn góc ảnh. Cần ít nhất 10 mẫu, nên dùng 20–30 mẫu. Không chấp nhận calibration bay khi RMS lớn hơn 1 px; nên chụp lại nếu các view có lỗi cao.

Khai báo:

```yaml
aruco_calibration_file: camera_calibration_1280x720.yaml
```

Để có pose mét trong chế độ board, đo cạnh phần marker đen-vuông thật, ví dụ 20 cm:

```yaml
aruco_marker_length_m: 0.20
```

Khi có calibration, bridge tự suy ra HFOV/VFOV từ camera matrix nếu hai giá trị `landing.camera_*fov_deg` vẫn là 0. Có thể đặt FOV thủ công nếu đã đo độc lập, nhưng không được đoán.

## 7. Chạy và quan sát detector mà chưa điều khiển drone

Giữ cấu hình an toàn:

```yaml
landing:
  mavlink_enabled: false
```

Chạy:

```bash
./run.sh
```

Ở terminal khác:

```bash
curl -s http://127.0.0.1:8080/api/camera/status
curl -s 'http://127.0.0.1:8080/api/camera/landing?camera_id=0&max_age_sec=2'
venv/bin/python tools/landing_preflight_check.py --require-live
```

Các trường quan trọng:

- `tracking_state=ACQUIRING`: đang đếm khung hình xác nhận, chưa điều khiển.
- `tracking_state=TRACKING` và `control_valid=true`: detector cho phép bridge dùng measurement.
- `LOST`: mất target; khung hold chỉ phục vụ hiển thị và không được điều khiển.
- `AMBIGUOUS`: trùng ID hoặc target thay đổi; không gửi điều khiển.
- `quality >= 0.55`: ngưỡng hiện tại; cần tune từ dataset thật, không hạ chỉ để làm cho test “xanh”.
- `measurement_monotonic_ms`: bridge từ chối dữ liệu cũ hơn 300 ms.

Log:

```bash
tail -f /tmp/camera_landing_events_0.jsonl
cat /tmp/camera_landing_0.json
cat /tmp/camera_stream_stats_0.json
```

Event log xoay vòng khoảng 5 MB và giữ một file `.1`.

## 8. Test tự động và replay video

Chạy test:

```bash
venv/bin/python -m unittest -v tests/test_aruco_landing.py
```

Bản hiện tại đạt 13/13 test, gồm: đúng/sai ID, hai marker cùng ID, board 12 ID, board thiếu marker, acquire nhiều khung hình, lost/recover, low quality, held frame, đổi target, scaling camera matrix, FOV và MAVLink fail-closed gate.

Replay video đã quay bằng đúng camera:

```bash
venv/bin/python tools/replay_aruco.py recordings/landing_test.mp4 \
  --config Find_landing/camera_config_0.json \
  --jsonl /tmp/aruco_replay.jsonl
```

Đọc kết quả:

```bash
tail -n 20 /tmp/aruco_replay.jsonl
```

Replay synthetic 60 frame trong lúc phát triển cho 30 lần detector update và 26 lần `control_valid`; bốn lần đầu bị chặn để hoàn thành acquire 5 measurement. Đây không thay thế dataset ngoài trời, rung, blur, bóng, chói và che khuất.

## 9. SITL trước Pixhawk thật

Cài và chạy ArduCopter SITL theo tài liệu ArduPilot. Đảm bảo SITL/MAVProxy gửi heartbeat về loopback UDP 14550; không để Mission Planner/QGC chiếm cùng cổng.

Ví dụ ở repository ArduPilot:

```bash
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map \
  --out=udp:127.0.0.1:14550
```

Ở dự án này, phát các tình huống chỉ vào loopback:

```bash
venv/bin/python tools/landing_target_sitl.py --sitl-confirm \
  --endpoint udpin:127.0.0.1:14550 \
  --pattern center --duration 30

venv/bin/python tools/landing_target_sitl.py --sitl-confirm \
  --endpoint udpin:127.0.0.1:14550 \
  --pattern sine --amplitude-deg 8 --noise-deg 0.5 \
  --packet-loss 0.1 --dropout-start 15 --dropout-duration 4 --duration 40
```

Tool từ chối endpoint không phải loopback và từ chối chạy nếu thiếu `--sitl-confirm`. Kiểm tra center, step, sine, noise, packet loss và dropout; xác nhận ArduCopter không tạo chuyển động đột ngột và thực hiện đúng `PLND_STRICT`/retry.

## 10. Kết nối Pixhawk 2.4.8 / ArduPilot

Pixhawk 2.4.8 thường dùng target firmware `Pixhawk1`, không phải `Pixhawk4`. Dùng stable firmware đúng board và lưu toàn bộ parameter trước khi thay đổi.

Ví dụ khi Pi nối TELEM2, cần xác minh mapping trên board/firmware:

```text
SERIAL2_PROTOCOL = 2      # MAVLink2
SERIAL2_BAUD     = 921    # 921600, khớp config
PLND_ENABLED     = 1
reboot Pixhawk
PLND_TYPE        = 1      # MAVLink LANDING_TARGET
```

Các tham số phải đánh giá theo frame và sân test, không copy mù quáng:

- `PLND_YAW_ALIGN`: hướng trục X camera so với hướng trước thân drone.
- `PLND_CAM_POS_X/Y/Z`: offset camera so với tâm thân.
- `PLND_XY_DIST_MAX`: không cho descent khi lệch ngang quá xa.
- `PLND_STRICT`, `PLND_ALT_MAX`, `PLND_ALT_MIN`, `PLND_RET_MAX`, `PLND_RET_BEHAVE`: hành vi mất marker/retry.
- `LAND_SPD_MS`, `LAND_ALT_LOW_M`: tốc độ cuối và ngưỡng đổi tốc độ.

Precision Landing vẫn cần horizontal position estimate và attitude ổn định. GPS giả/cố định không tạo ra định vị indoor an toàn. Để bay trong nhà cần một nguồn vị trí phù hợp như optical flow + rangefinder, VIO, motion capture hoặc beacon đã được ArduPilot hỗ trợ và kiểm chứng.

Sau calibration, đặt `mavlink_enabled: true` **chỉ cho SITL/bench tháo cánh trước**:

```yaml
landing:
  mavlink_enabled: true
  mavlink_hz: 10
  mavlink_camera_id: 0
  camera_hfov_deg: 0       # tự lấy từ calibration
  camera_vfov_deg: 0
  min_quality: 0.55
  max_measurement_age_ms: 300
  require_control_valid: true
```

Restart app sau khi sửa. Nếu không có calibration và FOV vẫn bằng 0, bridge tự tắt và ghi lỗi; không có `LANDING_TARGET` được phát.

## 11. Bench/HIL bắt buộc trước khi lắp cánh

Tháo toàn bộ cánh/quạt, giữ drone DISARMED:

1. Chạy preflight với `--require-live`; không được có `ERROR`.
2. Chạy `--probe-camera` sau khi dừng streamer để xác nhận frame thật 1280×720:

   ```bash
   curl -s -X POST http://127.0.0.1:8080/api/camera/stop
   venv/bin/python tools/landing_preflight_check.py --require-live --probe-camera
   curl -s -X POST http://127.0.0.1:8080/api/camera/start
   ```

3. Di chuyển marker sang trái/phải/trước/sau và xác nhận dấu `angle_x/angle_y` bằng log/Mission Planner. Nếu drone phản ứng ngược ở HIL, dừng và sửa orientation/FOV; không “bù” bằng đảo dấu ngẫu nhiên.
4. Đưa hai ID 5 vào ảnh: phải có `AMBIGUOUS`, `control_valid=false`, không có LANDING_TARGET mới.
5. Che marker: phải chuyển `LOST`; khung hình hold không được gửi điều khiển.
6. Tắt camera/rút USB: stream và telemetry phải stale, bridge ngừng phát trong tối đa 300 ms theo measurement gate.
7. Tạo CPU/network load và kiểm tra `measurement_monotonic_ms`, FPS, event log; dữ liệu cũ không được điều khiển.
8. Kiểm tra RC override, mode switch thoát LAND/RTL, failsafe radio, battery, EKF, rangefinder và geofence.
9. Xem DataFlash log `PL`, attitude, EKF, rangefinder và mode transition. Không qua bench nếu dấu trục, timestamp hoặc retry chưa đúng.

## 12. Mở dần flight envelope

Không bay trong phòng chật ở chuyến đầu. Dùng khu test trống, lưới/cage nếu có, người quan sát và RC pilot sẵn sàng chuyển mode.

1. Marker detection khi drone đặt trên giá, motor không quay.
2. Hover thấp bằng pilot, precision landing chỉ quan sát/log.
3. Precision Loiter/align ở độ cao an toàn, chưa descent tự động.
4. LAND từ độ cao thấp, tốc độ giảm, marker lớn, ánh sáng đều.
5. Lặp với offset nhỏ, mất marker có chủ ý và kiểm tra retry.
6. Chỉ tăng độ cao/tốc độ/góc nhìn sau nhiều lần không có outlier, reverse-axis hoặc stale control.
7. Cuối cùng mới thử RTL có precision landing, vì RTL thêm các trạng thái điều hướng và chuyển mode.

Mỗi lần thay camera focus, vị trí lắp, độ phân giải/crop, board geometry, firmware hoặc tham số PLND phải quay lại calibration/replay/bench tương ứng.

## 13. Giảm lag video 1280×720

Cấu hình hiện tại ưu tiên latency: MJPEG input, H.264 `ultrafast`, `zerolatency`, bitrate 2500 kbit/s, GOP 15 và detector lores. Nếu vẫn lag:

1. Xem bằng WebRTC/WHEP qua UDP; tránh HLS khi điều khiển realtime.
2. Mở UDP WebRTC/ICE trên server/NAT và cấu hình public candidate đúng; TURN/TCP chỉ fallback.
3. Đo riêng capture FPS, encode drop, RTT và packet loss; không kết luận chỉ bằng cảm giác trên UI.
4. Giữ 1280×720 nhưng giảm 30 xuống 20–25 fps nếu Pi encode drop.
5. Thử bitrate 1800–2200 kbit/s khi uplink yếu; tăng quá cao làm queue dài, quá thấp làm marker nhòe.
6. Nếu cần độ trễ thấp nhất và không cần overlay nhận diện trên video, tắt detection/overlay rồi bật `usb_direct_mode: true`.
7. Không tăng `detect_size` lên 1280×720 trên Pi chỉ để “nét hơn”; trước tiên tăng kích thước marker, ánh sáng và shutter.

## 14. Tài liệu kỹ thuật chính thức

- ArduPilot Precision Landing: https://ardupilot.org/copter/docs/precision-landing-and-loiter.html
- ArduPilot LAND Mode: https://ardupilot.org/copter/docs/land-mode.html
- ArduPilot SITL: https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html
- MAVLink Landing Target Protocol: https://mavlink.io/en/services/landing_target.html
- OpenCV ArUco detection/pose: https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html
- OpenCV ChArUco detection/calibration: https://docs.opencv.org/master/df/d4a/tutorial_charuco_detection.html

Phân tích tình huống, thuật toán, bài báo khoa học và kế hoạch gốc chi tiết nằm trong `KE_HOACH_THUAT_TOAN_HA_CANH_ARUCO.md`.
