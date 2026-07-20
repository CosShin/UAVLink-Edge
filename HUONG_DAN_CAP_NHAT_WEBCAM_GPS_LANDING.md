# Cập nhật webcam USB và ArUco Precision Landing

Tài liệu này ghi lại các thay đổi đã thực hiện trong dự án UAVLink Edge Python,
cấu hình hiện tại, cách chạy và quy trình kiểm thử với Raspberry Pi/CM5, webcam
USB và Pixhawk 2.4.8 chạy ArduPilot.

> Cảnh báo an toàn: chỉ thử điều khiển khi drone ở trạng thái DISARMED và đã
> tháo cánh/quạt. Không dùng tọa độ giả hoặc vị trí Wi-Fi/IP làm nguồn định vị.

## 1. Những phần đã cập nhật

### 1.1 Hỗ trợ webcam USB

- Thêm nguồn camera USB V4L2 bên cạnh camera CSI.
- Cho phép chọn thiết bị như `/dev/video0` và định dạng đầu vào MJPEG.
- Thêm dò camera USB và cấu hình qua trang Settings.
- Tự kết nối lại nếu webcam bị rút ra rồi cắm lại.
- Có hai đường truyền:
  - Direct mode: V4L2 → FFmpeg → RTSP, độ trễ thấp nhất nhưng không có nhận diện/overlay.
  - CV mode: webcam → OpenCV → ArUco/overlay → H.264 → RTSP.

Các file chính đã thay đổi:

- `web/camera_probe.py`
- `web/camera_service.py`
- `web/camera_handlers.py`
- `web/static/settings.html`
- `Find_landing/camera_manager.py`
- `Find_landing/camera_streamer.py`
- `Find_landing/stream/capture_source.py`
- `Find_landing/stream/capture_loop.py`

### 1.2 Cấu hình video CAM0 hiện tại

CAM0 đang dùng:

```yaml
camera_id: 0
source: usb
device_path: /dev/video0
usb_input_format: mjpeg
usb_direct_mode: false
size: [1280, 720]
framerate: 30
bitrate: 2500
keyframe_interval: 15
preset: ultrafast
tune: zerolatency
detection_enabled: true
overlay_enabled: true
overlay_burn_enabled: true
landing_detection_mode: aruco
aruco_dictionary: DICT_4X4_50
aruco_marker_id: 5
lores_size: [320, 240]
detect_frame_skip: 2
overlay_frame_skip: 2
```

Video gửi lên server ở 1280×720. Detector chỉ xử lý ảnh 320×240 rồi scale kết
quả lên khung hình chính để giảm tải CPU. Vì overlay được burn vào video nên
`usb_direct_mode` phải là `false`.

Marker test hiện tại:

```text
Find_landing/templates/aruco_dict_4x4_50_id5.png
```

### 1.3 ArUco và LANDING_TARGET

Đã sửa các vấn đề sau:

- Trước đây `offset_x` và `offset_y` là pixel nhưng bị gửi trực tiếp như radian.
- Giờ pixel được chuyển sang góc dựa trên HFOV/VFOV thật của webcam.
- Trục Y của ảnh được đổi dấu đúng trước khi tạo `LANDING_TARGET`.
- `LANDING_TARGET` được gửi cả vào Pixhawk và mirror lên server.
- Chỉ gửi target đang được phát hiện thật; không gửi dữ liệu đang ở trạng thái hold.
- File telemetry landing được cập nhật 10 Hz thay vì mỗi 5 giây.
- Bổ sung kích thước frame và kích thước marker vào telemetry.

Các file liên quan:

- `landing_mavlink.py`
- `Find_landing/stream/metrics.py`
- `Find_landing/stream/capture_loop.py`
- `Find_landing/stream/h264_cv_loop.py`

Mặc định bridge điều khiển hạ cánh vẫn tắt:

```yaml
landing:
  mavlink_enabled: false
  mavlink_hz: 10
  mavlink_camera_id: 0
  camera_hfov_deg: 0
  camera_vfov_deg: 0
```

Phải đo FOV và xác minh chiều lắp camera trước khi bật.

## 2. Chuẩn bị Pixhawk 2.4.8

Pixhawk 2.4.8 dùng firmware target `Pixhawk1`. Dùng bản ArduPilot stable đúng cho
Pixhawk1, không chọn firmware Pixhawk4.

Nếu Pi nối vào TELEM2, kiểm tra các tham số tương ứng:

```text
SERIAL2_PROTOCOL = 2
SERIAL2_BAUD     = 921
```

`SERIAL2_BAUD=921` tương ứng 921600 baud trong `config.yaml`. Nếu dùng TELEM1
hoặc cổng khác, phải đổi đúng số `SERIALx`.

Kết nối UART:

- TX của Pi nối RX của Pixhawk.
- RX của Pi nối TX của Pixhawk.
- Hai thiết bị phải chung GND.
- Không cấp nguồn Pixhawk từ chân 5V của UART.

## 3. Chạy toàn bộ hệ thống

### Terminal 1: chạy UAVLink

Từ thư mục dự án:

```bash
cd ~/test/tess2/UAVLink-Edge-Python
./run.sh
```

Chỉ chạy một instance `main.py`. Khi chương trình khởi động đúng cần thấy các
thông tin tương tự:

```text
Pixhawk connected
Forwarder started
Camera streaming started
```

Sau mỗi lần sửa `config.yaml`, hãy dừng chương trình cũ bằng Ctrl+C rồi chạy lại
`./run.sh`.

## 4. Kiểm thử từng phần

### 4.1 Kiểm tra webcam USB

Trên Pi:

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
v4l2-ctl --device=/dev/video0 --list-formats-ext
```

Webcam cần hỗ trợ MJPEG 1280×720 ở 30 fps. Nếu không hỗ trợ, đổi
`usb_input_format` hoặc chọn mode mà `v4l2-ctl` liệt kê.

Chạy hệ thống và kiểm tra server:

- Video phải có độ phân giải 1280×720.
- FPS mục tiêu khoảng 30.
- Dùng WebRTC/WHEP để xem realtime, không dùng HLS khi điều khiển drone.
- Đưa marker ID 5 vào giữa ảnh; stream phải vẽ khung, tâm, offset và hướng.

Nếu video trễ nhiều:

1. Kiểm tra server/client đang dùng WebRTC UDP, không fallback TURN/TCP hoặc HLS.
2. Giữ GOP 15 và `tune: zerolatency`.
3. Kiểm tra log `encode drops` và CPU của Pi.
4. Nếu Pi quá tải, tăng `detect_frame_skip` lên 3 hoặc giảm FPS xuống 24/25.
5. Chỉ khi không cần marker overlay mới tắt detection/overlay và bật
   `usb_direct_mode: true`.

### 4.2 Kiểm tra marker telemetry

Trong lúc camera đang chạy và marker ID 5 đang xuất hiện:

```bash
watch -n 0.5 cat /tmp/camera_landing_0.json
```

Kết quả mong đợi:

```json
{
  "camera_id": 0,
  "detected": true,
  "hold": false,
  "offset_x": 0,
  "offset_y": 0,
  "h_size": [100, 100],
  "frame_width": 1280,
  "frame_height": 720,
  "updated_at": 0
}
```

Giá trị thực tế sẽ thay đổi. Khi di chuyển marker sang phải, `offset_x` phải tăng.
Khi marker đi lên phía trên ảnh, `offset_y` phải tăng.

### 4.3 Đo FOV webcam

Không dùng FOV đoán cho precision landing.

1. Đặt webcam vuông góc với một bức tường, cách tường khoảng `D` mét.
2. Đo chiều rộng vùng nhìn thấy trên tường là `W` mét.
3. Đo chiều cao vùng nhìn thấy là `H` mét.
4. Tính:

```text
HFOV = 2 × atan(W / (2 × D))
VFOV = 2 × atan(H / (2 × D))
```

Đổi kết quả từ radian sang độ rồi nhập vào `config.yaml`:

```yaml
landing:
  mavlink_enabled: true
  mavlink_hz: 10
  mavlink_camera_id: 0
  camera_hfov_deg: 60.0  # thay bằng kết quả thật
  camera_vfov_deg: 45.0  # thay bằng kết quả thật
```

Khởi động lại `./run.sh` sau khi sửa.

### 4.4 Cấu hình và test Precision Landing

Trong Mission Planner:

```text
PLND_ENABLED = 1
```

Reboot Pixhawk để hiện đầy đủ tham số, sau đó đặt:

```text
PLND_TYPE      = 1
PLND_YAW_ALIGN = góc lắp camera so với hướng trước của drone
```

Nên có rangefinder hướng xuống và khai báo đúng vị trí camera bằng các tham số
`PLND_CAM_POS_X/Y/Z` nếu camera không nằm tại tâm drone.

Thứ tự kiểm tra an toàn:

1. Tháo cánh/quạt, DISARMED, kiểm tra marker và dấu offset.
2. Nghiêng/di chuyển drone bằng tay để xác nhận hướng hiệu chỉnh không bị ngược.
3. Kiểm tra `LANDING_TARGET` được gửi liên tục khi `detected=true`.
4. Lắp cánh và thử ở khu vực an toàn, có người giữ quyền điều khiển/failsafe.
5. Thử Precision Loiter ở độ cao thấp trước.
6. Chỉ thử LAND/RTL sau khi tracking, rangefinder và EKF đều ổn định.

Precision Landing vẫn cần ước lượng vị trí ngang và attitude ổn định. Marker
camera không thay thế hoàn toàn EKF, optical flow/GPS/VIO hoặc rangefinder.

## 5. Test phần mềm đã thực hiện

Các kiểm tra sau đã chạy thành công trong môi trường dự án:

- Compile `main.py`, `forwarder.py`, `landing_mavlink.py` và các
  module stream đã chỉnh sửa.
- Parse `config.yaml` và sinh lại `Find_landing/camera_config_0.json`.
- Xác nhận cấu hình CAM0 là 1280×720, 30 fps và detector 320×240.
- Nhận diện thành công ảnh ArUco `DICT_4X4_50`, ID 5.
- Kiểm tra phép đổi pixel sang góc radian theo FOV.
- Pack/parse thành công message MAVLink `LANDING_TARGET`.
- Xác nhận một message được gửi đồng thời tới kết nối Pixhawk và socket server.
- Kiểm tra `git diff --check` không có lỗi whitespace.

Chưa thể xác nhận phần cứng trực tiếp trong môi trường kiểm thử vì không nhìn
thấy `/dev/video0`, `/dev/ttyAMA0`, server localhost:8080 hoặc tiến trình chạy
thật trên CM5. Các bước phần cứng ở mục 4 cần thực hiện trực tiếp trên Pi.

## 6. Lệnh kiểm tra nhanh

Kiểm tra cú pháp:

```bash
venv/bin/python -m py_compile \
  main.py forwarder.py landing_mavlink.py \
  Find_landing/stream/metrics.py \
  Find_landing/stream/capture_loop.py \
  Find_landing/stream/h264_cv_loop.py
```

Sinh lại file camera từ `config.yaml`:

```bash
venv/bin/python -c "from config import Config; from web.camera_service import write_streamer_configs; print(write_streamer_configs(Config('config.yaml')))"
```

Kiểm tra API local khi `main.py` đang chạy:

```bash
curl http://127.0.0.1:8080/api/status
curl http://127.0.0.1:8080/api/connection
```

Kiểm tra process:

```bash
ps -ef | grep -E 'main.py|camera_streamer.py|mediamtx'
```

## 7. Tài liệu tham khảo

- ArduPilot MAVLink Precision Landing:
  <https://ardupilot.org/dev/docs/mavlink-precision-landing.html>
- ArduPilot Precision Landing and Loiter:
  <https://ardupilot.org/copter/docs/precision-landing-and-loiter.html>
- MAVLink Landing Target Protocol:
  <https://mavlink.io/en/services/landing_target.html>
