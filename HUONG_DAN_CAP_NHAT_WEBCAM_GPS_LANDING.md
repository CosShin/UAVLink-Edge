# Cập nhật webcam USB, GPS qua Wi-Fi và ArUco Precision Landing

Tài liệu này ghi lại các thay đổi đã thực hiện trong dự án UAVLink Edge Python,
cấu hình hiện tại, cách chạy và quy trình kiểm thử với Raspberry Pi/CM5, webcam
USB và Pixhawk 2.4.8 chạy ArduPilot.

> Cảnh báo an toàn: các bước dùng GPS cố định chỉ dành cho kiểm tra trên bàn,
> drone ở trạng thái DISARMED và đã tháo cánh/quạt. Không dùng tọa độ giả hoặc
> vị trí Wi-Fi/IP làm nguồn định vị cho chuyến bay thật.

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

### 1.3 GPS qua Wi-Fi

Đã thêm file chạy độc lập:

```text
wifi_gps.py
```

Đường dữ liệu:

```text
Điện thoại/nguồn GPS
  → JSON UDP tới IP của Pi, cổng 25100
  → wifi_gps.py
  → GPS_INPUT tại 127.0.0.1:14600
  → main.py/forwarder
  → Pixhawk
  → GPS_RAW_INT
  → server 45.117.171.237:14550
```

`127.0.0.1:14600` chỉ là chặng nội bộ giữa hai chương trình trên Pi, không phải
web local và không phải đích cuối. Server cuối vẫn là
`45.117.171.237:14550`.

Các tính năng an toàn của `wifi_gps.py`:

- Yêu cầu token khi nhận từ Wi-Fi, trừ khi chủ động cho phép không xác thực.
- Kiểm tra lat/lon, độ cao và độ chính xác đầu vào.
- Dừng phát GPS_INPUT nếu dữ liệu Wi-Fi cũ quá 2 giây.
- Chế độ `--fixed` bắt buộc có `--bench-confirm`.
- Cảnh báo nếu không phát hiện `main.py` tại localhost:8080.
- In rõ chặng nội bộ và địa chỉ server cuối.

Forwarder đã được cập nhật để:

- Chỉ nhận `GPS_INPUT` từ loopback `127.0.0.1:14600`.
- Chuyển GPS_INPUT vào đúng kết nối Pixhawk đang hoạt động.
- Gửi `GPS_RAW_INT` của Pixhawk lên server.
- Tự yêu cầu Pixhawk phát `GPS_RAW_INT` ở 2 Hz nếu chưa thấy stream GPS.
- Chuyển gói điều khiển từ server về Pixhawk qua cùng kết nối MAVLink.

### 1.4 ArUco và LANDING_TARGET

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

Để ArduPilot nhận `GPS_INPUT`:

```text
GPS1_TYPE = 14
```

Firmware cũ có thể dùng tên:

```text
GPS_TYPE = 14
```

Sau khi đổi GPS type phải reboot Pixhawk. Khi muốn dùng lại GPS vật lý, trả tham
số này về giá trị ban đầu/Auto.

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

### Terminal 2: chạy GPS qua Wi-Fi

Nhận GPS JSON từ điện thoại/thiết bị khác:

```bash
cd ~/test/tess2/UAVLink-Edge-Python
WIFI_GPS_TOKEN='thay-bang-token-rieng' ./wifi_gps.py
```

Khi chưa nhận tọa độ, terminal chỉ hiện:

```text
Nhận GPS JSON tại udp://0.0.0.0:25100
GPS_INPUT → udpout:127.0.0.1:14600
```

Hai dòng trên chưa có nghĩa là GPS đã hoạt động. Khi có nguồn GPS hợp lệ phải
thấy định kỳ:

```text
GPS OK lat=... lon=... alt=... fix=3 injected=...
```

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

### 4.2 Test GPS cố định trên bàn

Tháo cánh/quạt, để drone DISARMED và chạy:

```bash
./wifi_gps.py --fixed 21.0285,105.8542,10 --bench-confirm
```

Thay lat/lon/alt bằng vị trí test mong muốn. Đây là tọa độ giả cố định, chỉ dùng
để kiểm tra đường truyền.

Kết quả mong đợi:

1. Terminal `wifi_gps.py` hiện `GPS OK ... injected=N`.
2. Log `main.py` hiện listener GPS injection và yêu cầu phát GPS_RAW_INT.
3. Mission Planner hiển thị GPS 3D Fix và số vệ tinh giả lập.
4. Server chuyển từ `0 sats` sang số vệ tinh nhận từ Pixhawk.

Nếu vẫn 0 sats:

1. Xác nhận `main.py` được khởi động trước `wifi_gps.py`.
2. Xác nhận đã reboot sau khi đặt `GPS1_TYPE=14` hoặc `GPS_TYPE=14`.
3. Xác nhận `SERIALx_PROTOCOL=2` và baud khớp 921600.
4. Kiểm tra terminal có dòng `GPS OK`; nếu chỉ có dòng listener thì chưa có fix.
5. Xác nhận `network.forward_gps_raw_int: true` trong `config.yaml`.
6. Xác nhận server UDP đang dùng `45.117.171.237:14550`.

### 4.3 Test nguồn GPS thật gửi qua Wi-Fi

Wi-Fi không tự tạo ra vị trí. Nguồn gửi phải có cảm biến định vị thật, ví dụ điện
thoại có GPS hoặc module GNSS. Nguồn đó gửi JSON liên tục tới:

```text
udp://IP_CUA_PI:25100
```

JSON:

```json
{
  "token": "thay-bang-token-rieng",
  "lat": 21.0285,
  "lon": 105.8542,
  "alt_m": 10.0,
  "accuracy_m": 3.0,
  "speed_m_s": 0.0,
  "course_deg": 0.0,
  "fix_type": 3,
  "satellites": 10
}
```

Nguồn cần gửi liên tục khoảng 1–5 Hz. Nếu ngừng quá 2 giây, `wifi_gps.py` chủ
động dừng GPS_INPUT và báo `GPS Wi-Fi stale`.

Lưu ý quan trọng:

- Điện thoại để cạnh người vận hành sẽ cho vị trí người vận hành, không phải drone.
- Muốn theo dõi drone, nguồn GPS phải được gắn trên drone.
- Định vị dựa trên IP/router/BSSID Wi-Fi chỉ gần đúng và không dùng để điều khiển bay.
- Bay trong nhà nên dùng optical flow/VIO kết hợp rangefinder và EKF phù hợp.

### 4.4 Kiểm tra marker telemetry

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

### 4.5 Đo FOV webcam

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

### 4.6 Cấu hình và test Precision Landing

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

- Compile `main.py`, `forwarder.py`, `landing_mavlink.py`, `wifi_gps.py` và các
  module stream đã chỉnh sửa.
- Parse `config.yaml` và sinh lại `Find_landing/camera_config_0.json`.
- Xác nhận cấu hình CAM0 là 1280×720, 30 fps và detector 320×240.
- Nhận diện thành công ảnh ArUco `DICT_4X4_50`, ID 5.
- Kiểm tra phép đổi pixel sang góc radian theo FOV.
- Pack/parse thành công message MAVLink `LANDING_TARGET`.
- Xác nhận một message được gửi đồng thời tới kết nối Pixhawk và socket server.
- Kiểm tra chế độ GPS fixed/dry-run sinh `GPS OK` đúng định dạng.
- Kiểm tra `git diff --check` không có lỗi whitespace.

Chưa thể xác nhận phần cứng trực tiếp trong môi trường kiểm thử vì không nhìn
thấy `/dev/video0`, `/dev/ttyAMA0`, server localhost:8080 hoặc tiến trình chạy
thật trên CM5. Các bước phần cứng ở mục 4 cần thực hiện trực tiếp trên Pi.

## 6. Lệnh kiểm tra nhanh

Kiểm tra cú pháp:

```bash
venv/bin/python -m py_compile \
  main.py forwarder.py landing_mavlink.py wifi_gps.py \
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
ps -ef | grep -E 'main.py|wifi_gps.py|camera_streamer.py|mediamtx'
```

## 7. Tài liệu tham khảo

- ArduPilot GPS Input:
  <https://ardupilot.org/mavproxy/docs/modules/GPSInput.html>
- ArduPilot MAVLink Precision Landing:
  <https://ardupilot.org/dev/docs/mavlink-precision-landing.html>
- ArduPilot Precision Landing and Loiter:
  <https://ardupilot.org/copter/docs/precision-landing-and-loiter.html>
- MAVLink Landing Target Protocol:
  <https://mavlink.io/en/services/landing_target.html>
