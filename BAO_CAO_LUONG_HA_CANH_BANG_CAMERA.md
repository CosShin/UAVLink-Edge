# Báo cáo luồng hoạt động hạ cánh bằng camera

## 1. Mục đích

Tài liệu này giải thích luồng hoạt động của chức năng Precision Landing trong
dự án, từ lúc camera nhìn thấy marker ArUco đến lúc Pixhawk điều khiển drone hạ
cánh.

Điểm quan trọng nhất:

> Camera và Raspberry Pi không trực tiếp điều khiển motor và cũng không tự ra
> lệnh hạ cánh. Pi chỉ đo marker rồi gửi sai lệch mục tiêu cho Pixhawk bằng bản
> tin MAVLink `LANDING_TARGET`. Pixhawk chạy ArduCopter mới là thiết bị điều
> khiển drone bay ngang, hạ độ cao và disarm sau khi chạm đất.

## 2. Các thành phần tham gia

| Thành phần | Trách nhiệm |
|---|---|
| Marker ArUco | Làm mục tiêu hạ cánh có ID xác định |
| Camera hướng xuống | Chụp hình khu vực bên dưới drone |
| Detector trên Pi | Tìm marker, đo tâm marker, chất lượng và trạng thái tracking |
| Safety gate trên Pi | Chặn kết quả yếu, cũ, giữ hình hoặc không rõ ràng |
| MAVLink bridge trên Pi | Đổi pose camera sang BODY_FRD `x/y/z` và gửi `LANDING_TARGET` |
| Pixhawk/ArduCopter | Kết hợp target với EKF, attitude và độ cao để điều khiển drone |
| Rangefinder | Cảm biến phụ cho Pixhawk nếu có; bridge hiện không dùng rangefinder |
| Pilot/GCS | Chuyển mode LAND/RTL, giám sát và hủy hạ cánh khi có sự cố |

## 3. Sơ đồ tổng thể

```text
Marker ArUco dưới đất
          │
          ▼
Camera USB 1280×720, hướng xuống
          │  khung hình BGR
          ▼
Detector ArUco trên Raspberry Pi
          │  ID, tâm marker, quality, trạng thái tracking
          ▼
Safety gate
          │
          ├── Không hợp lệ ──► không gửi LANDING_TARGET
          │
          └── Hợp lệ
                 │
                 ▼
       Đổi pose OpenCV thành BODY_FRD x/y/z
                 │
                 ▼
        MAVLink LANDING_TARGET, khoảng 10 Hz
                 │
                 ▼
          Pixhawk chạy ArduCopter
                 │
       kết hợp EKF + attitude + altitude estimator
                 │
                 ▼
     Điều khiển ngang + tốc độ hạ + land detector
                 │
                 ▼
          Chạm đất và tự disarm
```

## 4. Điều kiện để chức năng được sử dụng

### 4.1 Trên Raspberry Pi

Camera 0 trong `config.yaml` đang được cấu hình:

```yaml
camera:
  streams:
  - camera_id: 0
    source: usb
    device_path: /dev/video0
    size: [1280, 720]
    landing_detection_mode: aruco
    aruco_marker_id: 6
    aruco_target_strategy: single
    aruco_marker_length_m: 0.105
    aruco_calibration_file: camera_calibration_1280x720.yaml
    aruco_min_quality: 0.55
    aruco_acquire_frames: 5
```

Đầu ra MAVLink hiện đang mặc định tắt an toàn:

```yaml
landing:
  mavlink_enabled: false
  mavlink_hz: 10
  mavlink_camera_id: 0
  min_quality: 0.55
  max_measurement_age_ms: 300
  require_control_valid: true
```

Chỉ bật `mavlink_enabled: true` sau khi đã kiểm tra SITL và bench tháo cánh.

### 4.2 Trên Pixhawk

Ví dụ cấu hình cơ bản cho ArduCopter:

```text
PLND_ENABLED = 1
reboot Pixhawk
PLND_TYPE    = 1     # nhận MAVLink LANDING_TARGET
```

Còn phải kiểm tra/tune theo camera và airframe thực tế:

- `PLND_YAW_ALIGN`: chiều lắp camera so với đầu drone.
- `PLND_CAM_POS_X/Y/Z`: vị trí camera so với tâm thân drone.
- `PLND_XY_DIST_MAX`: sai lệch ngang tối đa cho phép hạ.
- `PLND_STRICT`: cách xử lý khi mất marker.
- `PLND_ALT_MIN`, `PLND_ALT_MAX`: vùng độ cao áp dụng logic mất target.
- `PLND_RET_MAX`, `PLND_RET_BEHAVE`: số lần và cách retry.
- `LAND_SPD_MS`: tốc độ hạ ở pha cuối.

Precision Landing cần attitude và horizontal position estimate ổn định. Bay
ngoài trời thường dùng GPS; bay trong nhà phải có nguồn vị trí ngang phù hợp như
optical flow/VIO/motion capture, không nên chỉ dựa vào camera marker.

## 5. Luồng xử lý chi tiết trên Raspberry Pi

### Bước 1: Camera tạo khung hình

Camera USB `/dev/video0` cung cấp hình 1280×720. Detector có thể xử lý ảnh thu
nhỏ để giảm tải, nhưng offset cuối cùng được quy đổi về kích thước output.

### Bước 2: Detector tìm marker

Chế độ hiện tại là `single`, chỉ ID 6 được xem là mục tiêu. Detector trả về các
thông tin chính:

- Có phát hiện đúng target hay không.
- ID marker.
- Tâm marker trong ảnh.
- `offset_x`, `offset_y` so với tâm ảnh.
- Kích thước marker trong ảnh.
- `quality` và chi tiết chất lượng.
- Có duplicate ID hoặc trường hợp ambiguous hay không.
- Pose/khoảng cách ước lượng nếu calibration hợp lệ.

Ví dụ:

```text
Khung hình:       1280 × 720
Tâm ảnh:          x=640, y=360
Tâm marker:       x=740, y=330
offset_x:         +100 pixel
offset_y:         +30 pixel theo quy ước detector
```

### Bước 3: Xác nhận tracking

Kết quả không được dùng ngay ở lần phát hiện đầu tiên. State machine đi qua:

```text
SEARCH → ACQUIRING → TRACKING
```

Với cấu hình hiện tại, cần 5 measurement hợp lệ liên tiếp mới chuyển sang
`TRACKING` và đặt `control_valid=true`.

Các trạng thái khác:

| Trạng thái | Ý nghĩa | Có gửi điều khiển không? |
|---|---|---:|
| `SEARCH` | Chưa thấy target hợp lệ | Không |
| `ACQUIRING` | Đang xác nhận target nhiều frame | Không |
| `TRACKING` | Đã khóa đúng target | Có, nếu qua toàn bộ gate |
| `LOST` | Target vừa bị mất | Không |
| `AMBIGUOUS` | Duplicate ID hoặc target thay đổi bất thường | Không |

### Bước 4: Ghi telemetry trung gian

Kết quả detector được ghi nguyên tử vào:

```text
/tmp/camera_landing_0.json
```

File này chứa offset, quality, trạng thái tracking, timestamp measurement và
các thông tin chẩn đoán. MAVLink bridge đọc snapshot này để tránh lấy dữ liệu
trực tiếp từ luồng camera.

### Bước 5: Safety gate trước MAVLink

Pi chỉ phát target khi đồng thời thỏa tất cả điều kiện:

```text
detected = true
hold = false
ambiguous = false
control_valid = true
quality >= 0.55
0 ms <= measurement age <= 300 ms
```

Nếu một điều kiện sai, Pi không gửi measurement đó. Cách làm này tránh gửi ảnh
cũ hoặc marker không chắc chắn cho flight controller.

### Bước 6: Đổi pixel thành góc

Bridge dùng FOV lấy từ calibration camera để đổi offset thành góc:

```text
offset_x > 0  → angle_x > 0
offset_x < 0  → angle_x < 0
marker ở tâm  → angle_x ≈ 0 và angle_y ≈ 0
```

Các góc được gửi bằng radian. Dấu trục thực tế vẫn phải kiểm tra khi camera đã
lắp lên drone; không được suy đoán chỉ từ hình overlay.

## 6. Pi gửi gì trong LANDING_TARGET?

Code hiện tại gửi **metric position** trong hệ thân drone:

| Field MAVLink | Giá trị | Ý nghĩa |
|---|---:|---|
| `time_usec` | Thời gian monotonic của Pi | Timestamp bản tin |
| `target_num` | `0` | Mục tiêu số 0 |
| `frame` | `MAV_FRAME_BODY_FRD` | Hệ trục thân: trước, phải, xuống |
| `angle_x` | Góc radian | Sai lệch mục tiêu theo trục ngang |
| `angle_y` | Góc radian | Sai lệch mục tiêu theo trục còn lại |
| `distance` | Khoảng cách pose (m) | Chuẩn vector target, bắt buộc dương |
| `size_x`, `size_y` | Kích thước góc | Kích thước marker nhìn thấy |
| `x`, `y`, `z` | Tọa độ mét | Target trong hệ thân forward/right/down |
| `type` | Vision fiducial | Mục tiêu camera |
| `position_valid` | `1` | Pixhawk sử dụng position XYZ theo mét |

Có thể hiểu bản tin như câu:

```text
“Marker đang ở trước/phải/dưới camera bao nhiêu mét trong hệ BODY_FRD.”
```

Pi hiện không gửi trong bản tin này:

- Lệnh arm/disarm.
- Lệnh chuyển mode LAND hoặc RTL.
- Roll, pitch, throttle hay tốc độ motor.
- GPS của marker.
- Lệnh độ cao/tốc độ hạ.

Bridge dùng pose/khoảng cách từ calibration để gửi `x/y/z` và `distance`. Phép
đổi từ OpenCV sang thân drone, với camera hướng xuống và mép trên ảnh hướng về
đầu drone, là `x=-camera_y`, `y=camera_x`, `z=camera_z`. Nếu pose thiếu, không
hữu hạn, nằm sau camera hoặc ngoài 0,05–30 m thì bridge ngừng gửi target.

`z` này là khoảng cách quang học ước lượng từ kích thước marker, không phải phép
đo độ cao độc lập. Sai kích thước marker, focus, calibration hoặc marker không
phẳng sẽ làm sai toàn bộ tỷ lệ XYZ, nên phải kiểm tra bằng thước ở nhiều khoảng
cách trước khi cho Pixhawk sử dụng.

## 7. Pixhawk sử dụng bản tin như thế nào?

Khi Pixhawk đang ở mode phù hợp, ArduCopter nhận `LANDING_TARGET` rồi kết hợp:

```text
target vector x/y/z từ Pi
+ attitude của drone
+ vị trí ngang/EKF
+ độ cao từ barometer/rangefinder nếu hệ có cấu hình
+ cấu hình PLND_*
= mục tiêu điều khiển ngang và hành vi hạ cánh
```

Pixhawk không chỉ “bay theo camera”. Nó vẫn dùng toàn bộ bộ điều khiển bay,
estimator, failsafe và land detector của ArduCopter.

### Mode LAND

Đây là mode nên dùng để thử Precision Landing trực tiếp:

```text
Pilot đưa drone tới phía trên pad
→ camera đã TRACKING
→ pilot chuyển LAND
→ Pixhawk chỉnh ngang theo LANDING_TARGET
→ Pixhawk điều khiển hạ độ cao
→ land detector xác nhận chạm đất
→ Pixhawk disarm
```

### Mode RTL

Trong RTL, Pixhawk tự bay về khu vực Home. Precision Landing chỉ hỗ trợ pha hạ
cuối nếu target được nhìn thấy. Nên thử RTL sau khi LAND trực tiếp đã ổn định.

### Precision Loiter

Precision Loiter dùng cùng target để thử căn ngang nhưng giữ độ cao. Đây là bước
an toàn nên làm trước LAND vì pilot có thể quan sát drone có sửa đúng hướng hay
không mà chưa cho hạ tự động.

### Mode AUTO

AUTO không tự động có nghĩa là đang hạ bằng camera. Mission phải đi đến lệnh
LAND hoặc bước vào RTL/final landing thì Precision Landing mới tham gia pha hạ.

## 8. Điều gì xảy ra khi mất marker?

Luồng trên Pi:

```text
Mất marker / quality thấp / duplicate / camera treo
→ control_valid=false hoặc measurement stale
→ Pi ngừng gửi LANDING_TARGET mới
```

Sau đó hành vi của drone do Pixhawk quyết định, không phải Pi:

- `PLND_STRICT=0`: tiếp tục LAND bình thường.
- `PLND_STRICT=1`: retry theo cấu hình, hết retry thì land bình thường.
- `PLND_STRICT=2`: retry theo cấu hình, hết retry thì hover.
- Mất target dưới `PLND_ALT_MIN`: có thể tiếp tục hạ thẳng.

Vì vậy “Pi ngừng gửi” không đồng nghĩa với “drone tự bay lên” hoặc “tự hủy hạ
cánh”. Phải test chính sách mất marker trên đúng firmware và parameter của
Pixhawk.

## 9. Ví dụ một lần hạ cánh thành công

```text
1. Drone đang LOITER phía trên marker ID 6.
2. Camera nhìn thấy marker nhưng mới có 1 frame:
   ACQUIRING, chưa gửi target.
3. Đủ 5 measurement tốt:
   TRACKING, control_valid=true.
4. Bridge bắt đầu gửi LANDING_TARGET khoảng 10 Hz.
5. Pilot chuyển Pixhawk sang LAND.
6. Marker lệch phải:
   Pi gửi `y` BODY_FRD dương, nghĩa là target ở bên phải drone.
7. ArduCopter điều khiển drone giảm sai lệch ngang.
8. Marker tiến gần tâm:
   `x` và `y` tiến về 0, `z` là khoảng cách quang học từ camera tới marker.
9. Pixhawk điều khiển tốc độ hạ bằng land controller và altitude estimator.
10. Land detector xác nhận chạm đất; Pixhawk tự disarm.
```

## 10. Ví dụ một lần mất marker

```text
1. Drone đang LAND và target đang TRACKING.
2. Marker bị người hoặc vật che.
3. Detector chuyển LOST và control_valid=false.
4. Pi ngừng gửi LANDING_TARGET mới trong giới hạn age gate 300 ms.
5. Pixhawk phát hiện mất target.
6. Pixhawk retry, land thường hoặc hover tùy PLND_STRICT và độ cao.
7. Pilot phải sẵn sàng chuyển mode để abort nếu hành vi không an toàn.
```

## 11. Trình tự test bắt buộc

Không thử bay thật ngay sau khi bật cấu hình. Nên đi theo thứ tự:

```text
Unit test
→ replay video thật
→ ArduCopter SITL
→ bench Pixhawk tháo cánh
→ Precision Loiter thấp
→ LAND thấp có pilot override
→ tăng dần độ cao và điều kiện khó
→ cuối cùng mới thử RTL
```

### Test phần mềm

```bash
venv/bin/python -m unittest discover -v tests
venv/bin/python tools/landing_preflight_check.py
```

### Test live camera

Sau khi dừng streamer để tránh chiếm camera:

```bash
venv/bin/python tools/landing_preflight_check.py \
  --require-live --probe-camera
```

### Những tình huống phải thử khi tháo cánh

1. Marker ở tâm, trái, phải, trước và sau.
2. Xoay thân drone 90°, 180° và 270°.
3. Che marker.
4. Đưa hai marker trùng ID vào ảnh.
5. Rút camera USB.
6. Ngắt MAVLink UART.
7. Tạo tải CPU làm telemetry bị trễ.
8. Kiểm tra mode switch để thoát LAND.
9. Kiểm tra radio, battery và EKF failsafe.
10. Kiểm tra pose `x/y/z`, altitude source của Pixhawk và DataFlash log.

## 12. Kết luận trạng thái hiện tại

Phần mềm hiện đã có detector, tracking state machine, stale-data gate và bridge
`LANDING_TARGET`. Unit test offline đã pass, nhưng cấu hình đang để
`mavlink_enabled: false` và môi trường kiểm tra chưa có camera/Pixhawk/SITL live.

Do đó trạng thái đúng là:

```text
Sẵn sàng cho SITL và bench tháo cánh.
Chưa đủ bằng chứng để xác nhận sẵn sàng bay tự động hạ cánh ngoài thực tế.
```

Các file chính liên quan:

- `config.yaml`: cấu hình camera, detector và MAVLink landing.
- `Find_landing/processing/detectors/aruco/processor.py`: xử lý ArUco.
- `Find_landing/processing/detectors/aruco/track_state.py`: state machine tracking.
- `Find_landing/stream/metrics.py`: ghi telemetry landing trung gian.
- `landing_mavlink.py`: safety gate, đổi pixel thành góc và gửi MAVLink.
- `tools/landing_preflight_check.py`: kiểm tra trước khi test.
- `tools/landing_target_sitl.py`: tạo tình huống `LANDING_TARGET` cho SITL.
