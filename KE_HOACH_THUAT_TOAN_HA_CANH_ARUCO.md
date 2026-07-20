# Phân tích và kế hoạch hệ thống hạ cánh bằng camera ArUco

Tài liệu này mô tả đúng hiện trạng code trong dự án, các tình huống có thể gặp
khi bay, thuật toán tìm mục tiêu/bãi đáp, thuật toán điều khiển hạ cánh, cách xử
lý nhiều marker, kế hoạch phát triển và quy trình test từ lý thuyết đến bay thật.

Phạm vi giả định là multirotor chạy **ArduCopter** trên Pixhawk 2.4.8, camera USB
hướng xuống và Raspberry Pi/CM5 làm companion computer. Nếu firmware thực tế là
ArduPlane cho máy bay cánh bằng thì không áp dụng trực tiếp state machine hạ cánh
thẳng đứng trong tài liệu này; cần thiết kế lại đường glide, flare và điều khiển
theo airspeed.

> Đây là tài liệu thiết kế, không phải xác nhận hệ thống đã an toàn để bay tự
> động. Cấu hình hiện tại vẫn để `landing.mavlink_enabled: false`; camera đã trỏ
> tới file calibration để ước lượng pose, nhưng vẫn cần kiểm thử thực tế trước
> khi bật phát `LANDING_TARGET`.

## 1. Kết luận ngắn gọn trước

### 1.1 Thuật toán hiện tại thực sự làm gì?

Hệ thống hiện tại gồm hai bên:

1. **Pi/CM5**:
   - Chụp webcam USB 1280×720.
   - Resize ảnh nhận diện xuống 320×240.
   - Dùng OpenCV ArUco `DICT_4X4_50`.
   - Chỉ chọn marker ID 6 làm mục tiêu theo `config.yaml`.
   - Làm mượt tâm marker bằng EMA.
   - Tính pose/khoảng cách từ calibration camera.
   - Đổi pose OpenCV sang BODY_FRD `x/y/z`.
   - Gửi MAVLink `LANDING_TARGET` 10 Hz với `position_valid=1` khi bridge được bật.

2. **Pixhawk/ArduCopter**:
   - Nhận `LANDING_TARGET`.
   - Ước lượng vị trí tương đối mục tiêu bằng backend Precision Landing.
   - Dùng position controller ngang và land controller dọc của ArduCopter.
   - Xử lý mất mục tiêu, retry, tiếp tục đáp hoặc giữ vị trí theo các tham số
     `PLND_*`.

Pi hiện tại **không tự gửi roll/pitch/throttle**, không tự chạy PID bay và không
tự chuyển mode LAND. Server/pilot/mission đưa drone về LAND hoặc RTL; ArduCopter
mới điều khiển motor.

### 1.2 “Tìm bãi đáp” hiện tại có nghĩa gì?

Code hiện tại chỉ tìm **bãi đáp đã được đánh dấu bằng đúng ArUco ID 6** theo
`config.yaml`. Nó chưa:

- Phân loại mặt đất có bằng phẳng hay không.
- Phát hiện người, cây, dây điện, xe hoặc vật cản trên bãi.
- Đo độ dốc/độ nhám bề mặt.
- Chọn một vùng đất tự nhiên không có marker.
- Xác minh bãi có đủ lớn cho kích thước drone.

Vì vậy tên chính xác là **marker-based precision landing**, không phải
**autonomous safe landing-zone detection**. Nếu muốn tìm một bãi trống bất kỳ,
đó là module khác cần depth/lidar/stereo hoặc monocular depth, semantic
segmentation và kiểm tra hình học bề mặt.

### 1.3 Kiến trúc khuyến nghị

Trong giai đoạn đầu, dùng kiến trúc:

```text
Camera → ArUco/board pose → lọc + quality gate → LANDING_TARGET
                                                  ↓
                                    ArduCopter Precision Landing
                                                  ↓
                              position/attitude/motor controllers
```

Không tạo thêm một PID vận tốc trên Pi khi ArduCopter Precision Landing đang
hoạt động. Hai vòng điều khiển độc lập cùng sửa X/Y có thể gây dao động hoặc
lệnh đối nghịch.

## 2. Luồng xử lý hiện tại trong code

### 2.1 Capture và stream

`config.yaml` đang cấu hình CAM0:

```yaml
source: usb
device_path: /dev/video0
usb_input_format: mjpeg
usb_direct_mode: false
size: [1280, 720]
framerate: 30
bitrate: 2500
detection_enabled: true
overlay_enabled: true
overlay_burn_enabled: true
lores_size: [320, 240]
detect_frame_skip: 2
overlay_frame_skip: 2
landing_detection_mode: aruco
aruco_dictionary: DICT_4X4_50
aruco_marker_id: 5
```

Pipeline dùng queue kích thước 1. Khi xử lý chậm, frame cũ bị bỏ và worker lấy
frame mới nhất. Đây là lựa chọn phù hợp cho điều khiển realtime vì giảm backlog,
nhưng cần đo latency thực tế thay vì chỉ nhìn FPS stream.

Với `detect_frame_skip=2`, detector danh nghĩa chạy khoảng 15 lần/giây khi camera
đạt 30 fps. Khi vừa mất target, code tạm detect mỗi frame trong 2,5 giây để bắt
lại nhanh.

### 2.2 Detector ArUco hiện tại

Trình tự trong `Find_landing/processing/detectors/aruco/detect.py`:

1. Resize frame sang `detect_size`, mặc định 320×240.
2. Đổi BGR → grayscale.
3. Gọi `ArucoDetector.detectMarkers()`.
4. Chỉ giữ ID trong khoảng 0–11.
5. Tìm đúng `aruco_marker_id`, hiện là ID 6.
6. Lấy trung bình bốn corner làm tâm.
7. Scale tâm/corner từ 320×240 về hệ tọa độ 1280×720.
8. Tính:

```text
offset_x = marker_center_x - image_center_x
offset_y = image_center_y - marker_center_y
```

Do đó:

- Marker bên phải ảnh → `offset_x > 0`.
- Marker phía trên ảnh → `offset_y > 0`.

`similarity` hiện bị gán cố định `0.99`; đây chưa phải confidence đo từ detector.
Code cũng chưa tính reprojection error, pose covariance hoặc Hamming margin.

### 2.3 Làm mượt và xử lý mất marker

`SmoothTracker` hiện dùng:

- EMA với `alpha=0.28` cho tâm, kích thước và corner.
- Giới hạn bước nhảy mỗi update bằng 14% kích thước frame lớn nhất.
- Hysteresis 20/15 pixel cho nhãn hướng.
- Giữ target tối đa 1500 ms sau khi mất (`hold=true`).
- Reset tracker sau khi hết hold.

Bridge MAVLink cố ý không gửi `LANDING_TARGET` khi `hold=true`. Như vậy overlay
có thể giữ target cũ để người xem dễ quan sát, nhưng Pixhawk không nhận một vị
trí giả đã cũ.

Lưu ý: nhãn chữ `UP/DOWN` trên overlay hiện mang tính hướng dẫn hình ảnh và có
thể gây hiểu nhầm theo hệ trục drone. Không được dùng nhãn này làm lệnh bay. Dấu
góc MAVLink phải được kiểm tra bằng test hệ tọa độ riêng.

### 2.4 Chuyển pixel sang LANDING_TARGET

Với FOV ngang `FOVx`, chiều rộng ảnh `W` và offset ngang `u`, code tính:

```text
angle_x = atan((2u/W) × tan(FOVx/2))
```

Tương tự với trục dọc. Vì detector định nghĩa Y dương hướng lên nhưng tọa độ ảnh
dương hướng xuống, code đảo dấu trước khi tạo `angle_y`.

Message hiện dùng:

```text
frame          = MAV_FRAME_BODY_FRD
angle_x/y      = góc radian
distance       = chuẩn Euclid của target XYZ metric
x/y/z          = target trong body frame: forward/right/down
position_valid = 1
```

ArduPilot yêu cầu `LANDING_TARGET` ít nhất 1 Hz; dự án phát 10 Hz khi target mới,
hợp lệ và không phải hold.

### 2.5 Thuật toán bay nằm ở ArduCopter

Khi vào LAND/pha cuối RTL và Precision Landing được bật, ArduCopter:

- Kiểm tra target có được acquire hay không.
- Lấy vị trí mục tiêu tương đối trong hệ NED/body.
- Đưa target position/velocity vào position controller ngang.
- Dùng controller dọc để giảm độ cao.
- Có state machine retry/failsafe khi mất target.
- Có thể giảm tốc độ hạ khi gần mặt đất và sai số mục tiêu còn lớn.
- Cho phép pilot hủy LAND hoặc reposition tùy cấu hình.

Đây không phải một PID đơn giản trong file Python. Nó là chuỗi estimator,
precision-landing state machine, position controller, attitude controller và
motor mixer trong firmware ArduCopter.

## 3. Điều gì xảy ra khi thấy nhiều ArUco?

Phải phân biệt ba trường hợp hoàn toàn khác nhau.

### 3.1 Nhiều ID khác nhau thuộc cùng một bảng

Ví dụ camera thấy ID `[2, 6, 7, 8]` trên tấm board 0–11.

Hành vi hiện tại:

- Overlay vẽ tất cả marker nhìn thấy.
- Khi chạy `single`, chỉ ID 6 được dùng làm tâm điều khiển.
- Khi chạy `board`, các ID hợp lệ được hợp nhất để ước lượng tâm/pose board.

Hệ quả: với cấu hình hiện tại `single`, nếu ID 6 bị che dù các ID khác vẫn rõ,
Pi ngừng gửi `LANDING_TARGET`. Muốn tận dụng nhiều ID thì phải chuyển
`aruco_target_strategy: board` và test lại layout/thước đo.

Giải pháp đề xuất:

- Khai báo tọa độ vật lý của từng ID trên board, đơn vị mét.
- Dùng tất cả corner hợp lệ của các ID duy nhất.
- `Board.matchImagePoints()` + `solvePnP()`/IPPE hoặc `estimatePoseBoard()`.
- Dùng RANSAC/reprojection error để loại corner/marker sai.
- Tâm đáp là origin cố định của board, không phải tâm của marker bất kỳ.
- Cho phép pose còn hợp lệ khi một phần board bị che.

### 3.2 Nhiều bãi đáp khác nhau, mỗi bãi một ID

Ví dụ trong sân có pad ID 5, ID 6 và ID 7.

Quy tắc an toàn khuyến nghị:

- Mission/server phải chỉ định `target_pad_id` trước khi LAND.
- Pi chỉ khóa đúng ID được giao.
- Không tự chọn “marker gần tâm nhất” nếu chưa có chính sách nhiệm vụ.
- Sau khi acquire, giữ `locked_pad_id` cho đến khi abort hoặc quay lại SEARCH.
- Không đổi bãi giữa lúc đang descend chỉ vì marker khác lớn hơn/rõ hơn.

Nếu muốn tự chọn bãi, score phải gồm vùng an toàn, mission priority, pose quality,
khoảng cách, khả năng tiếp cận và obstacle clearance; không chỉ dựa trên kích
thước marker.

### 3.3 Nhiều bản sao cùng một ID

Ví dụ có hai marker ID 5 ở hai vị trí khác nhau.

Hành vi hiện tại:

- `_pick_landing()` lấy phần tử ID 5 đầu tiên do OpenCV trả về.
- Thứ tự trả về không phải cam kết ổn định.
- Frame sau có thể chọn bản sao còn lại.
- Tracker giới hạn bước nhảy nhưng vẫn có nguy cơ drift hoặc đổi target.
- Dictionary `markers_by_id` còn làm mất thông tin một trong các bản sao khi
  dùng ID làm key.

Đây là tình huống **không an toàn**. Chính sách đề xuất:

1. Cấm ID trùng trong cùng vùng bay, hoặc
2. Nếu phát hiện hai instance của `target_pad_id`, đặt trạng thái `AMBIGUOUS`,
   ngừng phát target và HOLD/ABORT, hoặc
3. Chỉ tiếp tục instance gần pose dự đoán cũ nếu đã target-lock và khoảng cách
   Mahalanobis/reprojection nằm trong gate; không được chọn lại ngẫu nhiên.

Khuyến nghị giai đoạn đầu là cấm trùng ID và abort khi ambiguous.

### 3.4 Marker đa kích thước cho nhiều độ cao

Một marker duy nhất thường quá nhỏ ở cao độ lớn và tràn khung hình khi xuống rất
thấp. Có thể dùng bộ marker nhiều kích thước với ID khác nhau nhưng cùng biểu diễn
một origin đáp:

- Marker lớn để acquire ở cao độ lớn.
- Board trung bình để align/descend.
- Marker nhỏ gần tâm để final approach.

Việc chuyển scale phải có hysteresis theo độ cao/diện tích ảnh và tất cả marker
phải có transform vật lý về cùng origin. Không được đổi tâm đơn giản theo marker
đang lớn nhất vì sẽ tạo bước nhảy vị trí.

## 4. Các trường hợp có thể gặp khi bay

### 4.1 Bảng tình huống tổng hợp

| Tình huống | Nguy cơ | Hành vi hiện tại | Hành vi cần có |
|---|---|---|---|
| Đúng ID 6, rõ, gần tâm | Bình thường | Gửi `x/y/z` 10 Hz | Track, align và cho ArduCopter descend |
| Không thấy marker | Mất tham chiếu | Không gửi target | ArduCopter xử lý theo `PLND_STRICT/ALT_*`; Pi báo LOST |
| Chỉ thấy ID khác | Đáp nhầm nếu chọn sai | Không gửi ID khác | Giữ SEARCH cho ID nhiệm vụ |
| Nhiều ID trên cùng board | Bỏ phí thông tin nếu đang ở `single` | Chỉ dùng ID 6 | Fuse board pose từ mọi ID hợp lệ |
| Hai ID 6 khác vị trí | Nhảy mục tiêu | AMBIGUOUS, không gửi | AMBIGUOUS → ngừng target/HOLD/ABORT |
| Marker bị che một phần | Corner sai/mất ID | Có thể mất target | Board fusion + reprojection gate |
| Marker sát mép ảnh | Pose sai, sắp mất | Vẫn có thể gửi | Giảm tốc ngang, không descend nếu quality thấp |
| Marker quá nhỏ | Decode sai/không thấy | SEARCH | Marker lớn hơn hoặc approach theo GPS/optical flow |
| Marker quá lớn/tràn ảnh | Mất bốn corner | Mất target gần đất | Marker đa scale, final logic và rangefinder |
| Motion blur | Corner rung/mất | EMA, có thể hold overlay | Tăng shutter/light, quality gate, pause descent |
| Thiếu sáng | False negative | SEARCH/reacquire | Đèn chủ động, exposure lock, test lux |
| Chói/nắng/gương | False corner | Chưa có quality thật | Matte print, polarizer nếu cần, reprojection gate |
| Bóng cánh quạt thay đổi | ID chập chờn | Hold overlay | Temporal confirmation + lighting design |
| Bụi/nước che marker | Mất target cuối pha | Ngừng target | Rangefinder + PLND lost-target policy, abort nếu còn cao |
| Camera rung | Offset dao động | EMA alpha 0.28 | Cơ khí chống rung + timestamped filter |
| Camera lệch yaw | Sửa sai hướng | Phụ thuộc `PLND_YAW_ALIGN` | Hiệu chuẩn extrinsic, test dấu X/Y |
| Camera không thẳng xuống | Bias X/Y theo độ cao | Không bù đầy đủ | Camera-to-body transform 6DoF |
| Lens distortion | Sai góc ở rìa | FOV pinhole đơn giản | Intrinsic calibration + undistort/PnP |
| Crop/resolution đổi | FOV không còn đúng | Có thể gửi sai góc | Calibration profile riêng cho 1280×720 |
| FPS CV giảm | Target stale | Queue bỏ frame cũ | Giám sát age/latency, ngừng gửi khi quá hạn |
| Encode video nghẽn | Điều khiển bị trễ nếu chung tài nguyên | Queue leaky | Ưu tiên CV/telemetry hơn overlay/stream |
| USB camera rút | Mất toàn bộ vision | Reconnect | Báo CAMERA_LOST, ArduCopter fallback/abort |
| Pi treo/reboot | Mất LANDING_TARGET | Pixhawk mất stream | Watchdog và policy mất target độc lập |
| Mất Internet/server | Mất giám sát remote | Local Pi↔Pixhawk vẫn có thể chạy | Landing local không phụ thuộc Internet |
| Mất UART Pi↔Pixhawk | Không có correction | Không gửi được | Pixhawk fallback theo PLND policy |
| Rangefinder mất/sai | Sai khoảng cách mặt đất | Pi không kiểm soát | Health gate; không precision land nếu range invalid |
| EKF/GPS/optical flow lỗi | Không giữ được vị trí ngang | ArduPilot có thể từ chối/dao động | Không arm/LAND precision khi estimator unhealthy |
| Gió ngang mạnh | Trôi khỏi marker | ArduPilot sửa nhưng có giới hạn | Giới hạn điều kiện gió, pause descend, abort |
| Ground effect | Dao động cuối pha | ArduCopter land controller xử lý phần nào | Tune LAND speed, test tải thật |
| Pad nghiêng/mềm/trơn | Lật sau touchdown | Không phát hiện | Pose normal/slope + site design |
| Người/vật đi vào pad | Va chạm | ArUco vẫn hợp lệ | Obstacle/person detector độc lập, inhibit descent |
| Pad đang di chuyển | Sai giả định target tĩnh | Mặc định tĩnh | Chỉ bật moving-target option khi có estimator vận tốc |
| Pilot reposition | Hai nguồn lệnh | ArduPilot có policy | Pilot luôn có quyền override rõ ràng |
| Battery failsafe | LAND gấp | Autopilot ưu tiên failsafe | Định nghĩa ưu tiên failsafe, không chờ marker vô hạn |
| RTL tới sai Home | Marker ngoài FOV | SEARCH khi đang descend | Approach waypoint/loiter trước khi bật final landing |

### 4.2 Mất target theo độ cao

Không nên dùng cùng một phản ứng ở mọi độ cao:

- **Cao**: dừng descend, giữ vị trí hoặc search trong vùng giới hạn.
- **Trung bình**: giữ/leo nhẹ và reacquire nếu pin đủ.
- **Thấp nhưng chưa chạm đất**: không thực hiện search ngang mạnh; giữ hoặc đáp
  thẳng tùy `PLND_ALT_MIN`, bãi đã xác minh và risk policy.
- **Rất gần touchdown**: dựa vào land detector/rangefinder; không đuổi theo một
  detection mới nhảy xa.

ArduCopter có `PLND_ALT_MAX`, `PLND_ALT_MIN`, `PLND_STRICT`, `PLND_RET_MAX` và
retry behavior để cấu hình các phản ứng này. Cần tune trong SITL trước.

## 5. Thuật toán tìm mục tiêu đề xuất

### 5.1 Mục tiêu thiết kế giai đoạn 1

Chỉ giải bài toán bãi đáp chuyên dụng có marker/board đã biết:

```text
Input: frame, timestamp, camera calibration, board geometry, target_pad_id
Output: target pose/angle + quality + state
```

### 5.2 Các bước xử lý

1. **Capture có timestamp monotonic**
   - Timestamp ngay khi frame được nhận.
   - Không dùng thời điểm sau encode làm thời điểm measurement.

2. **Camera calibration**
   - Hiệu chuẩn đúng webcam, focus và mode 1280×720.
   - Lưu ma trận intrinsic `K` và distortion `D`.
   - Không tái sử dụng calibration của 640×480 nếu camera crop khác.

3. **Detect ArUco**
   - Detect tất cả ID trong whitelist.
   - Giữ mọi instance, không collapse bằng dictionary trước bước ambiguity.
   - Corner refinement nếu CPU đáp ứng.

4. **Xác thực identity**
   - Mission pad ID/board ID phải khớp.
   - Reject ID ngoài whitelist.
   - Reject duplicate target ID nếu chưa có target-lock hợp lệ.

5. **Pose estimation**
   - Marker đơn: dùng bốn corner và kích thước marker thật.
   - Board: map corner 2D ↔ point 3D của toàn board.
   - Dùng `solvePnP` với phương pháp planar phù hợp như IPPE/IPPE_SQUARE.
   - Với board nhiều corner, có thể dùng RANSAC + refine.

6. **Quality score thực**
   - Số marker/corner hợp lệ.
   - Reprojection RMSE.
   - Diện tích marker trong ảnh.
   - Khoảng cách corner tới mép ảnh.
   - Pose jump so với prediction.
   - Detection age và consecutive confirmations.
   - Không dùng constant `similarity=0.99`.

7. **Transform camera → body**
   - Đo translation và rotation camera so với thân drone.
   - Chuyển target sang `MAV_FRAME_BODY_FRD`: X forward, Y right, Z down.
   - Bù `PLND_CAM_POS_X/Y/Z` hoặc thực hiện transform nhất quán, không bù hai lần.

8. **Temporal estimator**
   - State tối thiểu `[x, y, vx, vy]` tương đối.
   - Prediction theo timestamp, update bằng measurement covariance từ quality.
   - Outlier gate bằng innovation/Mahalanobis distance.
   - Target-lock không đổi ID/instance tùy tiện.

9. **Quality gate trước MAVLink**
   - Chỉ phát khi measurement/prediction còn mới.
   - Có minimum consecutive frames trước ACQUIRED.
   - Không phát pose nếu ambiguous hoặc reprojection quá lớn.

### 5.3 Mô hình camera

Mô hình pinhole đầy đủ:

```text
s [u v 1]ᵀ = K [R | t] [X Y Z 1]ᵀ
```

Trong đó `K` chứa `fx, fy, cx, cy`; distortion được xử lý bằng hệ số camera đã
calibrate. FOV chỉ là phương án gần đúng. PnP với calibration cho phép có
translation theo mét và orientation của pad, hữu ích hơn chỉ góc pixel.

### 5.4 Khi nào dùng angle, khi nào dùng XYZ?

- **Giai đoạn hiện tại**: gửi `x/y/z`, `distance` trong body frame và
  `position_valid=1`.
- Bridge đổi pose OpenCV `(right, down, optical-forward)` thành BODY_FRD
  `(-camera_y, camera_x, camera_z)` và fail-closed nếu pose không hợp lệ.
- Pose PnP vẫn phải được kiểm tra scale, ambiguity planar, trục và covariance
  bằng ground truth trước khi bay thật.

## 6. State machine đề xuất

Pi nên có state machine **validity/tracking**, còn ArduCopter giữ state machine
bay. Không để Pi tự điều khiển motor.

```text
DISABLED
   ↓ enable + precheck pass
PRECHECK
   ↓ vehicle approaching landing zone
SEARCH
   ↓ N detections valid
ACQUIRE_CONFIRM
   ↓ identity + pose + quality pass
TRACK_ALIGN
   ↓ horizontal error within altitude-dependent gate
DESCEND_ALLOWED
   ↓ low altitude + stable target/range
FINAL
   ↓ ArduPilot land detector
TOUCHDOWN / COMPLETE

Từ TRACK/DESCEND/FINAL:
  target stale → LOST/RECOVER
  ambiguous/bad sensor/obstacle → INHIBIT or ABORT
```

### 6.1 DISABLED

- Không gửi `LANDING_TARGET`.
- Chỉ stream/overlay nếu cần.
- Trạng thái mặc định sau boot.

### 6.2 PRECHECK

Điều kiện tối thiểu:

- Camera đúng device và calibration profile.
- CV update rate/latency đạt ngưỡng.
- Pi↔Pixhawk MAVLink hoạt động.
- Rangefinder healthy.
- EKF/attitude/horizontal position healthy.
- `PLND_ENABLED=1`, `PLND_TYPE=1`.
- Marker geometry, FOV/intrinsic và extrinsic không bằng 0.
- Không có duplicate target trong ảnh test ban đầu.

Fail bất kỳ điều kiện nào → không cho server kích hoạt auto precision landing.

### 6.3 SEARCH

Khuyến nghị giai đoạn đầu:

- ArduPilot/GPS/optical flow đưa drone tới waypoint/loiter trên pad.
- Pi chỉ tìm marker trong FOV.
- Giữ độ cao search cố định và geofence nhỏ.
- Có timeout và battery budget.
- Chưa cho descend khi chưa acquire đủ N frame.

Không nên ngay lập tức viết spiral search bằng lệnh velocity từ Pi. Nếu cần search
chủ động sau này, dùng expanding square/yaw scan có giới hạn tốc độ, vùng bay,
obstacle sensing và đường abort rõ ràng.

### 6.4 ACQUIRE_CONFIRM

Ví dụ tiêu chí khởi đầu để test, chưa phải giá trị bay cuối:

- Ít nhất 5 detection hợp lệ liên tiếp.
- Thời gian measurement < 200 ms.
- Không có duplicate target.
- Reprojection RMSE < 2–3 pixel tùy vùng ảnh.
- Marker không cắt mép ảnh.
- Pose jump nằm trong motion gate.

### 6.5 TRACK_ALIGN

- Phát target liên tục cho ArduCopter.
- Chưa descend nhanh nếu sai số ngang lớn.
- Gate sai số nên phụ thuộc độ cao:

```text
allowed_xy_error(z) = clamp(k × z, error_min, error_max)
```

Ở cao có thể chấp nhận sai số lớn hơn; càng gần đất phải siết chặt.

### 6.6 DESCEND_ALLOWED

- Target locked và quality ổn định.
- Rangefinder valid.
- Horizontal error dưới gate trong một dwell time.
- Không có obstacle/person inhibit.
- ArduCopter điều khiển X/Y và tốc độ hạ.

Nếu sai số tăng, ưu tiên pause/giảm tốc hạ thay vì tiếp tục xuống.

### 6.7 LOST/RECOVER

- Dừng gửi measurement cũ.
- Phân biệt occlusion ngắn và mất hoàn toàn.
- ArduCopter áp dụng `PLND_STRICT` và retry.
- Pi tăng detect rate trong cửa sổ reacquire.
- Không đổi sang marker khác nếu chưa thoát target-lock.

### 6.8 FINAL và TOUCHDOWN

- Giới hạn chuyển động ngang đột ngột.
- Không tin marker size là bằng chứng touchdown.
- Touchdown/disarm dựa vào land detector của ArduCopter, rangefinder, vertical
  velocity và motor/thrust logic.
- Sau touchdown ngừng target và ghi log kết quả.

## 7. Nếu tự viết controller trên Pi thì thuật toán nào?

Chỉ xem đây là phương án giai đoạn nghiên cứu, không chạy đồng thời với native
Precision Landing.

### 7.1 Visual servoing/PID phân tầng

Outer loop biến sai số mục tiêu thành velocity setpoint:

```text
v_x_cmd = sat(Kp_x e_x + Ki_x ∫e_x dt + Kd_x de_x/dt)
v_y_cmd = sat(Kp_y e_y + Ki_y ∫e_y dt + Kd_y de_y/dt)
```

Vertical command chỉ âm khi horizontal error và quality đạt gate:

```text
v_z_cmd = descent_rate(z, quality, horizontal_error)
```

Inner-loop attitude/rate vẫn phải để Pixhawk xử lý. Cần anti-windup, saturation,
latency compensation, setpoint timeout và mode arbitration.

### 7.2 Vì sao chưa chọn MPC ngay?

MPC có thể xử lý ràng buộc vận tốc/gia tốc, moving pad và trajectory tốt hơn,
nhưng cần model, estimator và compute/tuning phức tạp. Với pad tĩnh và ArduCopter
đã có Precision Landing, ưu tiên dùng native controller, tập trung làm vision
measurement tin cậy trước. MPC chỉ đáng triển khai sau khi dataset và state
estimation đã đạt chuẩn.

## 8. Kế hoạch triển khai

### Phase 0 — đóng băng an toàn và logging

Mục tiêu:

- Giữ `mavlink_enabled=false` mặc định.
- Ghi log frame timestamp, detect timestamp, ID, corner, offset, pose, quality,
  telemetry age, vehicle mode, altitude và rangefinder.
- Hiển thị rõ `VISION_DISABLED/SEARCH/LOCKED/AMBIGUOUS/STALE` trên server.

Deliverable:

- Schema log JSONL/CSV.
- API status vision.
- Test không có message được phát khi disabled/stale/ambiguous.

### Phase 1 — hiệu chuẩn và hệ tọa độ

Mục tiêu:

- Viết tool ChArUco calibration cho mode 1280×720.
- Lưu `K`, `D`, reprojection error và serial/device identity.
- Khai báo camera-to-body extrinsic.
- Test dấu X/Y/Z và yaw trên bàn.

Deliverable:

- `camera_calibration_1280x720.yaml`.
- Báo cáo reprojection error.
- Unit test coordinate transforms.

### Phase 2 — detector quality và duplicate safety

Mục tiêu:

- Không collapse duplicate ID.
- Thêm target whitelist/mission ID.
- Thêm quality score thật.
- Thêm target-lock và trạng thái AMBIGUOUS.
- Loại `similarity=0.99` cố định.

Deliverable:

- Unit test nhiều ID, duplicate ID, wrong ID, occlusion.
- Dataset replay có metric precision/recall và false-lock.

### Phase 3 — board fusion/pose

Mục tiêu:

- Định nghĩa board geometry theo mét.
- Fuse corner của nhiều ID về một landing origin.
- PnP/IPPE + reprojection gating.
- Hỗ trợ partial occlusion.

Deliverable:

- Board config có physical size/gap/origin.
- Pose XYZ/RPY + covariance/quality.
- Test ground truth ở nhiều khoảng cách/góc.

### Phase 4 — estimator và validity state machine

Mục tiêu:

- Timestamp-aware alpha-beta/Kalman estimator.
- Innovation gate và latency prediction.
- State machine SEARCH/ACQUIRE/TRACK/LOST/AMBIGUOUS.
- Altitude-dependent quality/descent gate.

Deliverable:

- Không target jump khi đổi frame/occlusion.
- Không phát measurement cũ.
- Fault-injection tests đạt pass criteria.

### Phase 5 — tích hợp ArduCopter SITL

Mục tiêu:

- Gửi `LANDING_TARGET` thật từ bridge vào SITL với `PLND_TYPE=1`.
- Test native simulated precision landing với `PLND_TYPE=4` để có baseline.
- Mô phỏng rangefinder, wind, packet loss, marker loss và retry.

Deliverable:

- Script chạy repeatable scenarios.
- Log `PL`, vehicle trajectory và landing error.
- 100% test abort/failsafe đúng policy trước HIL.

### Phase 6 — bench/HIL

Mục tiêu:

- Pixhawk thật, Pi thật, webcam thật, tháo cánh/quạt.
- Di chuyển board dưới camera theo trajectory có đo ground truth.
- Rút USB, ngắt UART, giảm sáng, che marker, tạo duplicate.

Deliverable:

- Báo cáo latency p50/p95/p99.
- Bảng fault response.
- Không có false target output trong trường hợp ambiguous/stale.

### Phase 7 — flight envelope mở dần

Thứ tự:

1. Test trong lồng/tether, pilot giữ quyền override.
2. Hover thấp, chỉ quan sát measurement, chưa bật PLND.
3. Precision Loiter, không descend.
4. LAND từ 1 m trong điều kiện không gió.
5. LAND từ 2–3 m.
6. Test mất marker có chủ đích.
7. Test RTL final phase.
8. Chỉ sau đó mới tăng gió, ánh sáng khó hoặc moving pad.

Mỗi bước chỉ mở khi bước trước đạt acceptance gate.

## 9. Kế hoạch test chi tiết

### 9.1 Unit test toán học

| Test | Input | Kết quả mong đợi |
|---|---|---|
| Center | offset `(0,0)` | angle `(0,0)` |
| Right | `offset_x > 0` | `angle_x > 0` |
| Left | `offset_x < 0` | `angle_x < 0` |
| Image top | `offset_y > 0` | `angle_y < 0` theo mapping hiện tại |
| Invalid FOV | 0 hoặc ≥179° | Bridge từ chối chạy |
| Stale telemetry | age > limit | Không gửi MAVLink |
| Hold telemetry | `hold=true` | Không gửi MAVLink |
| Duplicate target | hai ID 5 | AMBIGUOUS, không gửi |
| Wrong ID | chỉ ID 4/6 | SEARCH, không gửi |
| Board partial | mất một số ID | Pose vẫn đúng nếu đủ corner |

### 9.2 Synthetic image test

Tạo ảnh marker/board với:

- Homography góc nhìn 0–60°.
- Scale từ rất nhỏ tới gần đầy frame.
- Blur, motion blur và Gaussian noise.
- Brightness/contrast/gamma khác nhau.
- Shadow, glare và partial occlusion 0–60%.
- Lens distortion mô phỏng.
- Một/nhiều ID và duplicate ID.

Metric:

- Detection precision/recall theo điều kiện.
- ID confusion matrix.
- Corner RMSE.
- Pose translation/rotation error.
- False-lock count; mục tiêu là 0 trong bộ safety test.

### 9.3 Recorded dataset replay

Ghi video thật ở 1280×720 từ webcam đang dùng:

- Nhiều độ cao/khoảng cách.
- Bay ngang qua pad.
- Roll/pitch/yaw khác nhau.
- Trong nhà/ngoài trời/sáng tối.
- Gió/rung/motion blur.
- Người hoặc vật che pad.
- Nhiều marker và duplicate.

Replay phải deterministic: cùng input cho cùng chuỗi state/target trong giới hạn
sai số định nghĩa.

### 9.4 Acceptance metric khởi đầu

Các ngưỡng sau là mục tiêu engineering ban đầu và phải tune theo camera/airframe:

- False target lock trong safety dataset: **0**.
- Target-ID switch khi đang locked: **0**, trừ khi explicit reset.
- CV update khi track: ≥10 Hz.
- Measurement age p95: <150–200 ms.
- End-to-end LANDING_TARGET age p95: <200 ms.
- Board reprojection RMSE: <2 px vùng trung tâm, <3 px vùng biên.
- Detect recall trong flight envelope đã công bố: ≥95%.
- Landing error ban đầu: median <15 cm, p95 <30 cm trong điều kiện chuẩn.
- Mọi fault critical phải dẫn tới HOLD/ABORT/fallback đúng thiết kế, không tiếp
  tục gửi target giả.

Không quảng bá độ chính xác centimet trước khi có số liệu flight-test lặp lại.

### 9.5 SITL baseline

Theo tài liệu ArduPilot, có thể test Precision Landing native:

```text
param set PLND_ENABLED 1
param fetch
param set PLND_TYPE 4
param set SIM_PLD_ENABLE 1
param set SIM_PLD_LAT -35.3632
param set SIM_PLD_LON 149.1652
```

SITL hiện yêu cầu simulated rangefinder cho test Precision Landing. Sau đó test
LAND và xem log `PL` bằng `mavlogdump.py --type PL`.

Để test bridge của dự án, dùng `PLND_TYPE=1` và gửi `LANDING_TARGET` từ một
scenario generator vào cổng MAVLink SITL. Cần tạo các scenario:

- Target tĩnh đúng tâm.
- Target offset step/ramp/sine.
- Noise và latency.
- Packet loss 10/30/50%.
- Dropout 0,5/1/2/5 giây.
- Duplicate/target jump.
- Rangefinder failure.
- Wind tăng dần.

### 9.6 Bench test phần cứng

Tháo cánh/quạt và giữ airframe cố định:

1. Đặt board ở tâm → angle gần 0.
2. Di chuyển board sang phải/trái/trước/sau → xác minh dấu.
3. Xoay airframe 90/180/270° → xác minh yaw alignment.
4. Nghiêng roll/pitch → kiểm tra transform camera-body.
5. Che lần lượt từng marker board.
6. Đặt hai ID 5 → phải AMBIGUOUS.
7. Rút/cắm webcam → CAMERA_LOST rồi recover có kiểm soát.
8. Ngắt UART → MAVLINK_LOST.
9. Tạo CPU load → measurement stale phải bị chặn.
10. Đo latency bằng timestamp/log, không ước lượng bằng mắt.

### 9.7 Flight test card cho mỗi chuyến

Trước bay:

- Firmware/version và parameter dump đã lưu.
- Prop/motor/RC/failsafe bình thường.
- Camera calibration đúng profile 1280×720.
- Webcam cố định, focus/exposure phù hợp.
- Rangefinder và EKF healthy.
- Marker đúng kích thước, matte, cố định và không trùng ID.
- Pilot biết switch abort/mode override.
- Geofence, vùng trống và người quan sát.
- Log Pi + DataFlash + video đồng bộ.

Sau bay:

- Landing error X/Y thực đo.
- Số lần target acquired/lost/retry.
- Latency distribution.
- Reprojection/quality distribution.
- PLND state và mode transitions.
- Pilot override/failsafe event.
- Không chỉ ghi “đáp thành công”; phải lưu cả near-miss và anomaly.

## 10. Tham số ArduPilot cần nghiên cứu/tune

Nhóm chính:

```text
PLND_ENABLED
PLND_TYPE
PLND_YAW_ALIGN
PLND_CAM_POS_X / Y / Z
PLND_LAND_OFS_X / Y
PLND_XY_DIST_MAX
PLND_STRICT
PLND_ALT_MIN
PLND_ALT_MAX
PLND_RET_MAX
PLND_RET_BEHAVE
PLND_OPTIONS
```

Nguyên tắc:

- Không copy nguyên parameter từ drone khác.
- Đọc đúng parameter docs của version ArduCopter đang cài.
- Tune trong SITL và test thấp trước.
- Bắt đầu với policy mất target bảo thủ, có pilot override.
- Moving pad chỉ bật option tương ứng khi estimator thực sự xuất target velocity
  ổn định; mặc định ArduPilot giả định target đứng yên.

## 11. Phân chia trách nhiệm Pi, Pixhawk và server

| Thành phần | Trách nhiệm |
|---|---|
| Pi/CM5 | Capture, calibration, detect, board pose, filter, quality gate, LANDING_TARGET, logging |
| Pixhawk/ArduCopter | EKF, rangefinder integration, Precision Landing state machine, position/attitude/rate/motor control, land detector, failsafe |
| Server/GCS | Chọn pad/mission, yêu cầu mode, giám sát, hiển thị trạng thái, lưu telemetry, cho phép pilot override |

Server mất kết nối không nên làm vòng local Pi→Pixhawk ngừng ngay nếu vehicle đã
vào một state an toàn được định nghĩa. Tuy nhiên server cũng không được tự coi
video overlay là bằng chứng duy nhất để arm hoặc LAND.

## 12. Các bài báo và tài liệu kỹ thuật nên đọc

### 12.1 ArUco và pose estimation

1. **Garrido-Jurado et al., “Automatic generation and detection of highly
   reliable fiducial markers under occlusion,” Pattern Recognition, 2014.**
   Nền tảng của ArUco: dictionary, inter-marker distance, detection và khả năng
   chịu che khuất.
   DOI: <https://doi.org/10.1016/j.patcog.2014.01.005>

2. **Collins and Bartoli, “Infinitesimal Plane-Based Pose Estimation,” IJCV,
   2014.** Phương pháp IPPE phù hợp cho mục tiêu phẳng và liên quan trực tiếp đến
   `SOLVEPNP_IPPE/IPPE_SQUARE` trong OpenCV.
   DOI: <https://doi.org/10.1007/s11263-014-0725-5>

3. **OpenCV ArUco Marker Detection and Pose Estimation.** Tài liệu chính thức về
   camera matrix, distortion và pose marker.
   <https://docs.opencv.org/master/d5/dae/tutorial_aruco_detection.html>

4. **OpenCV ArUco Board Detection.** Giải thích fuse nhiều marker trên board và
   khả năng dùng board khi bị che một phần.
   <https://docs.opencv.org/trunk/db/da9/tutorial_aruco_board_detection.html>

5. **OpenCV ChArUco Detection/Calibration.** Dùng để hiệu chuẩn camera và pose
   corner chính xác.
   <https://docs.opencv.org/master/df/d4a/tutorial_charuco_detection.html>

### 12.2 Visual servoing và lọc trạng thái

6. **Chaumette and Hutchinson, “Visual Servo Control, Part I: Basic
   Approaches,” IEEE Robotics & Automation Magazine, 2006.** Nền tảng IBVS/PBVS,
   interaction matrix và đóng vòng điều khiển bằng feature ảnh.
   DOI: <https://doi.org/10.1109/MRA.2006.250573>

7. **R. E. Kalman, “A New Approach to Linear Filtering and Prediction
   Problems,” 1960.** Nền tảng cho estimator prediction/update; chỉ nên dùng khi
   mô hình và covariance được định nghĩa/tune đúng.
   DOI: <https://doi.org/10.1115/1.3662552>

### 12.3 UAV landing bằng vision

8. **Wang et al., “Quadrotor Autonomous Landing on Moving Platform,” 2022.**
   Kết hợp ArUco pose, planner tránh vật cản và state machine; hữu ích để tham
   khảo kiến trúc tổng thể, dù dự án hiện tại ưu tiên pad tĩnh.
   <https://arxiv.org/abs/2208.05201>

9. **Lee et al., “A Vision-Based Control Method for Autonomous Landing of
   Vertical Flight Aircraft on a Moving Platform Without Using GPS,” 2020.**
   Monocular vision, gain-scheduled PID, simulation và flight test.
   <https://arxiv.org/abs/2008.05699>

10. **Paris, Lopez and How, “Dynamic Landing of an Autonomous Quadrotor on a
    Moving Platform in Turbulent Wind Conditions,” 2019.** Estimation, planning
    và robust control dưới gió; cho thấy moving-pad landing cần nhiều hơn một
    marker detector đơn giản.
    <https://arxiv.org/abs/1909.11071>

11. **“Artificial Marker and MEMS IMU-Based Pose Estimation Method to Meet
    Multirotor UAV Landing Requirements,” Sensors, 2019.** Nghiên cứu fusion
    marker + IMU và yêu cầu 6DoF cho landing.
    DOI: <https://doi.org/10.3390/s19245428>

12. **“A Precision Drone Landing System using Visual and IR Fiducial Markers
    and a Multi-Payload Camera,” 2024.** Tham khảo thiết kế robust qua điều kiện
    ánh sáng và cách tránh phụ thuộc quá nhiều vào pose marker.
    <https://arxiv.org/abs/2403.03806>

13. **“TDMBPLD: A Dataset Focusing on Marker Scene for UAV Landing,” IEEE
    Geoscience and Remote Sensing Letters, 2025.** Hữu ích cho cách xây dataset
    high/low resolution và đánh giá marker landing trong điều kiện thực tế.
    DOI: <https://doi.org/10.1109/LGRS.2025.3567863>

### 12.4 ArduPilot và MAVLink chính thức

14. **ArduPilot MAVLink Precision Landing.** Định nghĩa field, frame, tần số và
    angle/XYZ của `LANDING_TARGET`.
    <https://ardupilot.org/dev/docs/mavlink-precision-landing.html>

15. **ArduPilot Precision Landing and Loiter.** Tham số `PLND_*`, lost-target,
    retry, moving target và quy trình sử dụng.
    <https://ardupilot.org/copter/docs/precision-landing-and-loiter.html>

16. **ArduPilot SITL simulated peripherals — Testing Precision Landing.** Quy
    trình mô phỏng precision landing và rangefinder, xem log `PL`.
    <https://ardupilot.org/dev/docs/adding_simulated_devices.html#testing-precision-landing>

17. **MAVLink Landing Target Protocol.** Ý nghĩa angle, distance, size và pose.
    <https://mavlink.io/en/services/landing_target.html>

18. **ArduCopter source — precision landing control/state machine call sites.**
    Dùng để đối chiếu firmware thực tế thay vì suy đoán controller.
    <https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/mode.cpp>

## 13. Quyết định kỹ thuật đề xuất

Để phát triển an toàn và ít rủi ro nhất:

1. Giữ ArduCopter làm flight controller duy nhất.
2. Pi chỉ xuất measurement đã quality-gate.
3. Hiệu chuẩn intrinsic/extrinsic trước khi bật MAVLink landing.
4. Chuyển từ single ID center sang board pose có geometry thật.
5. Cấm duplicate target ID; ambiguous phải ngừng target.
6. Thêm target-lock và state machine validity.
7. Thêm rangefinder và nguồn horizontal position phù hợp cho bay trong nhà.
8. Test native Precision Landing trong SITL trước, sau đó mới test bridge.
9. Không trộn bài toán marker landing với bài toán tìm bãi tự nhiên/an toàn.
10. Chỉ mở flight envelope theo từng gate có log và metric định lượng.
