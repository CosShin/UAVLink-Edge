# Mô phỏng hạ cánh chính xác bằng ArUco

Thư mục này dựng luồng mô phỏng hạ cánh camera hoàn chỉnh:

```text
Gazebo Harmonic + iris_with_gimbal
  -> RTP/H.264 camera nhìn xuống UDP 5600
  -> scripts/vision_landing_bridge.py
  -> detector ArUco production trong Find_landing/
  -> MAVLink LANDING_TARGET
  -> ArduCopter SITL
```

Không cần ROS. Camera bridge chỉ nối vào loopback và không thể gửi lệnh tới
drone thật hoặc server từ xa.

## Môi trường khuyến nghị

- Ubuntu 22.04 Desktop x86_64, GPU/OpenGL acceleration bật.
- Gazebo Harmonic (`gz sim`).
- ArduPilot SITL, MAVProxy và plugin `ArduPilot/ardupilot_gazebo`.
- GStreamer và virtual environment của repository.

Raspberry Pi ARM64 chạy Debian không phải môi trường phù hợp để chạy Gazebo
GUI. Hãy chạy mô phỏng trên máy Ubuntu/VM, còn Pi dùng cho UAVLink Edge thật.

Tài liệu chính thức:

- <https://ardupilot.org/dev/docs/sitl-with-gazebo.html>
- <https://github.com/ArduPilot/ardupilot_gazebo>
- <https://ardupilot.org/copter/docs/precision-landing-and-loiter.html>

## 1. Cài plugin Gazebo

```bash
sudo apt update
sudo apt install -y libgz-sim8-dev rapidjson-dev \
  libopencv-dev libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev \
  gstreamer1.0-plugins-bad gstreamer1.0-libav gstreamer1.0-gl

mkdir -p ~/gz_ws/src
cd ~/gz_ws/src
git clone https://github.com/ArduPilot/ardupilot_gazebo
cd ardupilot_gazebo
export GZ_VERSION=harmonic
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=RelWithDebInfo
make -j4
```

Cài ArduPilot SITL theo tài liệu chính thức và kiểm tra `sim_vehicle.py` trước.

## 2. Chuẩn bị và self-test

Từ root của repository:

```bash
simulation/gazebo_precision_landing/scripts/prepare_assets.sh
simulation/gazebo_precision_landing/scripts/check_environment.sh
venv/bin/python simulation/gazebo_precision_landing/scripts/test_detector_asset.py
```

Self-test phải báo đủ 12 marker, `TRACKING` và `control_valid=true`.

Nếu workspace đặt ở vị trí khác:

```bash
export ARDUPILOT_HOME=~/ardupilot
export ARDUPILOT_GAZEBO_HOME=~/gz_ws/src/ardupilot_gazebo
```

## 3. Chạy bốn terminal

Terminal 1 — Gazebo:

```bash
simulation/gazebo_precision_landing/scripts/run_gazebo.sh
```

Terminal 2 — ArduCopter SITL:

```bash
simulation/gazebo_precision_landing/scripts/run_sitl.sh
```

Trong MAVProxy:

```text
mode guided
arm throttle
takeoff 8
```

Terminal 3 — bật camera và xoay xuống:

```bash
simulation/gazebo_precision_landing/scripts/enable_down_camera.sh
```

Trong MAVProxy:

```text
rc 7 1100
```

Terminal 4 — detector production và LANDING_TARGET:

```bash
simulation/gazebo_precision_landing/scripts/run_vision_bridge.sh --preview
```

Khi trạng thái chuyển sang `TRACKING`, cho hạ cánh:

```text
mode land
```

Nhấn `q` hoặc `Esc` để đóng preview. Bỏ `--preview` khi chạy headless.

## 4. Kiểm tra video thô

Chỉ chạy viewer khi bridge đang tắt để tránh hai process tranh UDP 5600:

```bash
simulation/gazebo_precision_landing/scripts/view_camera.sh
```

## 5. Kịch bản kiểm thử Precision Landing

Sau khi SITL chạy:

```bash
simulation/gazebo_precision_landing/scripts/run_scenario.sh center
simulation/gazebo_precision_landing/scripts/run_scenario.sh sine
simulation/gazebo_precision_landing/scripts/run_scenario.sh dropout
```

- `center`: target chính giữa ổn định.
- `sine`: target lệch tuần hoàn, có noise và packet loss.
- `dropout`: mất target bốn giây để kiểm tra retry/strict policy.

## 6. Tiêu chí đạt

- Board chuyển `ACQUIRING -> TRACKING` sau 5 measurement tốt.
- Chỉ phát target khi quality >= 0.55, không duplicate và control hợp lệ.
- Khi mất board, dừng phát measurement mới ngay; không dùng target cũ.
- Drone sửa sai số đúng chiều và không rung tăng dần.
- Dropout thực hiện đúng policy retry.
- Chỉ thử bay thật sau replay, SITL và bench test tháo cánh đều đạt.

Texture dùng marker cạnh 0.20 m và FOV giả lập 60 x 45 độ. SITL không thay thế
calibration camera thật, kiểm tra motion blur, rolling shutter, ánh sáng và rung.
