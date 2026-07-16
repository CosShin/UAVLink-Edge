#!/usr/bin/env bash
set -euo pipefail
EXPECTED_TOPIC="/world/precision_landing/model/iris_with_gimbal/model/gimbal/link/pitch_link/sensor/camera/image/enable_streaming"
TOPIC="$(gz topic -l | awk '/\/enable_streaming$/ && /\/camera\/image\// { print; exit }')"
if [ -z "$TOPIC" ]; then
  TOPIC="$EXPECTED_TOPIC"
  echo "Không tự dò thấy topic; thử topic mặc định: $TOPIC"
else
  echo "Đã tìm thấy camera topic: $TOPIC"
fi
echo "Bật RTP/H.264 camera trên UDP 5600..."
gz topic -t "$TOPIC" -m gz.msgs.Boolean -p "data: 1"
echo "Camera đã bật. Trong MAVProxy dùng 'rc 7 1100' để nhìn xuống."

