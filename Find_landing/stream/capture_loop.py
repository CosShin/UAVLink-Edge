"""Hướng 1 — vòng stream: CaptureSource → FrameGate → wire → EncoderSink.

Mỗi camera (cam0/cam1) chạy một instance riêng của module này trong process
camera_streamer.py riêng: cùng code xử lý (Hướng 2), đầu ra trả về đúng
publish_path / RTSP của camera_id đó — không trộn frame giữa hai cổng.
"""

import os
import time

from camera_manager import get_camera_manager
from processing.pipeline import build_pipeline
from stream.capture_source import CaptureSource
from stream.encoder import EncoderSink
from stream.frame_gate import FrameGate
from stream.metrics import write_landing_telemetry, write_stats
from stream.wire_format import bgr_to_wire, wire_pixel_format


def run_capture_loop(streamer, pipe_write_fd: int):
    """
    Orchestrate Hướng 1 only — không nhúng CV/overlay (Hướng 2).
    streamer: CameraStreamer instance (config, running, counters).
    """
    find_landing_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    config = streamer.config

    cam_manager = get_camera_manager()
    source = CaptureSource(streamer, cam_manager)

    if not source.open():
        print("✗ Failed to initialize camera")
        streamer.running.clear()
        return

    print(
        f" Camera initialized (reported={source.sensor_format}, "
        f"bytes={source.sensor_byte_order}, ui={config.get('format')}, "
        f"wire={wire_pixel_format(config)})"
    )

    processing = build_pipeline(
        config,
        find_landing_dir,
        streamer.running,
        # USB frames burn the latest detection directly onto the current frame.
        # Avoid producing and retaining a stale full-frame overlay in the worker.
        overlay_processor=not bool(config.get("detection_enabled", True)),
    )
    if processing:
        processing.start()
        print(" Processing pipeline started (Hướng 2)")
    elif not config.get("detection_enabled") and not config.get("overlay_enabled"):
        print("ℹ️  Processing disabled (stream-only)")
    else:
        print("ℹ️  Detection disabled")

    gate = FrameGate(config, processing)
    encoder = EncoderSink(pipe_write_fd, streamer.running)
    encoder.start()

    gate_ms = int(config.get("gate_timeout_ms", 0))
    if gate_ms > 0:
        gate_mode = f"wait {gate_ms}ms (legacy)"
    else:
        gate_mode = "no-wait latest overlay (H1 isolated)"
    print(f" FrameGate: {gate_mode}")

    fps_interval = 1.0 / config["framerate"]
    last_frame_time = 0.0
    frame_count = 0
    encode_drops = 0
    last_stats_time = time.time()
    last_landing_write = 0.0
    streamer.start_time = time.time()
    last_sent_at_stats = 0
    low_fps_streak = 0
    encode_drops_at_last_stats = 0
    capture_failures = 0
    usb_source = str(config.get("source") or "csi").lower() == "usb"
    reconnect_after = max(10, int(config.get("framerate", 30) or 30))

    try:
        while streamer.running.is_set():
            # Drain sensor nhanh khi encoder đầy — không throttle 30fps
            if encoder.is_full():
                if source.drain():
                    encode_drops += 1
                time.sleep(0.001)
                continue

            current_time = time.time()
            if current_time - last_frame_time < fps_interval:
                time.sleep(0.001)
                continue
            last_frame_time = current_time

            if processing is None:
                frame_wire = source.capture_wire(config)
                if frame_wire is None:
                    capture_failures += 1
                    if usb_source and capture_failures >= reconnect_after:
                        print(" [WARN] USB webcam stopped delivering frames — reconnecting...")
                        source.close()
                        while streamer.running.is_set() and not source.open():
                            print(" [WARN] USB webcam unavailable — retrying in 2s")
                            time.sleep(2)
                        capture_failures = 0
                    continue
            else:
                frame_bgr = source.capture_bgr()
                if frame_bgr is None:
                    capture_failures += 1
                    if usb_source and capture_failures >= reconnect_after:
                        print(" [WARN] USB webcam stopped delivering frames — reconnecting...")
                        source.close()
                        while streamer.running.is_set() and not source.open():
                            print(" [WARN] USB webcam unavailable — retrying in 2s")
                            time.sleep(2)
                        capture_failures = 0
                    continue
                frame_gated = gate.resolve(frame_count, frame_bgr)
                frame_wire = bgr_to_wire(frame_gated, config)

            capture_failures = 0
            frame_count += 1

            drops = encoder.enqueue(frame_wire)
            encode_drops += drops
            streamer.frames_sent = encoder.frames_sent

            if processing:
                streamer.detections_count = processing.detections_count
                streamer.detection_result = processing.latest_detection()
                # LANDING_TARGET needs a fresh target at >= 1 Hz.  Keep the
                # file publisher at 10 Hz instead of tying it to 5 s stats.
                if current_time - last_landing_write >= 0.1:
                    write_landing_telemetry(
                        int(config.get("camera_id", 0)),
                        streamer.detection_result,
                        processing.detections_count,
                        config.get("size"),
                    )
                    last_landing_write = current_time

            if frame_count == 1 and encoder.frames_sent == 0:
                print(" First frame queued for encoder")

            if current_time - last_stats_time >= 5.0:
                elapsed = current_time - streamer.start_time
                fps_actual = encoder.frames_sent / elapsed if elapsed > 0 else 0
                capture_fps = frame_count / elapsed if elapsed > 0 else 0
                window_fps = (encoder.frames_sent - last_sent_at_stats) / 5.0
                det_count = processing.detections_count if processing else 0
                detection_rate = (det_count / encoder.frames_sent * 100) if encoder.frames_sent > 0 else 0
                drops_this_window = encode_drops - encode_drops_at_last_stats
                drop_note = f" | encode drops: {encode_drops}" if encode_drops else ""
                print(
                    f" Stats: {encoder.frames_sent} sent @ {fps_actual:.1f} fps | "
                    f"capture {capture_fps:.1f} fps | window {window_fps:.1f} fps | "
                    f"Detections: {det_count} ({detection_rate:.1f}%){drop_note}"
                )
                if drops_this_window > 0:
                    print(f" [WARN] Encoder backlog — dropped {drops_this_window} frame(s) (total {encode_drops})")
                write_stats(config, encoder.frames_sent, streamer.start_time,
                            capture_fps, encode_drops, window_fps)
                if window_fps < 18:
                    low_fps_streak += 1
                    if low_fps_streak == 3:
                        print(
                            f" [WARN] Window FPS low ({window_fps:.1f}) — "
                            f"encode drops: {encode_drops}"
                        )
                else:
                    low_fps_streak = 0
                encode_drops_at_last_stats = encode_drops
                last_sent_at_stats = encoder.frames_sent
                last_stats_time = current_time

    except Exception as e:
        print(f"✗ Capture error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        encoder.stop()
        if processing:
            processing.stop()
        source.close()
        streamer.frames_sent = encoder.frames_sent
        print(" Camera released")
