#!/usr/bin/env python3
"""Vision Web Streamer — Streams camera frames with VisionBridge overlays over HTTP.

Usage on Raspberry Pi:
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=42
    python3 scripts/vision_web_streamer.py
"""

import os
from pathlib import Path
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
from socketserver import ThreadingMixIn

import cv2
import rclpy
import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from src.ros_bridge.vision_subscriber import MARKER_ID_LANDING, VisionBridge


class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_frame = None
        self.latest_target = None
        self.fps = 0.0
        self.cam_connected = False


state = SharedState()


# ── VisionBridge Polling ────────────────────────────────────────────────────
def ros_spin_thread(config):
    rclpy.init()
    bridge = VisionBridge(
        topic=config["ros2"]["vision_topic"],
        grid_config=config["grid"],
    )
    try:
        while rclpy.ok():
            bridge.spin_once()
            with state.lock:
                state.latest_target = bridge.get_latest_target()
            time.sleep(0.01)
    except Exception as error:
        print(f"[VISION ERROR] {error}", file=sys.stderr)
    finally:
        bridge.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


# ── Camera Capture & Overlay Loop ──────────────────────────────────────────
def camera_thread():
    print("[CAMERA] Initializing capture from /dev/video11...")
    cap = cv2.VideoCapture("/dev/video11", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

    if not cap.isOpened():
        print("[CAMERA ERROR] Could not open /dev/video10!")
        return

    print("[CAMERA] Capture loop running successfully.")
    frame_count = 0
    t_start = time.time()

    while True:
        ret, frame = cap.read()
        if not ret or frame is None:
            with state.lock:
                state.cam_connected = False
            time.sleep(0.05)
            continue

        with state.lock:
            state.cam_connected = True

        frame_count += 1
        elapsed = time.time() - t_start
        if elapsed >= 1.0:
            with state.lock:
                state.fps = frame_count / elapsed
            frame_count = 0
            t_start = time.time()

        h, w = frame.shape[:2]
        cx, cy = w // 2, h // 2

        # Draw optical center crosshairs (drone center)
        cv2.line(frame, (cx - 20, cy), (cx + 20, cy), (0, 255, 255), 1)
        cv2.line(frame, (cx, cy - 20), (cx, cy + 20), (0, 255, 255), 1)
        cv2.circle(frame, (cx, cy), 4, (0, 255, 255), 1)

        # Retrieve latest detection processed by VisionBridge.
        with state.lock:
            target = state.latest_target
            fps = state.fps

        # Draw HUD Box
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (330, 135), (20, 20, 20), -1)
        cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

        # Status text
        cv2.putText(frame, f"FPS: {fps:.1f}  |  Res: {w}x{h}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

        if target is not None:
            m_id = target["marker_id"]
            m_x = target["x_offset_m"]
            m_y = target["y_offset_m"]
            color = (0, 255, 0) if m_id == MARKER_ID_LANDING else (0, 200, 255)
            marker_name = (
                f"LANDING TARGET ({m_id})"
                if m_id == MARKER_ID_LANDING
                else f"TAKEOFF PAD ({m_id})"
            )

            cv2.putText(frame, "STATUS: TARGET ACQUIRED", (20, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
            cv2.putText(frame, f"TARGET: {marker_name}", (20, 78),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
            cv2.putText(frame, f"X OFFSET: {m_x:+.3f} m  (Forward)", (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.putText(frame, f"Y OFFSET: {m_y:+.3f} m  (Right)", (20, 122),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)

            # Draw vector pointing from drone center to target
            px_offset_x = -int(m_y * 350)
            px_offset_y = int(m_x * 350)
            target_px = (cx + px_offset_x, cy + px_offset_y)
            cv2.arrowedLine(frame, (cx, cy), target_px, color, 2, tipLength=0.2)
            cv2.circle(frame, target_px, 10, color, 2)
        else:
            cv2.putText(frame, "STATUS: SEARCHING...", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 255), 2)
            cv2.putText(frame, "VisionBridge: No active marker", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        # Compress to JPEG
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        with state.lock:
            state.latest_frame = jpeg.tobytes()

        time.sleep(0.03)

    cap.release()


# ── HTTP Multi-Threaded Streaming Server ───────────────────────────────────
class ThreadedHTTPServer(ThreadingMixIn, HTTPServer):
    daemon_threads = True


class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            html = """<!DOCTYPE html>
<html>
<head>
    <meta charset="utf-8">
    <title>ERC 2026 - Vision Stream</title>
    <style>
        body { background: #121212; color: #eee; font-family: monospace; text-align: center; margin: 0; padding: 20px; }
        h2 { color: #00e676; margin-bottom: 15px; }
        .box { display: inline-block; border: 2px solid #333; border-radius: 8px; overflow: hidden; background: #000; }
        img { display: block; width: 640px; height: 480px; }
        .info { margin-top: 15px; color: #888; font-size: 13px; }
    </style>
</head>
<body>
    <h2>ERC 2026 — VisionBridge Stream</h2>
    <div class="box">
        <img src="/stream.mjpg" alt="Video Stream" />
    </div>
    <div class="info">Live feed from /dev/video10 | Overlay from VisionBridge (/erc/vision_targets)</div>
</body>
</html>"""
            self.wfile.write(html.encode("utf-8"))

        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()

            while True:
                with state.lock:
                    frame = state.latest_frame
                if frame is not None:
                    self.wfile.write(b"--frame\r\n")
                    self.send_header("Content-Type", "image/jpeg")
                    self.send_header("Content-Length", str(len(frame)))
                    self.end_headers()
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
                time.sleep(0.04)
        else:
            self.send_error(404)
            self.end_headers()

    def log_message(self, format, *args):
        # Suppress noisy HTTP per-frame access logs
        return


def run_server(port=5000):
    server = ThreadedHTTPServer(("0.0.0.0", port), StreamingHandler)
    print(f"[STREAMER] Server ready! Open http://10.20.18.63:{port} in your browser.")
    server.serve_forever()


if __name__ == "__main__":
    config_path = REPO_ROOT / "config" / "mission_params.yaml"
    with config_path.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    os.environ.setdefault("ROS_DOMAIN_ID", str(config["ros2"]["domain_id"]))

    t_ros = threading.Thread(target=ros_spin_thread, args=(config,), daemon=True)
    t_cam = threading.Thread(target=camera_thread, daemon=True)
    t_ros.start()
    t_cam.start()

    run_server(5000)
