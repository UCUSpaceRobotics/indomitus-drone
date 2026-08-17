#!/usr/bin/env python3
"""Vision Web Streamer — Streams camera frames with Simulink ROS 2 overlays over HTTP.

Usage on Raspberry Pi:
    source /opt/ros/jazzy/setup.bash
    export ROS_DOMAIN_ID=42
    python3 scripts/vision_web_streamer.py

Then open in your browser:
    http://<pi_ip>:5000
"""

import cv2
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


# Shared state between ROS thread, Camera capture thread, and Web server
class SharedState:
    def __init__(self):
        self.lock = threading.Lock()
        self.latest_frame = None
        self.marker_id = 0.0
        self.x_offset = 0.0
        self.y_offset = 0.0
        self.last_ros_time = 0.0
        self.fps = 0.0


state = SharedState()


# ── ROS 2 Subscriber ────────────────────────────────────────────────────────
class VisionListenerNode(Node):
    def __init__(self):
        super().__init__("vision_web_listener")
        self.sub = self.create_subscription(
            Point, "/erc/vision_targets", self.callback, 10
        )

    def callback(self, msg: Point):
        with state.lock:
            state.x_offset = msg.x
            state.y_offset = msg.y
            state.marker_id = msg.z
            state.last_ros_time = time.time()


def ros_spin_thread():
    rclpy.init()
    node = VisionListenerNode()
    try:
        rclpy.spin(node)
    except Exception:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ── Camera Capture & Overlay Loop ──────────────────────────────────────────
def camera_thread():
    # OpenCV V4L2 capture from loopback device
    cap = cv2.VideoCapture("/dev/video10", cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))

    # Local OpenCV ArUco detector for bounding box visualization
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)
    try:
        aruco_detector = cv2.aruco.ArucoDetector(aruco_dict)
    except AttributeError:
        aruco_detector = None

    frame_count = 0
    t_start = time.time()

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret or frame is None:
            time.sleep(0.01)
            continue

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
        cv2.circle(frame, (cx, cy), 5, (0, 255, 255), 1)

        # Detect and outline ArUco corners
        if aruco_detector is not None:
            corners, ids, _ = aruco_detector.detectMarkers(frame)
            if ids is not None and len(ids) > 0:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)

        # Retrieve latest ROS 2 detection data
        with state.lock:
            m_id = state.marker_id
            m_x = state.x_offset
            m_y = state.y_offset
            last_t = state.last_ros_time
            fps = state.fps

        is_fresh = (time.time() - last_t) < 1.0

        # Draw status dashboard overlay
        overlay = frame.copy()
        cv2.rectangle(overlay, (10, 10), (320, 130), (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, frame, 0.4, 0, frame)

        # Status text
        cv2.putText(frame, f"FPS: {fps:.1f}  |  Res: {w}x{h}", (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

        if is_fresh and m_id > 0:
            color = (0, 255, 0) if m_id == 102 else (0, 200, 255)
            marker_name = "LANDING TARGET (102)" if m_id == 102 else "TAKEOFF PAD (101)"

            cv2.putText(frame, f"STATUS: DETECTED", (20, 55),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            cv2.putText(frame, f"TARGET: {marker_name}", (20, 78),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            cv2.putText(frame, f"X OFFSET: {m_x:+.3f} m  (Right)", (20, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(frame, f"Y OFFSET: {m_y:+.3f} m  (Forward)", (20, 120),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)

            # Draw vector pointing from drone center to target
            px_offset_x = int(m_x * 400)
            px_offset_y = int(m_y * 400)
            target_px = (cx + px_offset_x, cy + px_offset_y)
            cv2.arrowedLine(frame, (cx, cy), target_px, color, 2, tipLength=0.2)
            cv2.circle(frame, target_px, 12, color, 2)
        else:
            cv2.putText(frame, "STATUS: SEARCHING...", (20, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (100, 100, 255), 2)
            cv2.putText(frame, "No marker detected by Simulink", (20, 90),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (180, 180, 180), 1)

        # Compress to JPEG
        _, jpeg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with state.lock:
            state.latest_frame = jpeg.tobytes()

        time.sleep(0.03)

    cap.release()


# ── HTTP Streaming Server ──────────────────────────────────────────────────
class StreamingHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path == "/":
            self.send_response(200)
            self.send_header("Content-type", "text/html")
            self.end_headers()
            html = """
            <html>
            <head>
                <title>ERC 2026 - Vision Stream</title>
                <style>
                    body { background-color: #1a1a1a; color: #fff; font-family: monospace; text-align: center; margin: 0; padding: 20px; }
                    h2 { margin-bottom: 10px; color: #00e676; }
                    .stream-box { display: inline-block; border: 2px solid #333; border-radius: 8px; overflow: hidden; box-shadow: 0 4px 20px rgba(0,0,0,0.6); }
                    img { display: block; width: 640px; height: 480px; }
                    p { color: #888; margin-top: 10px; font-size: 13px; }
                </style>
            </head>
            <body>
                <h2>ERC 2026 Vision Streamer</h2>
                <div class="stream-box">
                    <img src="/stream.mjpg" alt="Live Feed" />
                </div>
                <p>Topic: /erc/vision_targets | Simulink ArUco Visualizer</p>
            </body>
            </html>
            """
            self.wfile.write(html.encode("utf-8"))

        elif self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=FRAME")
            self.end_headers()
            try:
                while True:
                    with state.lock:
                        frame_data = state.latest_frame
                    if frame_data is not None:
                        self.wfile.write(b"--FRAME\r\n")
                        self.send_header("Content-Type", "image/jpeg")
                        self.send_header("Content-Length", str(len(frame_data)))
                        self.end_headers()
                        self.wfile.write(frame_data)
                        self.wfile.write(b"\r\n")
                    time.sleep(0.04)
            except Exception:
                pass
        else:
            self.send_error(404)
            self.end_headers()


def run_server(port=5000):
    server = HTTPServer(("0.0.0.0", port), StreamingHandler)
    print(f"[STREAMER] Web Streamer active on http://0.0.0.0:{port}")
    server.serve_forever()


if __name__ == "__main__":
    t_ros = threading.Thread(target=ros_spin_thread, daemon=True)
    t_cam = threading.Thread(target=camera_thread, daemon=True)
    t_ros.start()
    t_cam.start()

    run_server(5000)
