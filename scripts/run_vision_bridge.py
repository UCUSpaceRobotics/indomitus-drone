#!/usr/bin/env python3
"""Run VisionBridge without starting autonomous mission or MAVLink communication.

Usage from repository root:
    source /opt/ros/jazzy/setup.bash
    source ~/ros2_ws/install/setup.bash
    source .venv/bin/activate
    python3 scripts/run_vision_bridge.py
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import sys
import time

import yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run only the ROS 2 VisionBridge subscriber."
    )
    parser.add_argument(
        "--topic",
        help="Override the vision topic configured in mission_params.yaml.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    config_path = REPO_ROOT / "config" / "mission_params.yaml"
    with config_path.open(encoding="utf-8") as config_file:
        config = yaml.safe_load(config_file)

    topic = args.topic or config["ros2"]["vision_topic"]
    os.environ.setdefault("ROS_DOMAIN_ID", str(config["ros2"]["domain_id"]))

    try:
        import rclpy
        from src.ros_bridge.vision_subscriber import VisionBridge
    except ImportError as error:
        print(f"[VISION] ROS 2 import failed: {error}", file=sys.stderr)
        print("[VISION] Source /opt/ros/jazzy/setup.bash first.", file=sys.stderr)
        return 1

    rclpy.init()
    bridge = None

    try:
        bridge = VisionBridge(topic=topic, grid_config=config["grid"])
        print(
            f"[VISION] Running on {topic} (ROS_DOMAIN_ID={os.environ['ROS_DOMAIN_ID']})."
        )
        print("[VISION] No flight or MAVLink components started. Press Ctrl+C to stop.")

        while rclpy.ok():
            report_at = time.monotonic() + 1.0
            while rclpy.ok() and time.monotonic() < report_at:
                bridge.spin_once()
                time.sleep(0.02)

            target = bridge.get_latest_target()
            probes = bridge.get_detected_probes()
            status = f"messages={bridge.get_message_count()}"

            if target is None:
                status += " marker=none"
            else:
                status += (
                    f" marker={target['marker_id']}"
                    f" forward={target['x_offset_m']:+.3f}m"
                    f" right={target['y_offset_m']:+.3f}m"
                    f" age={target['age_s']:.3f}s"
                )

            status += f" probes={probes}"
            print(f"[VISION] {status}")
    except KeyboardInterrupt:
        print("\n[VISION] Stopping.")
    finally:
        if bridge is not None:
            bridge.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
