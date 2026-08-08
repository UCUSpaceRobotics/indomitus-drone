#!/usr/bin/env python3
"""Dummy vision publisher — simulates the Simulink C++ ROS 2 node for bench testing.

Publishes geometry_msgs/msg/Point to /erc/vision_targets at 10 Hz.
Supports multiple modes to test different state machine behaviors.

Usage:
    # Default: marker 102 moving in a circle (tests ALIGN convergence)
    python3 tests/test_dummy_publisher.py

    # Static marker at a fixed offset (tests steady-state ALIGN + LAND)
    python3 tests/test_dummy_publisher.py --mode static

    # Marker approaching center (simulates successful alignment)
    python3 tests/test_dummy_publisher.py --mode approach

    # No marker, only probes (tests SEARCH timeout + probe detection)
    python3 tests/test_dummy_publisher.py --mode probes-only

    # Marker that appears and disappears (tests lost-target recovery)
    python3 tests/test_dummy_publisher.py --mode intermittent
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class DummyVisionPublisher(Node):
    """Publishes simulated vision detections to /erc/vision_targets."""

    def __init__(self, mode: str, topic: str = "/erc/vision_targets"):
        super().__init__("dummy_vision_publisher")
        self.pub = self.create_publisher(Point, topic, 10)
        self.timer = self.create_timer(0.1, self.tick)  # 10 Hz
        self.mode = mode
        self.t = 0.0
        self.start_time = time.time()
        self.get_logger().info(
            f"Publishing mode='{mode}' to {topic} at 10 Hz"
        )

    def tick(self):
        """Called 10 times per second by the ROS 2 timer."""
        self.t += 0.1
        msg = Point()

        if self.mode == "circle":
            msg = self._mode_circle()
        elif self.mode == "static":
            msg = self._mode_static()
        elif self.mode == "approach":
            msg = self._mode_approach()
        elif self.mode == "probes-only":
            msg = self._mode_probes_only()
        elif self.mode == "intermittent":
            msg = self._mode_intermittent()
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
            return

        self.pub.publish(msg)

    # ------------------------------------------------------------------
    # Simulation modes
    # ------------------------------------------------------------------

    def _mode_circle(self) -> Point:
        """Marker 102 drifting in a circle with 0.3 m radius.

        Tests: ALIGN state convergence — the state machine must send
        velocity corrections to follow the moving target.
        """
        msg = Point()
        msg.x = 0.3 * math.sin(self.t)    # right/left offset
        msg.y = 0.3 * math.cos(self.t)    # forward/back offset
        msg.z = 102.0                      # landing target marker
        return msg

    def _mode_static(self) -> Point:
        """Marker 102 at a fixed offset of (0.15, -0.10) meters.

        Tests: Steady-state ALIGN — the P-controller should drive the
        drone to center over this fixed point, then transition to LAND.
        """
        msg = Point()
        msg.x = 0.15     # 15 cm to the right
        msg.y = -0.10    # 10 cm behind
        msg.z = 102.0
        return msg

    def _mode_approach(self) -> Point:
        """Marker 102 starting at (0.5, 0.5) and slowly approaching (0, 0).

        Simulates a successful alignment sequence: the offsets shrink to
        zero over ~10 seconds, triggering the ALIGN → LAND transition.
        """
        elapsed = time.time() - self.start_time
        # Exponential decay toward center over ~10 seconds.
        decay = math.exp(-elapsed / 3.0)
        msg = Point()
        msg.x = 0.5 * decay
        msg.y = 0.5 * decay
        msg.z = 102.0
        return msg

    def _mode_probes_only(self) -> Point:
        """No marker detection — only probe detections every 3 seconds.

        Tests: SEARCH timeout (no marker 102 found) and probe
        accumulation in the grid mapper.

        Publishes 3 probes at fixed positions:
            Probe 1 at world (1.0, 0.5)  → depends on grid config for sector
            Probe 2 at world (-0.5, 1.5)
            Probe 3 at world (0.0, -1.0)
        """
        elapsed = time.time() - self.start_time
        probe_positions = [
            (1.0, 0.5),
            (-0.5, 1.5),
            (0.0, -1.0),
        ]
        msg = Point()

        # Cycle through probes, one every 3 seconds.
        probe_idx = int(elapsed / 3.0) % len(probe_positions)
        x, y = probe_positions[probe_idx]
        msg.x = x
        msg.y = y
        msg.z = -1.0  # Negative z = probe detection
        return msg

    def _mode_intermittent(self) -> Point:
        """Marker 102 that appears for 3 seconds, disappears for 2 seconds.

        Tests: Lost-target handling in ALIGN state. The state machine
        should fall back to SEARCH when the marker disappears, then
        re-acquire it when it reappears.
        """
        elapsed = time.time() - self.start_time
        cycle_pos = elapsed % 5.0  # 5-second cycle

        msg = Point()
        if cycle_pos < 3.0:
            # Marker visible for 3 seconds.
            msg.x = 0.10
            msg.y = 0.05
            msg.z = 102.0
        else:
            # Marker gone for 2 seconds.
            msg.z = 0.0  # No detection
        return msg


def main():
    parser = argparse.ArgumentParser(
        description="Dummy vision publisher for bench testing."
    )
    parser.add_argument(
        "--mode",
        choices=["circle", "static", "approach", "probes-only", "intermittent"],
        default="circle",
        help="Simulation mode (default: circle)",
    )
    parser.add_argument(
        "--topic",
        default="/erc/vision_targets",
        help="ROS 2 topic to publish to (default: /erc/vision_targets)",
    )

    # Parse known args to avoid conflicts with ROS 2 args.
    args, _ = parser.parse_known_args()

    rclpy.init(args=sys.argv)
    node = DummyVisionPublisher(mode=args.mode, topic=args.topic)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
