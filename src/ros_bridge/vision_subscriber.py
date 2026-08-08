"""ROS 2 vision bridge — subscribes to Simulink vision output and provides
a clean API for the state machine.

This module encapsulates all ROS 2 complexity. The state machine never
imports rclpy directly — it only talks to VisionBridge.

Message protocol on /erc/vision_targets (geometry_msgs/msg/Point):
    z = 101.0  → Takeoff pad marker detected.  x, y = camera-frame offset (meters).
    z = 102.0  → Landing target marker detected. x, y = camera-frame offset (meters).
    z < 0      → Probe detected. x, y = estimated world position relative to
                 takeoff pad (meters). |z| can encode probe ID.
    z = 0.0    → No detection (heartbeat / empty frame).

Usage:
    rclpy.init()
    bridge = VisionBridge(topic="/erc/vision_targets", grid_config=config["grid"])

    while running:
        bridge.spin_once()
        target = bridge.get_latest_target()
        if target is not None:
            print(f"Marker {target['marker_id']} at offset ({target['x_offset_m']}, {target['y_offset_m']})")

    bridge.shutdown()
    rclpy.shutdown()
"""

from __future__ import annotations

import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

from src.utils.grid_mapper import GridMapper


# Marker IDs used in the ERC 2026 competition.
MARKER_ID_ORIGIN = 101
MARKER_ID_LANDING = 102

# How old a detection can be before we consider it stale.
DEFAULT_DETECTION_TIMEOUT_S = 0.5


class _VisionSubscriberNode(Node):
    """Internal rclpy Node. Not exposed outside this module."""

    def __init__(self, topic: str, on_message_callback):
        super().__init__("vision_bridge")
        self.subscription = self.create_subscription(
            Point, topic, on_message_callback, 10
        )
        self.get_logger().info(f"Subscribed to {topic}")


class VisionBridge:
    """High-level interface between ROS 2 vision data and the state machine.

    Owns an internal rclpy Node, processes incoming messages, and provides
    simple getter methods that the state machine polls.
    """

    def __init__(
        self,
        topic: str = "/erc/vision_targets",
        grid_config: dict | None = None,
        detection_timeout_s: float = DEFAULT_DETECTION_TIMEOUT_S,
    ):
        """Create the vision bridge.

        Args:
            topic: ROS 2 topic name to subscribe to.
            grid_config: The 'grid' section of mission_params.yaml.
                         If None, probe-to-sector mapping is disabled.
            detection_timeout_s: Seconds after which a detection is considered
                                 stale and get_latest_target() returns None.
        """
        self._detection_timeout_s = detection_timeout_s

        # Grid mapper for probe detection (optional).
        self._grid_mapper = GridMapper(grid_config) if grid_config else None

        # Latest marker detection storage.
        self._latest_marker_id: float = 0.0
        self._latest_x: float = 0.0
        self._latest_y: float = 0.0
        self._last_detection_time: float = 0.0

        # Probe detection accumulator.
        # Using a set of sector IDs for automatic deduplication.
        self._detected_probe_sectors: set[str] = set()

        # Message counter for diagnostics.
        self._msg_count: int = 0

        # Create the internal ROS 2 node.
        self._node = _VisionSubscriberNode(topic, self._on_vision_msg)

    # ------------------------------------------------------------------
    # Public API — called by the state machine
    # ------------------------------------------------------------------

    def spin_once(self) -> None:
        """Process any pending ROS 2 messages without blocking.

        Must be called on every iteration of the main loop. Internally
        calls rclpy.spin_once() with zero timeout — it checks for messages
        and returns immediately.
        """
        rclpy.spin_once(self._node, timeout_sec=0)

    def get_latest_target(self) -> dict | None:
        """Return the most recent ArUco marker detection.

        Returns:
            A dict with keys:
                marker_id (int): 101 or 102.
                x_offset_m (float): Horizontal offset from camera center (right = positive).
                y_offset_m (float): Vertical offset from camera center (forward = positive).
                age_s (float): Seconds since this detection was received.
            Returns None if no marker has been detected, or if the last
            detection is older than detection_timeout_s.
        """
        if self._last_detection_time == 0.0:
            return None

        age = time.time() - self._last_detection_time

        if age > self._detection_timeout_s:
            return None

        return {
            "marker_id": int(self._latest_marker_id),
            "x_offset_m": self._latest_x,
            "y_offset_m": self._latest_y,
            "age_s": age,
        }

    def get_detected_probes(self) -> list[str]:
        """Return all unique probe sector IDs detected so far.

        Returns:
            Sorted list of sector ID strings, e.g. ["A2", "C4", "E1"].
            Empty list if no probes have been detected.
        """
        return sorted(self._detected_probe_sectors)

    def clear_probes(self) -> None:
        """Reset the probe detection accumulator.

        Call this at the start of each mission attempt if you want
        per-attempt tracking (the competition scores per-mission).
        """
        self._detected_probe_sectors.clear()

    def get_message_count(self) -> int:
        """Return the total number of messages received (for diagnostics)."""
        return self._msg_count

    def shutdown(self) -> None:
        """Destroy the internal ROS 2 node. Call before rclpy.shutdown()."""
        self._node.destroy_node()

    # ------------------------------------------------------------------
    # Internal callback
    # ------------------------------------------------------------------

    def _on_vision_msg(self, msg: Point) -> None:
        """Called by rclpy when a new Point message arrives.

        Routing logic based on msg.z:
            z = 101 or 102  → marker detection, store as latest target.
            z < 0           → probe detection, map to grid sector.
            z = 0           → no detection, ignore.
        """
        self._msg_count += 1
        marker_id = msg.z

        if marker_id in (MARKER_ID_ORIGIN, MARKER_ID_LANDING):
            # ArUco marker detection.
            self._latest_marker_id = marker_id
            self._latest_x = msg.x
            self._latest_y = msg.y
            self._last_detection_time = time.time()

        elif marker_id < 0:
            # Probe detection. x, y are world position relative to takeoff pad.
            if self._grid_mapper is not None:
                sector = self._grid_mapper.position_to_sector(msg.x, msg.y)
                if sector is not None:
                    if sector not in self._detected_probe_sectors:
                        self._node.get_logger().info(
                            f"New probe detected in sector {sector} "
                            f"(position: x={msg.x:.2f}, y={msg.y:.2f})"
                        )
                    self._detected_probe_sectors.add(sector)

        # z == 0.0 → no detection, silently ignore.
