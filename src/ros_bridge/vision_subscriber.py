"""ROS 2 vision bridge — subscribes to Simulink vision output and provides
a clean API for the state machine.

This module encapsulates all ROS 2 complexity. The state machine never
imports rclpy directly — it only talks to VisionBridge.

Message protocol on /erc/vision_targets (std_msgs/msg/Float64MultiArray):
    data = [marker_id, tx, ty, tz, rx, ry, rz]

Usage:
    rclpy.init()
    bridge = VisionBridge(topic="/erc/vision_targets", grid_config=config["grid"])

    while running:
        bridge.spin_once()
        target = bridge.get_latest_target()
        if target is not None:
            print(f"Marker {target['marker_id']} tvec={target['tvec']} rvec={target['rvec']}")

    bridge.shutdown()
    rclpy.shutdown()
"""

from __future__ import annotations

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import Int32, Float64MultiArray

from src.utils.grid_mapper import GridMapper


# Marker IDs used in the ERC 2026 competition.
MARKER_ID_ORIGIN = 101
MARKER_ID_LANDING = 102

# How old a detection can be before we consider it stale (300ms = ~4.5 frames at 15 FPS).
DEFAULT_DETECTION_TIMEOUT_S = 0.3


class _VisionSubscriberNode(Node):
    """Internal rclpy Node. Not exposed outside this module."""

    def __init__(
        self,
        vision_topic: str,
        state_topic: str,
        telemetry_topic: str,
        on_vision_callback,
        on_state_callback,
    ):
        super().__init__("vision_bridge")
        self.vision_sub = self.create_subscription(
            Float64MultiArray, vision_topic, on_vision_callback, 10
        )
        self.state_sub = self.create_subscription(
            Int32, state_topic, on_state_callback, 10
        )
        self.telemetry_pub = self.create_publisher(Pose, telemetry_topic, 10)
        self.get_logger().info(
            f"Subscribed to {vision_topic} and {state_topic}; publishing telemetry to {telemetry_topic}"
        )


class VisionBridge:
    """High-level interface between ROS 2 vision/telemetry data and the state machine.

    Owns an internal rclpy Node, processes incoming vision/state messages, and publishes
    drone telemetry to the MATLAB Simulink supervisor.
    """

    def __init__(
        self,
        topic: str = "/erc/vision_targets",
        state_topic: str = "/erc/mission_state",
        telemetry_topic: str = "/erc/drone_telemetry",
        grid_config: dict | None = None,
        detection_timeout_s: float = DEFAULT_DETECTION_TIMEOUT_S,
    ):
        """Create the vision bridge.

        Args:
            topic: ROS 2 topic name for vision targets (Simulink -> Python).
            state_topic: ROS 2 topic name for mission state (Simulink -> Python).
            telemetry_topic: ROS 2 topic name for drone telemetry (Python -> Simulink).
            grid_config: The 'grid' section of mission_params.yaml.
                         If None, probe-to-sector mapping is disabled.
            detection_timeout_s: Seconds after which a detection is considered
                                 stale and get_latest_target() returns None.
        """
        self._detection_timeout_s = detection_timeout_s

        # Grid mapper for probe detection (optional).
        self._grid_mapper = GridMapper(grid_config) if grid_config else None

        # Latest marker detection storage.
        self._latest_marker_id: int = 0
        self._latest_tvec: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._latest_rvec: tuple[float, float, float] = (0.0, 0.0, 0.0)
        self._latest_detection_seq: int = 0
        self._last_detection_time: float = 0.0

        # Latest supervisory state from Simulink.
        self._latest_simulink_state: int | None = None
        self._last_state_time: float = 0.0

        # Probe detection accumulator.
        # Using a set of sector IDs for automatic deduplication.
        self._detected_probe_sectors: set[str] = set()

        # Message counter for diagnostics.
        self._msg_count: int = 0
        self._state_msg_count: int = 0

        # Create the internal ROS 2 node.
        self._node = _VisionSubscriberNode(
            vision_topic=topic,
            state_topic=state_topic,
            telemetry_topic=telemetry_topic,
            on_vision_callback=self._on_vision_msg,
            on_state_callback=self._on_state_msg,
        )

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
                tvec (tuple[float, float, float]): Target translation (tx, ty, tz).
                rvec (tuple[float, float, float]): Target rotation vector (rx, ry, rz).
                detection_seq (int): Sequence number of this valid marker detection.
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
            "marker_id": self._latest_marker_id,
            "tvec": self._latest_tvec,
            "rvec": self._latest_rvec,
            "detection_seq": self._latest_detection_seq,
            "age_s": age,
        }

    def publish_telemetry(
        self,
        alt: float,
        is_armed: bool,
        ekf_healthy: bool,
        origin_dist: float,
        connected: bool,
        manual_override: bool,
        waypoints_exhausted: bool,
    ) -> None:
        """Publish real-time telemetry to the MATLAB Simulink supervisor.

        Packed into a geometry_msgs/msg/Pose message:
            position.x    = alt (meters)
            position.y    = is_armed (1.0 / 0.0)
            position.z    = ekf_healthy (1.0 / 0.0)
            orientation.x = origin_dist (meters)
            orientation.y = connected (1.0 / 0.0)
            orientation.z = manual_override (1.0 / 0.0)
            orientation.w = waypoints_exhausted (1.0 / 0.0)
        """
        msg = Pose(
            position=Point(
                x=float(alt),
                y=1.0 if is_armed else 0.0,
                z=1.0 if ekf_healthy else 0.0,
            ),
            orientation=Quaternion(
                x=float(origin_dist),
                y=1.0 if connected else 0.0,
                z=1.0 if manual_override else 0.0,
                w=1.0 if waypoints_exhausted else 0.0,
            ),
        )
        self._node.telemetry_pub.publish(msg)

    def get_simulink_state(self) -> int | None:
        """Return the latest supervisory mission state commanded by Simulink.

        Returns:
            int (0=IDLE, 1=TAKEOFF, 2=SEARCH, 3=DESCEND, 4=COMPLETE), or None if no state received.
        """
        if time.time() - self._last_state_time >= 0.5:
            return None
        return self._latest_simulink_state

    def get_state_message_count(self) -> int:
        """Return the total number of state messages received from Simulink."""
        return self._state_msg_count

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
    # Internal callbacks
    # ------------------------------------------------------------------

    def _on_state_msg(self, msg: Int32) -> None:
        """Called by rclpy when a new mission state arrives from Simulink."""
        self._state_msg_count += 1
        self._latest_simulink_state = int(msg.data)
        self._last_state_time = time.time()

    def _on_vision_msg(self, msg: Float64MultiArray) -> None:
        """Called by rclpy when a new Float64MultiArray message arrives.

        Expected data: [marker_id, tx, ty, tz, rx, ry, rz].
        """
        self._msg_count += 1

        if len(msg.data) != 7:
            self._node.get_logger().warning(
                f"Ignoring malformed vision message: expected 7 values, got {len(msg.data)}"
            )
            return

        try:
            values = tuple(float(value) for value in msg.data)
        except (TypeError, ValueError, OverflowError):
            self._node.get_logger().warning(
                "Ignoring malformed vision message: values must be numeric"
            )
            return

        if not all(math.isfinite(value) for value in values):
            self._node.get_logger().warning(
                "Ignoring malformed vision message: values must be finite"
            )
            return

        marker_id_value = values[0]
        if not marker_id_value.is_integer():
            self._node.get_logger().warning(
                f"Ignoring malformed vision message: marker ID {marker_id_value} is not integral"
            )
            return

        marker_id = int(marker_id_value)
        tx, ty, tz, rx, ry, rz = values[1:]

        if marker_id in (MARKER_ID_ORIGIN, MARKER_ID_LANDING):
            # ArUco marker detection.
            self._latest_marker_id = marker_id
            self._latest_tvec = (tx, ty, tz)
            self._latest_rvec = (rx, ry, rz)
            self._latest_detection_seq += 1
            self._last_detection_time = time.time()

            # self._node.get_logger().info(
            #     f"Landing target {marker_id}ID detected at offset "
            #     f"(x={self._latest_x:.2f}, y={self._latest_y:.2f})"
            # )

        elif marker_id < 0:
            # Probe detection. tx, ty are position relative to takeoff pad.
            if self._grid_mapper is not None:
                sector = self._grid_mapper.position_to_sector(tx, ty)
                if sector is not None:
                    if sector not in self._detected_probe_sectors:
                        self._node.get_logger().info(
                            f"New probe detected in sector {sector} "
                            f"(position: x={tx:.2f}, y={ty:.2f})"
                        )
                    self._detected_probe_sectors.add(sector)

        # marker_id == 0 → no detection, silently ignore.
