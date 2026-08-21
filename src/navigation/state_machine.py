"""Autonomous flight state machine for ERC 2026 Droning Sub-Task.

Implements a single mission cycle:
    IDLE → TAKEOFF → SEARCH → DESCEND → COMPLETE

The drone is manually placed on the takeoff platform before each run.
The program executes one mission and exits. Restart it for each attempt.

Alignment with the landing target is handled by ArduPilot's built-in
precision landing controller. The companion computer continuously sends
LANDING_TARGET messages with the target's BODY_FRD position; ArduPilot
uses its internal PID loops to steer toward the target during descent.

Prerequisites (set via Mission Planner):
    PLND_ENABLED = 1
    PLND_TYPE    = 1  (MAVLink)

Usage:
    mission = MissionController(command_queue, telemetry_queue, vision_bridge, config)
    while mission.state != FlightState.COMPLETE:
        mission.update()
        time.sleep(0.02)
"""

from __future__ import annotations

import math
import queue
import time
from enum import Enum

from src.comm.mavlink_node import create_command


class FlightState(Enum):
    """All possible states of the autonomous mission."""
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    SEARCH = "SEARCH"
    DESCEND = "DESCEND"
    COMPLETE = "COMPLETE"


class MissionController:
    """Finite state machine controlling a single autonomous drone mission.

    Architecture:
        - Reads drone telemetry from telemetry_queue (filled by mavlink_node.py).
        - Reads vision data from VisionBridge (filled by Simulink via ROS 2).
        - Sends flight commands to command_queue (consumed by mavlink_node.py).

    Safety features:
        - Geofence: emergency LAND if the drone drifts >3 m from takeoff origin.
        - Timeouts: every state has a timeout that triggers LAND on expiry.

    Args:
        command_queue: multiprocessing.Queue for sending commands to the comm process.
        telemetry_queue: multiprocessing.Queue for receiving telemetry from the comm process.
        vision_bridge: VisionBridge instance for reading vision detections.
        config: Parsed mission_params.yaml dictionary.
        led_indicator: Optional LEDController instance.
    """

    # Maximum horizontal distance from takeoff origin before emergency LAND (meters).
    MAX_DISTANCE_FROM_ORIGIN_M = 3.0

    def __init__(self, command_queue, telemetry_queue, vision_bridge, config: dict, led_indicator=None):
        self.cmd_q = command_queue
        self.telem_q = telemetry_queue
        self.vision = vision_bridge
        self.config = config
        self.led = led_indicator

        # Current state.
        self.state = FlightState.IDLE
        self.state_entry_time = time.time()

        # Timeouts (seconds).
        self.timeout_takeoff = config["timeouts"]["takeoff_s"]
        self.timeout_search = config["timeouts"]["search_sweep_s"]
        self.timeout_descend = config["timeouts"]["landing_s"]

        # Flight parameters.
        self.takeoff_alt = config["flight"]["takeoff_altitude_m"]
        self.landing_target_id = config["markers"]["landing_target_id"]

        # Latest telemetry snapshot (refreshed every tick).
        self.telem: dict = {}

        # TAKEOFF sub-phase tracking.
        self._takeoff_phase = 0  # 0=loiter, 1=arm, 2=guided, 3=takeoff_cmd, 4=climbing
        self._takeoff_cmd_time = 0.0

        # DESCEND state tracking.
        self._land_cmd_sent = False

        print("[STATE] MissionController initialized — single mission run.")
        print(f"[STATE] Starting in {self.state.value}")
        self._update_led_indicator()

    # ==================================================================
    # Public API
    # ==================================================================

    def update(self):
        """Execute one tick of the state machine.

        Called ~50 times/second by the main loop. This is the ONLY method
        the main loop calls. Internally it:
            1. Refreshes telemetry from the comm process queue.
            2. Processes pending ROS 2 vision messages.
            3. Checks geofence safety.
            4. Dispatches to the handler for the current state.
            5. Updates the LED indicator state.
        """
        self._refresh_telemetry()
        self.vision.spin_once()

        # Safety: emergency land if drone drifts too far from origin.
        if self._check_geofence():
            return  # State was forced to COMPLETE, skip normal handler.

        if self.state == FlightState.IDLE:
            self._update_idle()
        elif self.state == FlightState.TAKEOFF:
            self._update_takeoff()
        elif self.state == FlightState.SEARCH:
            self._update_search()
        elif self.state == FlightState.DESCEND:
            self._update_descend()
        # COMPLETE: do nothing, main loop will exit.

        self._update_led_indicator()

    # ==================================================================
    # Safety
    # ==================================================================

    def _check_geofence(self) -> bool:
        """Emergency LAND if the drone drifts too far from the takeoff origin.

        Returns True if geofence was breached and state was forced to COMPLETE.
        """
        if not self.telem:
            return False
        if self.state in (FlightState.IDLE, FlightState.COMPLETE):
            return False

        x = self.telem.get("pos_x_m", 0.0)
        y = self.telem.get("pos_y_m", 0.0)
        distance = math.sqrt(x ** 2 + y ** 2)

        if distance > self.MAX_DISTANCE_FROM_ORIGIN_M:
            print(
                f"[STATE] ⚠️ GEOFENCE BREACH — {distance:.2f}m from origin "
                f"(limit: {self.MAX_DISTANCE_FROM_ORIGIN_M}m). Emergency LAND."
            )
            self._send("set_mode", mode="LAND")
            self._transition_to(FlightState.COMPLETE)
            return True

        return False

    # ==================================================================
    # State handlers
    # ==================================================================

    def _update_idle(self):
        """IDLE: Wait for telemetry connection and EKF health before takeoff."""
        if not self.telem:
            return  # No telemetry received yet.

        connected = self.telem.get("connected", False)
        ekf_healthy = self.telem.get("ekf_healthy", False)

        if not connected:
            self._log_throttled("Waiting for Pixhawk connection...")
            return

        if not ekf_healthy:
            self._log_throttled("Waiting for EKF to converge...")
            return

        # Both conditions met — clear to take off.
        print("[STATE] Pixhawk connected, EKF healthy. Starting mission.")
        self._transition_to(FlightState.TAKEOFF)

    def _update_takeoff(self):
        """TAKEOFF: Arm, switch to GUIDED, command takeoff, monitor altitude.

        Uses a phase counter to sequence sub-steps. Each phase sends a command
        once and then waits for the expected telemetry confirmation.

        Phase 0: Set LOITER mode (needed for optical-flow-based arming).
        Phase 1: Send ARM command.
        Phase 2: Wait for armed confirmation, then switch to GUIDED.
        Phase 3: Send takeoff command.
        Phase 4: Monitor altitude until target reached.
        """
        elapsed = time.time() - self.state_entry_time

        # Timeout failsafe.
        if elapsed > self.timeout_takeoff:
            print("[STATE] TAKEOFF TIMEOUT — commanding LAND.")
            self._send("set_mode", mode="LAND")
            self._transition_to(FlightState.COMPLETE)
            return

        if self._takeoff_phase == 0:
            # Phase 0: Set LOITER (required for arming with optical flow).
            self._send("set_mode", mode="LOITER")
            self._takeoff_cmd_time = time.time()
            self._takeoff_phase = 1

        elif self._takeoff_phase == 1:
            # Phase 1: Wait 2 seconds for mode change, then ARM.
            if time.time() - self._takeoff_cmd_time >= 2.0:
                self._send("arm", state=True)
                self._takeoff_cmd_time = time.time()
                self._takeoff_phase = 2

        elif self._takeoff_phase == 2:
            # Phase 2: Wait for armed confirmation, then switch to GUIDED.
            if self.telem.get("armed", False):
                self._send("set_mode", mode="GUIDED")
                self._takeoff_cmd_time = time.time()
                self._takeoff_phase = 3
            elif time.time() - self._takeoff_cmd_time > 5.0:
                # Retry ARM if no response after 5 seconds.
                print("[STATE] ARM not confirmed, retrying...")
                self._send("arm", state=True)
                self._takeoff_cmd_time = time.time()

        elif self._takeoff_phase == 3:
            # Phase 3: Wait 1 second for GUIDED mode, then send takeoff.
            if time.time() - self._takeoff_cmd_time >= 1.0:
                self._send("takeoff", altitude=self.takeoff_alt)
                self._takeoff_phase = 4

        elif self._takeoff_phase == 4:
            # Phase 4: Monitor altitude.
            altitude = self._get_altitude()
            target_threshold = self.takeoff_alt * 0.9

            if altitude >= target_threshold:
                print(f"[STATE] Takeoff complete. Altitude: {altitude:.2f}m (target: {self.takeoff_alt}m)")
                self._transition_to(FlightState.SEARCH)

    def _update_search(self):
        """SEARCH: Hover at altitude and look for the landing target marker.

        Current implementation: hover in place and wait for marker 102.
        The 120-degree wide-angle lens at 2m altitude covers ~7x5m, which
        should encompass the entire 3m search radius.

        TODO: Add expanding square spiral or grid sweep pattern if the
        hover-and-look approach doesn't cover enough area.
        """
        elapsed = time.time() - self.state_entry_time

        # Timeout — land wherever we are (counts as a failed attempt).
        if elapsed > self.timeout_search:
            print("[STATE] SEARCH TIMEOUT — no marker found. Commanding LAND.")
            self._send("set_mode", mode="LAND")
            self._transition_to(FlightState.COMPLETE)
            return

        # Check vision for landing target.
        target = self.vision.get_latest_target()
        if target is not None and target["marker_id"] == self.landing_target_id:
            print(
                f"[STATE] Landing target DETECTED at offset "
                f"({target['x_offset_m']:+.3f}, {target['y_offset_m']:+.3f})m"
            )
            self._transition_to(FlightState.DESCEND)

    def _update_descend(self):
        """DESCEND: Precision landing using ArduPilot's LANDING_TARGET system.

        Instead of manually computing velocity corrections, we continuously
        feed the target's BODY_FRD position to ArduPilot via LANDING_TARGET
        messages. ArduPilot's internal precision landing PID controller
        handles horizontal alignment during the descent.

        On first detection:
            1. Send LANDING_TARGET message with target position.
            2. Send MAV_CMD_NAV_LAND to initiate the descent.

        Every subsequent tick:
            - If the marker is visible: send updated LANDING_TARGET.
            - If the marker is lost: stop sending. ArduPilot continues
              descending vertically at the last known position
              (opportunistic precision landing behavior).

        Touchdown:
            altitude < 0.15m AND disarmed → COMPLETE.
        """
        elapsed = time.time() - self.state_entry_time

        # Timeout — force completion.
        if elapsed > self.timeout_descend:
            print("[STATE] DESCEND TIMEOUT — assuming touchdown.")
            self._send("set_mode", mode="LAND")
            self._transition_to(FlightState.COMPLETE)
            return

        # Check for touchdown: altitude near zero AND disarmed.
        altitude = self._get_altitude()
        armed = self.telem.get("armed", True)

        if altitude < 0.15 and not armed:
            print(f"[STATE] TOUCHDOWN confirmed. Altitude: {altitude:.2f}m, armed: {armed}")
            self._transition_to(FlightState.COMPLETE)
            return

        # Get latest target for precision landing updates.
        target = self.vision.get_latest_target()

        if target is not None and target["marker_id"] == self.landing_target_id:
            # Convert camera-frame offsets to BODY_FRD for LANDING_TARGET message.
            # Camera Y offset (forward) → body X (forward)
            # Camera X offset (right)   → body Y (right)
            # Altitude                  → body Z (down) — must be positive
            body_x = target["y_offset_m"]     # Forward.
            body_y = target["x_offset_m"]     # Right.
            body_z = max(altitude, 0.1)       # Down (altitude, clamped to avoid z=0 error).

            if not self._land_cmd_sent:
                # First detection: send target position AND initiate LAND mode.
                self._send("land_on_target",
                           target=[body_x, body_y, body_z],
                           initiate_landing=True)
                self._land_cmd_sent = True
                print(
                    f"[STATE] Precision landing initiated. "
                    f"Target FRD: ({body_x:.3f}, {body_y:.3f}, {body_z:.3f})"
                )
            else:
                # Subsequent ticks: just update the target position.
                self._send("land_on_target",
                           target=[body_x, body_y, body_z])
        else:
            # Marker not visible — if we haven't initiated landing yet, do it blind.
            if not self._land_cmd_sent:
                self._send("set_mode", mode="LAND")
                self._land_cmd_sent = True
                print("[STATE] Marker not visible — landing at current position.")
            # If landing is already in progress and we lost the marker,
            # ArduPilot continues descending at the last known position.

    # ==================================================================
    # LED indicator
    # ==================================================================

    def _update_led_indicator(self):
        """Update LED indicator (Green = Manual control, Red = Autonomous execution)."""
        if self.led is None:
            return

        is_autonomous_state = self.state in (
            FlightState.TAKEOFF,
            FlightState.SEARCH,
            FlightState.DESCEND,
        )

        mode = self.telem.get("mode", "GUIDED") if self.telem else "GUIDED"
        is_manual_mode = mode in ("LOITER", "ALT_HOLD", "STABILIZE", "POSHOLD", "RTL", "MANUAL")

        # Autonomous execution requires autonomous state AND active guided mode.
        is_autonomous = is_autonomous_state and not is_manual_mode
        self.led.update_state(is_autonomous)

    # ==================================================================
    # Internal helpers
    # ==================================================================

    def _transition_to(self, new_state: FlightState):
        """Change state and reset per-state tracking variables."""
        old = self.state.value
        self.state = new_state
        self.state_entry_time = time.time()

        # Reset state-specific variables.
        self._takeoff_phase = 0
        self._takeoff_cmd_time = 0.0
        self._land_cmd_sent = False

        print(f"[STATE] {old} -> {new_state.value}")

    def _send(self, action: str, **kwargs):
        """Send a command to the MAVLink comm process via the command queue."""
        cmd = create_command(action, **kwargs)
        self.cmd_q.put(cmd)

    def _refresh_telemetry(self):
        """Drain the telemetry queue and keep only the latest snapshot.

        The comm process may have pushed multiple telemetry dicts since our
        last tick. We only care about the most recent one.
        """
        latest = None
        while True:
            try:
                latest = self.telem_q.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self.telem = latest

    def _get_altitude(self) -> float:
        """Return the current altitude in meters (positive = above ground).

        In NED frame, pos_z_m is negative when the drone is above the origin.
        We negate it to get a positive altitude value.
        """
        return -self.telem.get("pos_z_m", 0.0)

    def _log_throttled(self, message: str, interval: float = 3.0):
        """Print a message at most once every `interval` seconds.

        Prevents log spam during states that loop waiting for a condition.
        """
        now = time.time()
        key = message[:40]  # Use first 40 chars as a cache key.
        if not hasattr(self, "_log_cache"):
            self._log_cache = {}
        if now - self._log_cache.get(key, 0) >= interval:
            print(f"[STATE] {message}")
            self._log_cache[key] = now
