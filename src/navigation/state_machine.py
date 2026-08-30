"""Autonomous flight state machine for ERC 2026 Droning Sub-Task.

Implements a single mission cycle:
    IDLE -> TAKEOFF -> SEARCH -> DESCEND -> COMPLETE

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


# Mapping from Simulink integer state codes to Python FlightState enum.
SIMULINK_STATE_MAP: dict[int, FlightState] = {
    0: FlightState.IDLE,
    1: FlightState.TAKEOFF,
    2: FlightState.SEARCH,
    3: FlightState.DESCEND,
    4: FlightState.COMPLETE,
}


class MissionController:
    """Finite state machine controlling a single autonomous drone mission.

    Architecture:
        - Reads drone telemetry from telemetry_queue (filled by mavlink_node.py).
        - Reads vision & supervisor state from VisionBridge (filled by Simulink via ROS 2).
        - Publishes drone telemetry to Simulink on /erc/drone_telemetry.
        - Sends flight commands to command_queue (consumed by mavlink_node.py).

    Safety features:
        - Geofence: emergency BRAKE then LAND if the drone drifts >8 m from takeoff origin.
        - Timeouts: every state has a timeout that triggers controlled descent on expiry.
        - Pilot Manual Override: instantly yields control if RC flight mode is toggled.

    Args:
        command_queue: multiprocessing.Queue for sending commands to the comm process.
        telemetry_queue: multiprocessing.Queue for receiving telemetry from the comm process.
        vision_bridge: VisionBridge instance for reading vision detections.
        config: Parsed mission_params.yaml dictionary.
        led_indicator: Optional LEDController instance.
    """

    # Maximum horizontal distance from takeoff origin before emergency LAND (meters).
    MAX_DISTANCE_FROM_ORIGIN_M = 8.0

    # Consecutive valid vision detection frames required before transitioning from SEARCH to DESCEND (~100ms at 50Hz).
    SEARCH_CONFIRM_TICKS = 5

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
        self.timeout_alignment = config["timeouts"]["alignment_s"]

        # Flight parameters.
        self.takeoff_alt = config["flight"]["takeoff_altitude_m"]
        self.landing_target_id = config["markers"]["landing_target_id"]

        # Latest telemetry snapshot (refreshed every tick).
        self.telem: dict = {}

        # TAKEOFF sub-phase tracking.
        self._takeoff_phase = 0  # 0=loiter, 1=arm, 2=guided, 3=takeoff_cmd, 4=climbing
        self._takeoff_cmd_time = time.time()

        # SEARCH state tracking (consecutive detection debounce).
        self._search_detect_ticks = 0

        # DESCEND state tracking.
        self._land_cmd_sent = False

        # SEARCH state: expanding-ring serpentine search pattern.
        self._search_waypoints = self._generate_search_waypoints()
        self._search_waypoint_index = 0
        self._search_phase = 0  # 0=initial hover, 1=send waypoint, 2=wait for travel+dwell
        self._waypoint_sent_time = 0.0
        self._search_dwell_s = 3.0   # Seconds to hover at each waypoint for scanning
        self._search_travel_s = 4.0  # Seconds allocated for 1m travel between waypoints

        # Target tracking: timestamp of last marker sighting (used for descent abort).
        self._last_target_seen_time = 0.0

        # Landing target message rate limiting & cutoff during descent.
        self._last_landing_target_send_time = 0.0
        self.LANDING_TARGET_SEND_INTERVAL_S = 0.2  # 5 Hz during descent (200ms)
        self.LANDING_TARGET_MIN_ALT_M = 0.5        # Stop sending corrections below 0.5m

        print("[STATE] MissionController initialized - single mission run.")
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
            2. Processes pending ROS 2 vision & state messages.
            3. Checks for external manual override or autopilot failsafe.
            4. Checks geofence safety (BRAKE -> LAND on breach).
            5. Publishes real-time telemetry to Simulink supervisor.
            6. Synchronizes state with Simulink supervisor.
            7. Dispatches to the handler for the current state.
            8. Updates precision landing target stream (runs at 50 Hz).
            9. Updates the LED indicator state.
        """
        self._refresh_telemetry()
        self.vision.spin_once()

        # Check if pilot intervened on RC or Pixhawk entered failsafe (e.g. Battery Failsafe LAND/RTL).
        if self._check_external_override_or_failsafe():
            self._publish_telemetry_to_simulink(manual_override=True)
            return

        # Safety: emergency land if drone drifts too far from origin.
        if self._check_geofence():
            self._publish_telemetry_to_simulink(manual_override=False)
            return

        # Publish telemetry to Simulink supervisor on every tick.
        self._publish_telemetry_to_simulink(manual_override=False)

        # Synchronize with Simulink state supervisor.
        self._sync_simulink_state()

        if self.state == FlightState.IDLE:
            self._update_idle()
        elif self.state == FlightState.TAKEOFF:
            self._update_takeoff()
        elif self.state == FlightState.SEARCH:
            self._update_search()
        elif self.state == FlightState.DESCEND:
            self._update_descend()
        # COMPLETE: do nothing, main loop will exit.

        # Continuously process vision detection and MAVLink streaming on every tick.
        self._update_target()

        self._update_led_indicator()

    # ==================================================================
    # Simulink Synchronization & Telemetry
    # ==================================================================

    def _publish_telemetry_to_simulink(self, manual_override: bool = False):
        """Send current drone state to MATLAB Simulink over /erc/drone_telemetry."""
        alt = self._get_altitude()
        is_armed = bool(self.telem.get("armed", False))
        ekf_healthy = bool(self.telem.get("ekf_healthy", False))
        pos_x = self.telem.get("pos_x_m", 0.0)
        pos_y = self.telem.get("pos_y_m", 0.0)
        origin_dist = math.sqrt(pos_x ** 2 + pos_y ** 2)
        connected = bool(self.telem.get("connected", False))
        waypoints_exhausted = (self._search_waypoint_index >= len(self._search_waypoints))

        self.vision.publish_telemetry(
            alt=alt,
            is_armed=is_armed,
            ekf_healthy=ekf_healthy,
            origin_dist=origin_dist,
            connected=connected,
            manual_override=manual_override,
            waypoints_exhausted=waypoints_exhausted,
        )

    def _sync_simulink_state(self):
        """Check if Simulink supervisor commanded a state transition.

        This is the single master transition hub. All state transitions
        originate from the MATLAB supervisor.
        """
        if self.state == FlightState.COMPLETE:
            return  # Once COMPLETE, mission is finished; never accept any state changes.

        sim_state_int = self.vision.get_simulink_state()
        if sim_state_int is None:
            return

        target_state = SIMULINK_STATE_MAP.get(sim_state_int)
        if target_state is not None and target_state != self.state:
            print(f"[STATE] [SIMULINK COMMAND] State transition: {self.state.value} -> {target_state.value}")

            # Handle transition entry actions:
            if target_state == FlightState.TAKEOFF:
                self._takeoff_phase = 0
                self._takeoff_cmd_time = time.time()

            elif target_state == FlightState.SEARCH:
                # If recovering from aborted descent, skip initial hover and resume search
                if self.state == FlightState.DESCEND:
                    print("[STATE] Aborting descent — returning to GUIDED and climbing to search altitude.")
                    self._send("set_mode", mode="GUIDED")
                    current_alt = self._get_altitude()
                    climb_needed = self.takeoff_alt - current_alt
                    if climb_needed > 0.1:
                        self._send("move_local_pos", dx=0.0, dy=0.0, dz=-climb_needed)
                    self._search_phase = 1
                else:
                    self._search_phase = 0

            elif target_state == FlightState.DESCEND:
                # Entering DESCEND: check if marker was confirmed or if this is a failsafe landing
                target = self.vision.get_latest_target()
                if target is not None and target["marker_id"] == self.landing_target_id:
                    alt = self._get_altitude()
                    print(f"[STATE] Initiating Precision Landing on marker {self.landing_target_id}.")
                    self._send("land_on_target", target=[target["x_offset_m"], target["y_offset_m"], alt], initiate_landing=True)
                else:
                    print("[STATE] Failsafe / Timeout landing — commanding BRAKE -> LAND.")
                    self._send("set_mode", mode="BRAKE")
                    self._send("set_mode", mode="LAND")

            elif target_state == FlightState.COMPLETE:
                print(f"[STATE] Touchdown confirmed. Final state: COMPLETE.")

            self._transition_to(target_state)

    # ==================================================================
    # Safety
    # ==================================================================

    def _check_external_override_or_failsafe(self) -> bool:
        """Abort mission if pilot takes manual control or Pixhawk enters failsafe.

        Returns True if an override/failsafe occurred and state was forced to COMPLETE.
        """
        if not self.telem or self.state in (FlightState.IDLE, FlightState.COMPLETE):
            return False

        mode = self.telem.get("mode", "UNKNOWN")

        # 1. BRAKE is always an immediate abort in an enclosed competition area.
        if mode == "BRAKE" and self.state not in (FlightState.DESCEND, FlightState.COMPLETE):
            print(f"[STATE] [FAILSAFE/OVERRIDE] BRAKE mode detected. Aborting mission.")
            self._transition_to(FlightState.COMPLETE)
            return True

        # 2. In SEARCH: Flight mode must be GUIDED.
        if self.state == FlightState.SEARCH and mode != "GUIDED":
            print(
                f"[STATE] [OVERRIDE] Mode changed from GUIDED to {mode} during SEARCH. "
                "Aborting mission."
            )
            self._transition_to(FlightState.COMPLETE)
            return True

        # 3. In TAKEOFF (climbing phase, _takeoff_phase >= 4): Flight mode must be GUIDED.
        if self.state == FlightState.TAKEOFF and self._takeoff_phase >= 4 and mode != "GUIDED":
            print(
                f"[STATE] [OVERRIDE] Mode changed to {mode} during takeoff climb. "
                "Aborting mission."
            )
            self._transition_to(FlightState.COMPLETE)
            return True

        # 4. In DESCEND: Pilot manual takeover (switching out of GUIDED/LAND/BRAKE).
        if self.state == FlightState.DESCEND and mode not in ("GUIDED", "LAND", "BRAKE"):
            print(
                f"[STATE] [OVERRIDE] Pilot took manual control ({mode}) during descent. "
                "Aborting mission."
            )
            self._transition_to(FlightState.COMPLETE)
            return True

        return False

    def _check_geofence(self) -> bool:
        """Emergency BRAKE then LAND if the drone drifts too far from the takeoff origin.

        Returns True if geofence was breached and state was forced to DESCEND.
        """
        if not self.telem:
            return False
        if self.state in (FlightState.IDLE, FlightState.COMPLETE, FlightState.DESCEND):
            return False

        x = self.telem.get("pos_x_m", 0.0)
        y = self.telem.get("pos_y_m", 0.0)
        distance = math.sqrt(x ** 2 + y ** 2)

        if distance > self.MAX_DISTANCE_FROM_ORIGIN_M:
            print(
                f"[STATE] [GEOFENCE BREACH] {distance:.2f}m from origin "
                f"(limit: {self.MAX_DISTANCE_FROM_ORIGIN_M}m). Emergency BRAKE -> LAND."
            )
            self._send("set_mode", mode="BRAKE")
            self._send("set_mode", mode="LAND")
            self._transition_to(FlightState.DESCEND)
            return True

        return False

    # ==================================================================
    # State handlers (Execution Routines — Transitions supervised by MATLAB)
    # ==================================================================

    def _update_idle(self):
        """IDLE: Wait for telemetry connection, EKF health, and stationary origin before takeoff."""
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

        # Verify initial local position is not drifting before takeoff
        x = self.telem.get("pos_x_m", 0.0)
        y = self.telem.get("pos_y_m", 0.0)
        origin_dist = math.sqrt(x ** 2 + y ** 2)
        if origin_dist > 1.0:
            self._log_throttled(
                f"Waiting for origin to settle (current offset: {origin_dist:.2f}m > 1.0m)..."
            )
            return

        self._log_throttled("Pixhawk connected, EKF healthy, origin stable. Awaiting Simulink TAKEOFF command...")

    def _update_takeoff(self):
        """TAKEOFF: Arm, switch to GUIDED, command takeoff, monitor climb.

        Phase 0: Set LOITER mode (needed for optical-flow-based arming).
        Phase 1: Send ARM command.
        Phase 2: Wait for armed confirmation, then switch to GUIDED.
        Phase 3: Send takeoff command.
        Phase 4: Monitor climb (transition to SEARCH commanded by MATLAB when alt >= 85%).
        """
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
            # Phase 4: Actively climbing.
            altitude = self._get_altitude()
            self._log_throttled(f"Climbing... Altitude: {altitude:.2f}m / {self.takeoff_alt:.2f}m")

    def _update_search(self):
        """SEARCH: Fly an expanding-ring serpentine pattern to find the landing marker.

        The drone systematically covers the 3m competition disc using 1m steps,
        hovering at each grid point for a few seconds to let the camera detect
        the ArUco marker without motion blur.

        Pattern: center → 3×3 ring → 5×5 ring → 7×7 ring.
        Body-frame relative moves (dx=forward, dy=right), 1m each.
        """
        if self._search_phase == 0:
            # Phase 0: Initial hover at center for stabilization after climb.
            if time.time() - self.state_entry_time >= self._search_dwell_s:
                print("[STATE] Initial hover complete. Starting search pattern.")
                print(f"[STATE] Search pattern: {len(self._search_waypoints)} waypoints across 3 rings.")
                self._search_phase = 1

        elif self._search_phase == 1:
            # Phase 1: Send next waypoint command.
            if self._search_waypoint_index >= len(self._search_waypoints):
                self._log_throttled("Search pattern exhausted — awaiting Simulink command...")
                return

            dx, dy = self._search_waypoints[self._search_waypoint_index]
            ring = self._get_search_ring()
            wp = self._search_waypoint_index + 1
            total = len(self._search_waypoints)
            print(f"[STATE] Waypoint {wp}/{total} (Ring {ring}): dx={dx:+.0f}m, dy={dy:+.0f}m")

            self._send("move_local_pos", dx=float(dx), dy=float(dy), dz=0.0)
            self._waypoint_sent_time = time.time()
            self._search_phase = 2

        elif self._search_phase == 2:
            # Phase 2: Wait for travel + dwell, then advance to next waypoint.
            if time.time() - self._waypoint_sent_time >= self._search_travel_s + self._search_dwell_s:
                self._search_waypoint_index += 1
                self._search_phase = 1

    def _get_search_ring(self) -> int:
        """Return current ring number (1, 2, or 3) based on waypoint index."""
        if self._search_waypoint_index < 8:
            return 1
        if self._search_waypoint_index < 24:
            return 2
        return 3

    @staticmethod
    def _generate_search_waypoints():
        """Pre-compute the expanding-ring serpentine as relative body-frame moves.

        Each move is 1m in the body frame: dx=forward/back, dy=right/left.
        The pattern covers a 7x7 grid (3m radius) in three expanding rings:

            Ring 1:  3x3 inner grid  ( 8 moves,  9 positions incl. center)
            Ring 2:  5x5 outer band  (16 moves, 16 new positions)
            Ring 3:  7x7 outer band  (24 moves, 24 new positions)

        Total: 48 moves, 49 unique scan positions.
        """
        # Ring 1: serpentine covering 3x3 grid from center.
        ring1 = [
            (1, 0), (0, 1), (-1, 0), (-1, 0),
            (0, -1), (0, -1), (1, 0), (1, 0),
        ]

        # Ring 2: clockwise perimeter sweep of 5x5 outer band (16 new positions).
        ring2 = [
            (1, 0), (0, 1), (0, 1), (0, 1),
            (-1, 0), (-1, 0), (-1, 0), (-1, 0),
            (0, -1), (0, -1), (0, -1), (0, -1),
            (1, 0), (1, 0), (1, 0), (1, 0),
        ]

        # Ring 3: clockwise perimeter sweep of 7x7 outer band (24 new positions).
        ring3 = [
            (1, 0), (0, 1), (0, 1), (0, 1), (0, 1), (0, 1),
            (-1, 0), (-1, 0), (-1, 0), (-1, 0), (-1, 0), (-1, 0),
            (0, -1), (0, -1), (0, -1), (0, -1), (0, -1), (0, -1),
            (1, 0), (1, 0), (1, 0), (1, 0), (1, 0), (1, 0),
        ]

        return ring1 + ring2 + ring3


    def _update_target(self):
        """Process landing target detection and stream LANDING_TARGET messages at 50 Hz.

        Continuously feeds LANDING_TARGET measurements to ArduPilot during DESCEND
        so the flight controller can steer to the landing target.
        """
        target = self.vision.get_latest_target()

        if target is not None and target["marker_id"] == self.landing_target_id:
            self._last_target_seen_time = time.time()
            x_off = target["x_offset_m"]
            y_off = target["y_offset_m"]
            alt = self._get_altitude()

            now = time.time()
            if self.state == FlightState.DESCEND:
                # During DESCEND: stop corrections below cutoff altitude (commit to touchdown).
                # Above cutoff: throttle updates to 5 Hz to prevent rapid oscillation.
                if alt > self.LANDING_TARGET_MIN_ALT_M:
                    if now - self._last_landing_target_send_time >= self.LANDING_TARGET_SEND_INTERVAL_S:
                        self._send("send_landing_target", target=[x_off, y_off, alt])
                        self._last_landing_target_send_time = now
            elif self.state == FlightState.SEARCH:
                # During SEARCH: pre-warm target position stream
                self._send("send_landing_target", target=[x_off, y_off, alt])
                self._last_landing_target_send_time = now


    def _update_descend(self):
        """DESCEND: Precision landing execution.

        Continuously feeds LANDING_TARGET messages to ArduPilot (via _update_target).
        State transitions (touchdown or lost-marker recovery) are supervised by MATLAB.
        """
        altitude = self._get_altitude()
        self._log_throttled(f"Descending... Altitude: {altitude:.2f}m")


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
        self._search_detect_ticks = 0
        self._land_cmd_sent = False
        self._last_landing_target_send_time = 0.0

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
