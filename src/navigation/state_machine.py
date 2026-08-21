"""Autonomous flight state machine for ERC 2026 Droning Sub-Task.

Implements the mission cycle: IDLE -> TAKEOFF -> SEARCH -> ALIGN -> LAND -> COMPLETE.
Repeated 3 times, then MISSION_DONE.

The state machine is driven by a single update() method called ~50 times/second
by main.py. Each call reads telemetry and vision, does the work for the current
state, and checks if the exit condition is met.

Usage:
    mission = MissionController(command_queue, telemetry_queue, vision_bridge, config)
    while mission.state != FlightState.MISSION_DONE:
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
    ALIGN = "ALIGN"
    LAND = "LAND"
    COMPLETE = "COMPLETE"
    MISSION_DONE = "MISSION_DONE"


class MissionController:
    """Finite state machine controlling the autonomous drone mission.

    Architecture:
        - Reads drone telemetry from telemetry_queue (filled by mavlink_node.py).
        - Reads vision data from VisionBridge (filled by Simulink via ROS 2).
        - Sends flight commands to command_queue (consumed by mavlink_node.py).

    Args:
        command_queue: multiprocessing.Queue for sending commands to the comm process.
        telemetry_queue: multiprocessing.Queue for receiving telemetry from the comm process.
        vision_bridge: VisionBridge instance for reading vision detections.
        config: Parsed mission_params.yaml dictionary.
    """

    # ALIGN controller gain. Start conservative — increase if convergence is too slow,
    # decrease if the drone oscillates. The Pixhawk's internal GUIDED-mode PID
    # adds additional damping, so P-only may be sufficient.
    ALIGN_KP = 0.5  # m/s per meter of offset

    # How close to center (in meters) the marker must be to count as "aligned".
    ALIGN_CENTERED_THRESHOLD_M = 0.05

    # How many consecutive update() ticks the drone must remain centered
    # before transitioning to LAND. At 50 Hz, 50 ticks = 1 second.
    ALIGN_CENTERED_TICKS_REQUIRED = 50

    # How long the marker can be missing before we consider it lost (seconds).
    ALIGN_LOST_TIMEOUT_S = 0.5

    def __init__(self, command_queue, telemetry_queue, vision_bridge, config: dict, led_indicator=None):
        self.cmd_q = command_queue
        self.telem_q = telemetry_queue
        self.vision = vision_bridge
        self.config = config
        self.led = led_indicator

        # Current state.
        self.state = FlightState.IDLE
        self.state_entry_time = time.time()

        # Mission progress.
        self.attempt = 0  # Incremented AFTER each landing (1, 2, 3).
        self.num_landings = config["mission"]["num_landings"]

        # Timeouts (seconds).
        self.timeout_takeoff = config["timeouts"]["takeoff_s"]
        self.timeout_search = config["timeouts"]["search_sweep_s"]
        self.timeout_align = config["timeouts"]["alignment_s"]
        self.timeout_land = config["timeouts"]["landing_s"]

        # Flight parameters.
        self.takeoff_alt = config["flight"]["takeoff_altitude_m"]
        self.landing_target_id = config["markers"]["landing_target_id"]

        # Latest telemetry snapshot (refreshed every tick).
        self.telem: dict = {}

        # TAKEOFF sub-phase tracking.
        self._takeoff_phase = 0  # 0=arm, 1=guided, 2=takeoff_cmd, 3=climbing
        self._takeoff_cmd_time = 0.0

        # ALIGN state tracking.
        self._centered_ticks = 0

        # LAND state tracking.
        self._land_initiated = False

        print(f"[STATE] MissionController initialized. Target: {self.num_landings} landings.")
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
            3. Dispatches to the handler for the current state.
            4. Updates the LED indicator state.
        """
        self._refresh_telemetry()
        self.vision.spin_once()

        if self.state == FlightState.IDLE:
            self._update_idle()
        elif self.state == FlightState.TAKEOFF:
            self._update_takeoff()
        elif self.state == FlightState.SEARCH:
            self._update_search()
        elif self.state == FlightState.ALIGN:
            self._update_align()
        elif self.state == FlightState.LAND:
            self._update_land()
        elif self.state == FlightState.COMPLETE:
            self._update_complete()
        # MISSION_DONE: do nothing, main loop will exit.

        self._update_led_indicator()

    def _update_led_indicator(self):
        """Update LED indicator (Green = Manual control, Red = Autonomous execution)."""
        if self.led is None:
            return

        is_autonomous_state = self.state in (
            FlightState.TAKEOFF,
            FlightState.SEARCH,
            FlightState.ALIGN,
            FlightState.LAND,
        )

        mode = self.telem.get("mode", "GUIDED") if self.telem else "GUIDED"
        is_manual_mode = mode in ("LOITER", "ALT_HOLD", "STABILIZE", "POSHOLD", "RTL", "MANUAL")

        # Autonomous execution requires autonomous state AND active guided mode
        is_autonomous = is_autonomous_state and not is_manual_mode
        self.led.update_state(is_autonomous)

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
        print(f"[STATE] Pixhawk connected, EKF healthy. Starting attempt {self.attempt + 1}/{self.num_landings}.")
        self.vision.clear_probes()  # Fresh probe accumulation for this attempt.
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
            self._transition_to(FlightState.LAND)

    # def _update_align(self):
    #     """ALIGN: Center the drone over the landing target using velocity corrections.

    #     Control law (proportional):
    #         vx = Kp * y_offset   (camera Y → body forward)
    #         vy = Kp * x_offset   (camera X → body right)

    #     Transition to LAND when centered within threshold for 1 full second.
    #     Fall back to SEARCH if the marker is lost for > 0.5 seconds.
    #     """
    #     elapsed = time.time() - self.state_entry_time

    #     # Timeout — land anyway (imprecise landing is better than no landing).
    #     if elapsed > self.timeout_align:
    #         print("[STATE] ALIGN TIMEOUT — landing at current position.")
    #         self._transition_to(FlightState.LAND)
    #         return

    #     target = self.vision.get_latest_target()

    #     # Lost target check.
    #     if target is None or target["marker_id"] != self.landing_target_id:
    #         if target is None:
    #             # Check if we've been without a detection for too long.
    #             self._centered_ticks = 0
    #             # Vision bridge returns None when detection age > 0.5s,
    #             # so if we get None, the marker has been gone long enough.
    #             print("[STATE] Marker lost — returning to SEARCH.")
    #             self._transition_to(FlightState.SEARCH)
    #             return
    #         # Detected a different marker (e.g., 101) — ignore it, keep aligning.
    #         return

    #     # Calculate velocity corrections.
    #     x_offset = target["x_offset_m"]  # Right of camera center.
    #     y_offset = target["y_offset_m"]  # Forward of camera center.

    #     vx = self.ALIGN_KP * y_offset   # Forward/back body velocity.
    #     vy = self.ALIGN_KP * x_offset   # Left/right body velocity.
    #     vz = 0.0                         # Hold altitude.

    #     # Clamp velocities to configured max.
    #     max_speed = self.config["flight"]["max_horizontal_speed_m_s"]
    #     vx = max(-max_speed, min(max_speed, vx))
    #     vy = max(-max_speed, min(max_speed, vy))

    #     # Send velocity command (must be sent continuously at high rate).
    #     self._send("move_local_vel", vx=vx, vy=vy, vz=vz)

    #     # Check if centered.
    #     error = math.sqrt(x_offset ** 2 + y_offset ** 2)
    #     if error < self.ALIGN_CENTERED_THRESHOLD_M:
    #         self._centered_ticks += 1
    #     else:
    #         self._centered_ticks = 0

    #     # Stable for long enough — transition to LAND.
    #     if self._centered_ticks >= self.ALIGN_CENTERED_TICKS_REQUIRED:
    #         print(
    #             f"[STATE] ALIGNED — centered within {self.ALIGN_CENTERED_THRESHOLD_M}m "
    #             f"for {self.ALIGN_CENTERED_TICKS_REQUIRED / 50:.1f}s. Initiating landing."
    #         )
    #         self._transition_to(FlightState.LAND)

    def _update_land(self):
        """LAND: Execute precision landing using LANDING_TARGET messages.

        Continuously sends the marker's BODY_FRD position to the Pixhawk.
        If the marker is lost, ArduCopter continues landing at the last
        known position (opportunistic precision landing).
        """
        elapsed = time.time() - self.state_entry_time

        # Timeout — force completion.
        if elapsed > self.timeout_land:
            print("[STATE] LAND TIMEOUT — assuming touchdown.")
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

            target_frd = (body_x, body_y, body_z)

            if not self._land_initiated:
                # First call — initiate landing mode AND send target.
                self._send("land_on_target", target=list(target_frd))
                # Also explicitly set LAND mode in case land_on_target
                # doesn't trigger it on the first try.
                self._land_initiated = True
                print(f"[STATE] Precision landing initiated. Target FRD: ({body_x:.3f}, {body_y:.3f}, {body_z:.3f})")
            else:
                # Subsequent calls — just update the target position.
                self._send("land_on_target", target=list(target_frd))
        else:
            # Marker not visible — if we haven't initiated landing yet, do it blind.
            if not self._land_initiated:
                self._send("land")
                self._land_initiated = True
                print("[STATE] Marker not visible — landing at current position.")

    def _update_complete(self):
        """COMPLETE: Post-landing cleanup. Log results and prepare for next cycle."""
        self.attempt += 1
        probes = self.vision.get_detected_probes()

        print(f"\n{'='*60}")
        print(f"[STATE] ATTEMPT {self.attempt}/{self.num_landings} COMPLETE")
        if probes:
            print(f"[STATE] Probes detected in sectors: {', '.join(probes)}")
        else:
            print("[STATE] No probes detected this attempt.")
        print(f"{'='*60}\n")

        if self.attempt >= self.num_landings:
            print("[STATE] All landing attempts finished. MISSION DONE.")
            # Final probe report.
            all_probes = self.vision.get_detected_probes()
            if all_probes:
                print(f"[STATE] FINAL PROBE REPORT: {', '.join(all_probes)}")
            self._transition_to(FlightState.MISSION_DONE)
        else:
            print(
                f"[STATE] Waiting for manual reposition to takeoff pad. "
                f"({self.num_landings - self.attempt} attempts remaining)"
            )
            # Don't clear probes here — probes accumulate across the whole mission.
            self._transition_to(FlightState.IDLE)

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
        self._centered_ticks = 0
        self._land_initiated = False

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
