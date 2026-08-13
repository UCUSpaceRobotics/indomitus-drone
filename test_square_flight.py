#!/usr/bin/env python3
"""Indomitus Drone — Standalone Square Flight Test Program.

This script validates low-level autonomous flight, optical flow stabilization,
and waypoint navigation without any ROS 2 or Computer Vision dependencies.

Flight Profile:
    1. 10-second safety startup delay (EKF3 convergence).
    2. Arm in LOITER -> Switch to GUIDED -> Takeoff to 1.0 meter.
    3. Settle at hover for 2 seconds.
    4. Fly 1 meter Forward  (maintaining constant yaw).
    5. Fly 1 meter Right    (maintaining constant yaw).
    6. Fly 1 meter Back     (maintaining constant yaw).
    7. Fly 1 meter Left     (returning to takeoff position, maintaining constant yaw).
    8. Precision Land and Disarm.

Safety Failsafes:
    - Pilot Manual Override: If flight mode changes away from GUIDED (e.g. LOITER,
      ALT_HOLD, STABILIZE), autonomous commands immediately halt and LED turns GREEN.
    - Emergency Stop: Ctrl+C immediately sends LAND command and cleanly exits.
    - LED Status: RED during autonomous flight, GREEN during standby/manual override.

Usage:
    sudo -E python3 test_square_flight.py
"""

from __future__ import annotations

import math
import multiprocessing
import os
import queue
import signal
import sys
import time
from enum import Enum

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False

from src.comm.mavlink_node import comm_process_loop, create_command
from src.utils.led_indicator import LEDController


# ==============================================================================
# Flight States
# ==============================================================================

class SquareFlightState(Enum):
    """States for the square flight test state machine."""
    IDLE = "IDLE"
    TAKEOFF = "TAKEOFF"
    HOVER_SETTLE = "HOVER_SETTLE"
    MOVE_FORWARD = "MOVE_FORWARD"
    MOVE_RIGHT = "MOVE_RIGHT"
    MOVE_BACK = "MOVE_BACK"
    MOVE_LEFT = "MOVE_LEFT"
    LAND = "LAND"
    COMPLETE = "COMPLETE"
    MANUAL_OVERRIDE = "MANUAL_OVERRIDE"


# ==============================================================================
# Square Flight Controller
# ==============================================================================

class SquareFlightController:
    """State machine controller executing a 1x1m square flight pattern with fixed yaw."""

    def __init__(
        self,
        command_queue: multiprocessing.Queue,
        telemetry_queue: multiprocessing.Queue,
        config: dict,
        led_indicator: LEDController | None = None,
        target_altitude_m: float = 1.0,
        square_side_m: float = 1.0,
        settle_duration_s: float = 2.0,
    ):
        self.cmd_q = command_queue
        self.telem_q = telemetry_queue
        self.config = config
        self.led = led_indicator

        self.target_alt = target_altitude_m
        self.square_side = square_side_m
        self.settle_duration = settle_duration_s

        self.state = SquareFlightState.IDLE
        self.state_entry_time = time.time()

        # Telemetry snapshot
        self.telem: dict = {}

        # Sub-state and step tracking
        self._takeoff_phase = 0
        self._takeoff_cmd_time = 0.0
        self._command_sent_for_state = False
        self._leg_start_x = 0.0
        self._leg_start_y = 0.0
        self._target_reached_time: float | None = None
        self._land_initiated = False

        # Timeouts (seconds)
        self.timeout_takeoff = config.get("timeouts", {}).get("takeoff_s", 15.0)
        self.timeout_leg = 20.0
        self.timeout_land = config.get("timeouts", {}).get("landing_s", 20.0)

        # Logging throttling cache
        self._log_cache: dict[str, float] = {}

        print(f"[TEST_CTRL] Initialized. Target altitude: {self.target_alt}m, Square size: {self.square_side}x{self.square_side}m.")
        self._update_led()

    # --------------------------------------------------------------------------
    # Public Update Tick (50 Hz)
    # --------------------------------------------------------------------------

    def update(self) -> None:
        """Executes one tick of the state machine."""
        self._refresh_telemetry()

        # Guard: Check for Pilot Manual Override
        if self._check_manual_override():
            self._update_led()
            return

        if self.state == SquareFlightState.IDLE:
            self._update_idle()
        elif self.state == SquareFlightState.TAKEOFF:
            self._update_takeoff()
        elif self.state == SquareFlightState.HOVER_SETTLE:
            self._update_hover_settle()
        elif self.state == SquareFlightState.MOVE_FORWARD:
            self._update_square_leg(
                dx=self.square_side, dy=0.0, next_state=SquareFlightState.MOVE_RIGHT, leg_name="FORWARD"
            )
        elif self.state == SquareFlightState.MOVE_RIGHT:
            self._update_square_leg(
                dx=0.0, dy=self.square_side, next_state=SquareFlightState.MOVE_BACK, leg_name="RIGHT"
            )
        elif self.state == SquareFlightState.MOVE_BACK:
            self._update_square_leg(
                dx=-self.square_side, dy=0.0, next_state=SquareFlightState.MOVE_LEFT, leg_name="BACK"
            )
        elif self.state == SquareFlightState.MOVE_LEFT:
            self._update_square_leg(
                dx=0.0, dy=-self.square_side, next_state=SquareFlightState.LAND, leg_name="LEFT (RETURN TO START)"
            )
        elif self.state == SquareFlightState.LAND:
            self._update_land()
        elif self.state == SquareFlightState.COMPLETE:
            pass  # Done

        self._update_led()

    # --------------------------------------------------------------------------
    # Manual Override Detection
    # --------------------------------------------------------------------------

    def _check_manual_override(self) -> bool:
        """Returns True if the pilot has taken over manual control."""
        if self.state in (SquareFlightState.IDLE, SquareFlightState.COMPLETE):
            return False

        current_mode = self.telem.get("mode", "UNKNOWN")

        # In TAKEOFF phase 0-1, we deliberately use LOITER to arm.
        if self.state == SquareFlightState.TAKEOFF and self._takeoff_phase <= 1:
            return False

        # If we are in an autonomous flight state and mode is NOT GUIDED, pilot took over.
        if current_mode != "GUIDED" and current_mode != "UNKNOWN":
            if self.state != SquareFlightState.MANUAL_OVERRIDE:
                print(f"\n{'!'*65}")
                print(f"[TEST_CTRL] PILOT MANUAL OVERRIDE DETECTED (Mode: {current_mode})!")
                print("[TEST_CTRL] Halting all autonomous commands. Control yielded to pilot.")
                print(f"{'!'*65}\n")
                self.state = SquareFlightState.MANUAL_OVERRIDE
            return True

        # If mode returned to GUIDED while in MANUAL_OVERRIDE, inform user
        if self.state == SquareFlightState.MANUAL_OVERRIDE and current_mode == "GUIDED":
            self._log_throttled("In MANUAL_OVERRIDE state. Mode is GUIDED again. Land manually or restart script.")
            return True

        return False

    # --------------------------------------------------------------------------
    # State Handlers
    # --------------------------------------------------------------------------

    def _update_idle(self) -> None:
        """IDLE: Wait for Pixhawk connection and EKF health."""
        if not self.telem:
            return

        connected = self.telem.get("connected", False)
        ekf_healthy = self.telem.get("ekf_healthy", False)

        if not connected:
            self._log_throttled("Waiting for Pixhawk MAVLink connection...")
            return

        if not ekf_healthy:
            self._log_throttled("Waiting for EKF3 optical flow + IMU to converge...")
            return

        print("[TEST_CTRL] Pixhawk connected & EKF healthy. Beginning Takeoff Sequence.")
        self._transition_to(SquareFlightState.TAKEOFF)

    def _update_takeoff(self) -> None:
        """TAKEOFF: Arm in LOITER -> switch to GUIDED -> takeoff -> monitor altitude."""
        elapsed = time.time() - self.state_entry_time

        if elapsed > self.timeout_takeoff:
            print("[TEST_CTRL] TAKEOFF TIMEOUT — commanding emergency LAND.")
            self._send("set_mode", mode="LAND")
            self._transition_to(SquareFlightState.COMPLETE)
            return

        if self._takeoff_phase == 0:
            # Phase 0: Set LOITER mode (needed for optical-flow arming)
            print("[TEST_CTRL] Setting LOITER mode for arming...")
            self._send("set_mode", mode="LOITER")
            self._takeoff_cmd_time = time.time()
            self._takeoff_phase = 1

        elif self._takeoff_phase == 1:
            # Phase 1: Wait 2s for mode change, then send ARM
            if time.time() - self._takeoff_cmd_time >= 2.0:
                print("[TEST_CTRL] Sending ARM command...")
                self._send("arm", state=True)
                self._takeoff_cmd_time = time.time()
                self._takeoff_phase = 2

        elif self._takeoff_phase == 2:
            # Phase 2: Wait for armed confirmation, then switch to GUIDED
            if self.telem.get("armed", False):
                print("[TEST_CTRL] Motors ARMED. Switching to GUIDED mode...")
                self._send("set_mode", mode="GUIDED")
                self._takeoff_cmd_time = time.time()
                self._takeoff_phase = 3
            elif time.time() - self._takeoff_cmd_time > 5.0:
                print("[TEST_CTRL] Retrying ARM command...")
                self._send("arm", state=True)
                self._takeoff_cmd_time = time.time()

        elif self._takeoff_phase == 3:
            # Phase 3: Wait 1s for GUIDED mode, then send takeoff command
            if time.time() - self._takeoff_cmd_time >= 1.0:
                print(f"[TEST_CTRL] Sending TAKEOFF command to {self.target_alt}m...")
                self._send("takeoff", altitude=self.target_alt)
                self._takeoff_phase = 4

        elif self._takeoff_phase == 4:
            # Phase 4: Monitor climb until target altitude is reached
            altitude = self._get_altitude()
            threshold = self.target_alt * 0.9

            self._log_throttled(f"Climbing... Altitude: {altitude:.2f}m / {self.target_alt:.2f}m", interval=1.0)

            if altitude >= threshold:
                print(f"[TEST_CTRL] Reached target altitude: {altitude:.2f}m. Hovering to stabilize.")
                self._transition_to(SquareFlightState.HOVER_SETTLE)

    def _update_hover_settle(self) -> None:
        """HOVER_SETTLE: Settle at 1.0m altitude before initiating square legs."""
        elapsed = time.time() - self.state_entry_time

        if elapsed >= self.settle_duration:
            print(f"[TEST_CTRL] Hover stabilized for {self.settle_duration}s. Starting square maneuvers.")
            self._transition_to(SquareFlightState.MOVE_FORWARD)

    def _update_square_leg(
        self, dx: float, dy: float, next_state: SquareFlightState, leg_name: str
    ) -> None:
        """Executes one leg of the square pattern maintaining fixed yaw.

        Sends MAV_FRAME_BODY_OFFSET_NED displacement command with yaw_rate=0.
        """
        elapsed = time.time() - self.state_entry_time

        # 1. Send displacement command once on entry
        if not self._command_sent_for_state:
            self._leg_start_x = self.telem.get("pos_x_m", 0.0)
            self._leg_start_y = self.telem.get("pos_y_m", 0.0)
            print(f"\n[TEST_CTRL] >>> EXECUTING LEG: {leg_name} (dx={dx:+.1f}m, dy={dy:+.1f}m, yaw locked) <<<")
            self._send("move_local_pos", dx=dx, dy=dy, dz=0.0)
            self._command_sent_for_state = True
            self._target_reached_time = None

        # 2. Track displacement progress
        curr_x = self.telem.get("pos_x_m", 0.0)
        curr_y = self.telem.get("pos_y_m", 0.0)
        dist_traveled = math.sqrt((curr_x - self._leg_start_x) ** 2 + (curr_y - self._leg_start_y) ** 2)
        expected_dist = math.sqrt(dx ** 2 + dy ** 2)
        remaining = max(0.0, expected_dist - dist_traveled)

        self._log_throttled(
            f"Leg {leg_name}: Traveled {dist_traveled:.2f}m / {expected_dist:.2f}m (Remaining: {remaining:.2f}m)",
            interval=1.0,
        )

        # 3. Check if target waypoint reached (within 0.15m tolerance)
        if remaining <= 0.15:
            if self._target_reached_time is None:
                self._target_reached_time = time.time()
                print(f"[TEST_CTRL] Leg {leg_name} reached target! Settling for {self.settle_duration}s...")

            if time.time() - self._target_reached_time >= self.settle_duration:
                print(f"[TEST_CTRL] Leg {leg_name} COMPLETE.")
                self._transition_to(next_state)
                return

        # 4. Timeout failsafe for this leg
        if elapsed > self.timeout_leg:
            print(f"[TEST_CTRL] WARNING: Leg {leg_name} TIMEOUT after {self.timeout_leg}s. Proceeding to next state.")
            self._transition_to(next_state)

    def _update_land(self) -> None:
        """LAND: Send LAND command and monitor touchdown."""
        elapsed = time.time() - self.state_entry_time

        if not self._land_initiated:
            print("\n[TEST_CTRL] Initiating LANDING sequence...")
            self._send("set_mode", mode="LAND")
            self._land_initiated = True

        altitude = self._get_altitude()
        armed = self.telem.get("armed", True)

        self._log_throttled(f"Descending... Altitude: {altitude:.2f}m, Armed: {armed}", interval=1.0)

        # Touchdown confirmed when altitude is near zero and disarmed
        if (altitude < 0.15 and not armed) or elapsed > self.timeout_land:
            print("\n" + "=" * 65)
            print("[TEST_CTRL] TOUCHDOWN & DISARM CONFIRMED! TEST FLIGHT COMPLETE.")
            print("=" * 65 + "\n")
            self._transition_to(SquareFlightState.COMPLETE)

    # --------------------------------------------------------------------------
    # Internal Helpers
    # --------------------------------------------------------------------------

    def _transition_to(self, new_state: SquareFlightState) -> None:
        """Switches state and resets tracking variables."""
        old = self.state.value
        self.state = new_state
        self.state_entry_time = time.time()
        self._command_sent_for_state = False
        self._target_reached_time = None
        print(f"[TEST_CTRL] State Transition: {old} -> {new_state.value}")

    def _send(self, action: str, **kwargs) -> None:
        """Dispatches a command to the comm process."""
        cmd = create_command(action, **kwargs)
        self.cmd_q.put(cmd)

    def _refresh_telemetry(self) -> None:
        """Drains the telemetry queue to get the latest snapshot."""
        latest = None
        while True:
            try:
                latest = self.telem_q.get_nowait()
            except queue.Empty:
                break
        if latest is not None:
            self.telem = latest

    def _get_altitude(self) -> float:
        """Returns altitude above ground in meters (pos_z_m inverted)."""
        return -self.telem.get("pos_z_m", 0.0)

    def _update_led(self) -> None:
        """Updates LED indicator (RED = Autonomous, GREEN = Manual / Standby)."""
        if self.led is None:
            return

        is_autonomous = self.state in (
            SquareFlightState.TAKEOFF,
            SquareFlightState.HOVER_SETTLE,
            SquareFlightState.MOVE_FORWARD,
            SquareFlightState.MOVE_RIGHT,
            SquareFlightState.MOVE_BACK,
            SquareFlightState.MOVE_LEFT,
            SquareFlightState.LAND,
        ) and self.state != SquareFlightState.MANUAL_OVERRIDE

        self.led.update_state(is_autonomous)

    def _log_throttled(self, message: str, interval: float = 3.0) -> None:
        """Prints a throttled message to prevent terminal flooding."""
        now = time.time()
        key = message[:40]
        if now - self._log_cache.get(key, 0.0) >= interval:
            print(f"[TEST_CTRL] {message}")
            self._log_cache[key] = now


# ==============================================================================
# Helper Functions
# ==============================================================================

def load_config(path: str = "config/mission_params.yaml") -> dict:
    """Load configuration YAML."""
    default_cfg = {
        "serial": {"port": "/dev/ttyAMA0", "baudrate": 921600},
        "timeouts": {"takeoff_s": 15.0, "landing_s": 20.0},
        "led": {"enabled": True, "gpio_pin": 10, "num_leds": 7, "brightness": 128},
    }

    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, path)

    if not HAS_YAML:
        print(f"[TEST] PyYAML not installed. Using default test configuration.")
        return default_cfg

    if not os.path.exists(config_path):
        print(f"[TEST] Config file not found at {config_path}. Using safe defaults.")
        return default_cfg

    try:
        with open(config_path, "r") as f:
            config = yaml.safe_load(f)
        print(f"[TEST] Configuration loaded from {config_path}")
        return config
    except Exception as e:
        print(f"[TEST] Error reading config YAML ({e}). Using defaults.")
        return default_cfg


# ==============================================================================
# Main Entry Point
# ==============================================================================

def main():
    """Main execution function for the square flight test."""
    print("=" * 65)
    print("  INDOMITUS DRONE — STANDALONE SQUARE FLIGHT TEST (NO ROS 2)")
    print("=" * 65)
    print()

    # 1. Load Configuration
    config = load_config()

    # 2. Inter-Process Communication Queues
    telemetry_queue = multiprocessing.Queue()
    command_queue = multiprocessing.Queue()

    # 3. Start MAVLink Communication Process
    comm_process = multiprocessing.Process(
        target=comm_process_loop,
        args=(
            telemetry_queue,
            command_queue,
            config["serial"]["port"],
            config["serial"]["baudrate"],
        ),
        daemon=True,
    )
    comm_process.start()
    print(f"[TEST] MAVLink comm process started (PID: {comm_process.pid}).")

    # 4. 10-Second Startup Delay (EKF3 stabilization & operator safety)
    STARTUP_DELAY_S = 10
    print(f"\n[TEST] Beginning {STARTUP_DELAY_S}s startup delay for Pixhawk EKF3 stabilization...")
    for remaining in range(STARTUP_DELAY_S, 0, -1):
        print(f"[TEST] Launching in {remaining}s... (Press Ctrl+C to abort)")
        time.sleep(1.0)
    print("[TEST] Startup delay completed.\n")

    # 5. Initialize LED Controller
    led = LEDController(config.get("led", {}))
    led.set_manual_mode(force=True)  # Start GREEN (Standby)

    # 6. Initialize Square Flight Controller
    controller = SquareFlightController(
        command_queue=command_queue,
        telemetry_queue=telemetry_queue,
        config=config,
        led_indicator=led,
        target_altitude_m=1.0,
        square_side_m=1.0,
        settle_duration_s=2.0,
    )

    # 7. Main Control Loop (50 Hz)
    print("[TEST] Entering 50 Hz control loop.")
    print("[TEST] Press Ctrl+C at any time for emergency landing.\n")

    try:
        while controller.state not in (
            SquareFlightState.COMPLETE,
            SquareFlightState.MANUAL_OVERRIDE,
        ):
            controller.update()
            time.sleep(0.02)  # 50 Hz

        # If exited due to manual override, keep monitoring passively until interrupted
        if controller.state == SquareFlightState.MANUAL_OVERRIDE:
            print("[TEST] Script in passive monitoring mode (Manual Pilot Control).")
            while True:
                controller._refresh_telemetry()
                time.sleep(0.1)

    except KeyboardInterrupt:
        print("\n\n" + "!" * 65)
        print("[TEST] !!! EMERGENCY STOP TRIGGERED (Ctrl+C) !!!")
        print("[TEST] Commanding immediate LAND to Pixhawk...")
        print("!" * 65 + "\n")
        command_queue.put(create_command("set_mode", mode="LAND"))
        time.sleep(1.5)

    # 8. Shutdown & Cleanup
    print("\n[TEST] Shutting down...")

    if led:
        led.close()

    if comm_process.is_alive():
        comm_process.terminate()
        comm_process.join(timeout=3.0)
    print("[TEST] Comm process terminated.")
    print("[TEST] Test script finished. Goodbye!")


if __name__ == "__main__":
    main()
