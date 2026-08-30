#!/usr/bin/env python3
"""Indomitus Drone — Application Entry Point.

Wires together the three layers of the system:
    1. VisionBridge  (ROS 2 subscriber for Simulink vision data)
    2. MissionController  (autonomous flight state machine)
    3. Comm process  (MAVLink communication with Pixhawk)

Each invocation runs a single mission cycle (IDLE → TAKEOFF → SEARCH →
DESCEND → COMPLETE). Restart the program for each mission attempt.

Usage:
    # From the repo root, with ROS 2 environment sourced:
    sudo -E python3 main.py

    # Or via the launcher script:
    ./scripts/start_mission.sh
"""

from __future__ import annotations

import multiprocessing
import os
import sys
import time

import yaml


def load_config(path: str = "config/mission_params.yaml") -> dict:
    """Load and return the mission configuration from YAML."""
    # Resolve path relative to this script's directory (not the cwd).
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, path)

    with open(config_path, "r") as f:
        config = yaml.safe_load(f)

    print(f"[MAIN] Configuration loaded from {config_path}")
    return config


def main():
    """Application entry point. Sets up all processes and runs the main loop."""

    print("=" * 60)
    print("  INDOMITUS DRONE — ERC 2026 Droning Sub-Task")
    print("  Single mission run — restart for each attempt")
    print("=" * 60)
    print()

    # ------------------------------------------------------------------
    # 1. Load configuration
    # ------------------------------------------------------------------
    config = load_config()

    # ------------------------------------------------------------------
    # 2. Startup standby delay (8 seconds — drone stationary)
    # ------------------------------------------------------------------
    start_delay_s = float(config.get("startup", {}).get("start_delay_s", 8.0))
    if start_delay_s > 0:
        print(f"[MAIN] Standby delay: waiting {start_delay_s:.0f}s before starting mission (no movement)...")
        remaining = start_delay_s
        while remaining > 0:
            step = min(1.0, remaining)
            print(f"[MAIN] Starting in {remaining:.0f}s...")
            time.sleep(step)
            remaining -= step
        print("[MAIN] Standby delay complete. Initializing mission components.\n")

    # ------------------------------------------------------------------
    # 3. Initialize ROS 2
    # ------------------------------------------------------------------
    # Import rclpy here (not at top level) because it requires the ROS 2
    # environment to be sourced. Importing at top level would crash on
    # machines without ROS 2 installed.
    try:
        import rclpy
    except ImportError:
        print("[MAIN] ERROR: rclpy not found. Is the ROS 2 environment sourced?")
        print("       Run: source /opt/ros/jazzy/setup.bash")
        sys.exit(1)

    rclpy.init()
    print("[MAIN] ROS 2 initialized.")

    # ------------------------------------------------------------------
    # 4. Create inter-process communication queues
    # ------------------------------------------------------------------
    telemetry_queue = multiprocessing.Queue()
    command_queue = multiprocessing.Queue()

    # ------------------------------------------------------------------
    # 5. Start the MAVLink communication process
    # ------------------------------------------------------------------
    from src.comm.mavlink_node import comm_process_loop

    comm_process = multiprocessing.Process(
        target=comm_process_loop,
        args=(
            telemetry_queue,
            command_queue,
            config["serial"]["port"],
            config["serial"]["baudrate"],
        ),
        daemon=True,  # Auto-terminate when main process exits.
    )
    comm_process.start()
    print(f"[MAIN] Comm process started (PID: {comm_process.pid}).")

    # ------------------------------------------------------------------
    # 6. Wait for Pixhawk initialization
    # ------------------------------------------------------------------
    init_delay = config["startup"]["pixhawk_init_delay_s"]
    print(f"[MAIN] Waiting {init_delay}s for Pixhawk heartbeat + EKF convergence...")
    time.sleep(init_delay)

    # ------------------------------------------------------------------
    # 7. Initialize LED Indicator
    # ------------------------------------------------------------------
    from src.utils.led_indicator import LEDController

    led = LEDController(config.get("led", {}))

    # ------------------------------------------------------------------
    # 8. Create the VisionBridge (ROS 2 subscriber)
    # ------------------------------------------------------------------
    vision = VisionBridge(
        topic=config["ros2"]["vision_topic"],
        state_topic=config["ros2"].get("mission_state_topic", "/erc/mission_state"),
        telemetry_topic=config["ros2"].get("telemetry_topic", "/erc/drone_telemetry"),
        grid_config=config["grid"],
    )
    print("[MAIN] VisionBridge created — subscribed to vision & state topics, publishing telemetry.")

    # ------------------------------------------------------------------
    # 9. Create the MissionController (state machine)
    # ------------------------------------------------------------------
    from src.navigation.state_machine import MissionController, FlightState

    mission = MissionController(
        command_queue=command_queue,
        telemetry_queue=telemetry_queue,
        vision_bridge=vision,
        config=config,
        led_indicator=led,
    )
    print("[MAIN] MissionController created — state machine ready.")

    # ------------------------------------------------------------------
    # 10. Main loop
    # ------------------------------------------------------------------
    print()
    print("[MAIN] Entering main loop (50 Hz).")
    print("[MAIN] Press Ctrl+C for emergency stop.\n")

    try:
        while mission.state != FlightState.COMPLETE:
            mission.update()
            time.sleep(0.02)  # 50 Hz — balances responsiveness and CPU usage.
    except ValueError as e:
        print(f"[MAIN] ERROR: {e}")
        print("[MAIN] Mission aborted due to error.")
    except KeyboardInterrupt:
        print("\n[MAIN] === EMERGENCY STOP ===")
        print("[MAIN] Sending LAND command to Pixhawk...")
        from src.comm.mavlink_node import create_command
        command_queue.put(create_command("set_mode", mode="LAND"))
        # Give the comm process a moment to send the command.
        time.sleep(1.0)

    # ------------------------------------------------------------------
    # 11. Shutdown
    # ------------------------------------------------------------------
    print("\n[MAIN] Shutting down...")

    # Clean up LED indicator.
    if led:
        led.close()

    # Clean up ROS 2.
    # rclpy may already be shut down if Ctrl+C triggered an internal cleanup,
    # so we catch the error to avoid a traceback on exit.
    try:
        vision.shutdown()
        rclpy.shutdown()
    except Exception:
        pass  # Already shut down — that's fine.
    print("[MAIN] ROS 2 shut down.")

    # Clean up comm process.
    if comm_process.is_alive():
        comm_process.terminate()
        comm_process.join(timeout=3.0)
    print("[MAIN] Comm process terminated.")

    print("[MAIN] Mission complete. Goodbye!")


if __name__ == "__main__":
    main()
