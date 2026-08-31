import time
import queue

# --- Configuration Constants ---
GCS_HEARTBEAT_INTERVAL_SEC = 1.0  # How often to tell Pixhawk we are alive
TELEMETRY_PUBLISH_RATE_HZ = (
    10.0  # How often to push telemetry to the rest of the system
)
COMMAND_MAX_AGE_SEC = 0.5  # Drop commands older than 500ms


def comm_process_loop(
    telemetry_queue, command_queue, connection_string="/dev/ttyAMA0", baudrate=921600
):
    """
    Main loop for the MAVLink communication process.
    Runs isolated from the main FastAPI/Computer Vision loops.

    Parameters:
    telemetry_queue (multiprocessing.Queue): Queue to push fresh telemetry OUT.
    command_queue (multiprocessing.Queue): Queue to receive commands IN.
    connection_string (str): Serial port path.
    baudrate (int): Baud rate for the serial connection.
    """
    print("[COMM_NODE] Starting MAVLink communication process...")

    try:
        from src.comm.mavlink_client import PixhawkClient

        # 1. Initialize hardware connection
        client = PixhawkClient(connection_string, baudrate)

        # 2. Block until drone is online
        if not client.wait_for_heartbeat(timeout=15.0):
            print("[COMM_NODE] CRITICAL: Drone not responding. Exiting process.")
            return

        # 3. Setup data streams (10Hz general telemetry, 20Hz pose)
        client.request_data_streams(rate_hz=10)
        client.request_pose_stream(rate_hz=20)

    except Exception as e:
        print(f"[COMM_NODE] FATAL hardware initialization error: {e}")
        return

    # Timers for loop management
    last_heartbeat_time = 0.0
    last_telemetry_pub_time = 0.0
    telemetry_interval_sec = 1.0 / TELEMETRY_PUBLISH_RATE_HZ

    print("[COMM_NODE] Entering main operation loop.")

    # 4. Main Event Loop
    while True:
        try:
            current_time = time.time()

            # ---------------------------------------------------------
            # A. HARDWARE MAINTENANCE (GCS Heartbeat)
            # ---------------------------------------------------------
            if (current_time - last_heartbeat_time) >= GCS_HEARTBEAT_INTERVAL_SEC:
                client.send_gcs_heartbeat()
                last_heartbeat_time = current_time

            # ---------------------------------------------------------
            # B. READ TELEMETRY FROM DRONE
            # ---------------------------------------------------------
            # This is non-blocking and processes all available UART bytes
            current_telemetry = client.get_telemetry_tick()

            # Publish telemetry to the rest of the system at a fixed rate
            if (current_time - last_telemetry_pub_time) >= telemetry_interval_sec:
                # Best Practice: Empty the queue first so the State Machine
                # always gets the freshest data, avoiding backlog latency.
                while not telemetry_queue.empty():
                    try:
                        telemetry_queue.get_nowait()
                    except queue.Empty:
                        break

                # Push a copy of the latest dictionary
                telemetry_queue.put_nowait(current_telemetry.copy())
                last_telemetry_pub_time = current_time

            # ---------------------------------------------------------
            # C. PROCESS COMMANDS FROM STATE MACHINE
            # ---------------------------------------------------------
            while not command_queue.empty():
                try:
                    cmd = command_queue.get_nowait()
                except queue.Empty:
                    break

                # --- 1. Timestamp Validation (Stale Command Filter) ---
                cmd_timestamp = cmd.get("timestamp", 0.0)
                cmd_age = current_time - cmd_timestamp

                if cmd_age > COMMAND_MAX_AGE_SEC:
                    print(
                        f"[COMM_NODE] WARNING: Dropped stale command '{cmd.get('action')}' (Age: {cmd_age:.3f}s)"
                    )
                    continue  # Skip execution, go to next command

                # --- 2. Command Dispatcher ---
                dispatch_command(client, cmd, telem=current_telemetry)

            # ---------------------------------------------------------
            # D. CPU RELIEF (Yield execution)
            # ---------------------------------------------------------
            time.sleep(0.005)  # 5ms sleep prevents 100% core usage

        except KeyboardInterrupt:
            print("[COMM_NODE] Process interrupted by user.")
            break
        except Exception as e:
            print(f"[COMM_NODE] Unexpected error in main loop: {e}")
            time.sleep(1)  # Prevent log spamming on failure


def dispatch_command(client, cmd, telem):
    action = cmd.get("action")

    if telem.get("armed", False) is False and action not in ["arm", "set_mode"]:
        raise ValueError(f"[COMM_NODE] ERROR: Ignoring command '{action}' because drone is not armed.")

    if telem.get("mode") != "GUIDED" and action not in ["arm", "set_mode"]:
        if telem.get("mode") is None:
            raise ValueError(f"[COMM_NODE] ERROR: Ignoring command '{action}' because drone mode is unknown.")
        if telem.get("mode") == "LAND" and action in ["send_landing_target", "land_on_target"]:
            ... # Ignore and proceed with landing commands even if in LAND mode
        else:
            raise ValueError(f"[COMM_NODE] ERROR: Ignoring command '{action}' because drone is not in GUIDED mode.")

    if action == "arm":
        client.arm(state=cmd.get("state", True))

    elif action == "set_mode":
        client.set_mode(mode_name=cmd.get("mode", "GUIDED"))

    elif action == "takeoff":
        client.takeoff(altitude_m=cmd.get("altitude", 2.0))

    elif action == "land":
        client.land()

    elif action == "move_local_pos":
        client.send_position_target_local_ned(
            dx_m=cmd.get("dx", 0.0),
            dy_m=cmd.get("dy", 0.0),
            dz_m=cmd.get("dz", 0.0),
            yaw_rad=cmd.get("yaw_rad"),
        )

    elif action == "set_local_position":
        client.send_local_ned_position_target(
            x_m=cmd.get("x", 0.0),
            y_m=cmd.get("y", 0.0),
            z_m=cmd.get("z", 0.0),
        )

    elif action == "move_local_vel":
        client.send_velocity_target_body_ned(
            vx_m_s=cmd.get("vx", 0.0),
            vy_m_s=cmd.get("vy", 0.0),
            vz_m_s=cmd.get("vz", 0.0),
        )

    elif action == "land_on_target":
        target = cmd.get("target")

        if target is None:
            print("[COMM_NODE] ERROR: land_on_target command has no target")
            return

        client.send_landing_target(tuple(target), cmd.get("initiate_landing", True))
    elif action == "send_landing_target":
        target = cmd.get("target")

        if target is None:
            print("[COMM_NODE] ERROR: send_landing_target command has no target")
            return

        client.send_landing_target(tuple(target), cmd.get("initiate_landing", False))
    else:
        print(f"[COMM_NODE] ERROR: Unknown command action: {action}")


# --- Helper for the Navigation Module ---
def create_command(action, **kwargs):
    """
    Helper function to be used by the State Machine to format commands.
    Automatically injects the current timestamp.

    Usage:
        cmd = create_command("move_local_vel", vx=0.5, vy=0.0, vz=0.0)
        command_queue.put(cmd)
    """
    command = {"action": action, "timestamp": time.time()}
    command.update(kwargs)
    return command
