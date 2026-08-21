"""Low-level MAVLink communication client for ArduPilot (Pixhawk)."""

import os
import time
import math

# Force MAVLink 2.0 protocol before importing pymavlink tools
os.environ["MAVLINK20"] = "1"

from pymavlink import mavutil


class PixhawkClient:
    """
    Low-level MAVLink communication client for ArduPilot (Pixhawk).
    Handles direct serial connection, telemetry parsing, and command dispatching.
    """

    def __init__(self, connection_string="/dev/ttyAMA0", baudrate=921600):
        """Initializes the serial connection to the Pixhawk."""
        print(
            f"[COMM] Initializing MAVLink connection on {connection_string} @ {baudrate}..."
        )

        # We set source_system=255 and source_component=0 to identify this RPi as a GCS
        # (Ground Control Station). This is crucial for the Pixhawk's failsafe logic.
        self.connection = mavutil.mavlink_connection(
            connection_string, baud=baudrate, source_system=255, source_component=0
        )

        # Internal state dictionary to hold the latest telemetry
        self.telemetry = {
            "connected": False,
            "armed": False,
            "mode": "UNKNOWN",
            "battery_voltage_v": 0.0,
            "battery_remaining_pct": 0,
            "ekf_flags": 0,
            "ekf_healthy": False,
            "rc_rssi": None,
            "rc_channel_count": 0,
            "rc_link_live": False,
            "pos_x_m": 0.0,  # Local North
            "pos_y_m": 0.0,  # Local East
            "pos_z_m": 0.0,  # Local Down (Negative is UP)
            "vel_x_m_s": 0.0,
            "vel_y_m_s": 0.0,
            "vel_z_m_s": 0.0,
            "roll_rad": 0.0,
            "pitch_rad": 0.0,
            "yaw_rad": 0.0,
            "last_local_position_time": 0.0,
            "last_attitude_time": 0.0,
            "last_heartbeat_time": 0.0,
            "last_ekf_time": 0.0,
            "last_rc_channels_time": 0.0,
            "landed_state": "undefined",
            "last_landed_state_time": 0.0,
        }
        self._protocol_events = []

    def wait_for_heartbeat(self, timeout=10.0):
        """
        Blocks execution until the first heartbeat is received from the Pixhawk.
        This establishes the target_system and target_component IDs.
        """
        print("[COMM] Waiting for Pixhawk heartbeat...")
        deadline = time.monotonic() + timeout
        msg = None
        while time.monotonic() < deadline:
            candidate = self.connection.recv_match(
                type="HEARTBEAT",
                blocking=True,
                timeout=max(0.0, deadline - time.monotonic()),
            )
            if candidate is None:
                break
            if getattr(candidate, "type", None) == mavutil.mavlink.MAV_TYPE_GCS:
                continue
            if (
                getattr(candidate, "autopilot", None)
                == mavutil.mavlink.MAV_AUTOPILOT_INVALID
            ):
                continue
            msg = candidate
            get_system = getattr(msg, "get_srcSystem", None)
            get_component = getattr(msg, "get_srcComponent", None)
            if get_system is not None:
                self.connection.target_system = get_system()
            if get_component is not None:
                self.connection.target_component = get_component()
            break

        if msg is not None:
            print(
                f"[COMM] Heartbeat received! Target System: {self.connection.target_system}, Component: {self.connection.target_component}"
            )
            self.telemetry["connected"] = True
            self.telemetry["last_heartbeat_time"] = time.monotonic()
            self.telemetry["armed"] = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )
            self.telemetry["mode"] = mavutil.mode_string_v10(msg)
            return True
        else:
            print("[COMM] ERROR: Timeout waiting for heartbeat.")
            return False

    def request_data_streams(self, rate_hz=10):
        """Requests the Pixhawk to stream specific data packets at a given frequency."""
        print(f"[COMM] Requesting telemetry streams at {rate_hz} Hz...")

        self.request_pose_stream(rate_hz=rate_hz)
        for msg_name in (
            "MAVLINK_MSG_ID_SYS_STATUS",
            "MAVLINK_MSG_ID_EKF_STATUS_REPORT",
            "MAVLINK_MSG_ID_RC_CHANNELS",
            "MAVLINK_MSG_ID_EXTENDED_SYS_STATE",
        ):
            self._request_message_interval_by_name(msg_name, rate_hz)

    def request_pose_stream(self, rate_hz=20):
        """Requests local position and attitude streams for autonomy pose control."""
        print(f"[COMM] Requesting pose streams at {rate_hz} Hz...")
        for msg_id in (
            mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
        ):
            self._request_message_interval(msg_id, rate_hz)

    def _request_message_interval(self, msg_id, rate_hz):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,  # Confirmation
            msg_id,  # Param 1: Message ID
            int(1e6 / rate_hz),  # Param 2: Interval in microseconds
            0,
            0,
            0,
            0,
            0,  # Params 3-7 (unused)
        )

    def _request_message_interval_by_name(self, msg_name, rate_hz):
        msg_id = getattr(mavutil.mavlink, msg_name, None)
        if msg_id is None:
            print(f"[COMM] WARNING: pymavlink lacks {msg_name}; stream not requested")
            return
        self._request_message_interval(msg_id, rate_hz)

    def send_gcs_heartbeat(self):
        """
        Sends a heartbeat FROM the Raspberry Pi TO the Pixhawk.
        Must be called at least 1Hz to prevent ArduPilot from triggering GCS Failsafe (RTL/LAND).
        """
        self.connection.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0
        )

    def get_telemetry_tick(self, max_messages=100):
        """
        Non-blocking read of the MAVLink buffer. Parses ALL incoming messages,
        updates telemetry, and prints Pixhawk text messages (STATUSTEXT).
        Returns the updated dictionary.
        """
        processed = 0
        while processed < max_messages:
            msg = self.connection.recv_match(blocking=False)
            if msg is None:
                break  # Buffer is empty, exit loop
            processed += 1

            msg_type = msg.get_type()
            if not self._message_from_target(msg, msg_type):
                continue

            if msg_type == "HEARTBEAT":
                self.telemetry["last_heartbeat_time"] = time.monotonic()
                self.telemetry["connected"] = True
                self.telemetry["armed"] = bool(
                    msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                )
                self.telemetry["mode"] = mavutil.mode_string_v10(msg)

            elif msg_type == "LOCAL_POSITION_NED":
                self.telemetry["pos_x_m"] = msg.x
                self.telemetry["pos_y_m"] = msg.y
                self.telemetry["pos_z_m"] = msg.z
                self.telemetry["vel_x_m_s"] = getattr(msg, "vx", 0.0)
                self.telemetry["vel_y_m_s"] = getattr(msg, "vy", 0.0)
                self.telemetry["vel_z_m_s"] = getattr(msg, "vz", 0.0)
                self.telemetry["last_local_position_time"] = time.monotonic()

            elif msg_type == "ATTITUDE":
                self.telemetry["roll_rad"] = msg.roll
                self.telemetry["pitch_rad"] = msg.pitch
                self.telemetry["yaw_rad"] = msg.yaw
                self.telemetry["last_attitude_time"] = time.monotonic()

            elif msg_type == "SYS_STATUS":
                self.telemetry["battery_voltage_v"] = msg.voltage_battery / 1000.0
                self.telemetry["battery_remaining_pct"] = msg.battery_remaining

            elif msg_type == "EKF_STATUS_REPORT":
                self.telemetry["ekf_flags"] = msg.flags
                self.telemetry["ekf_healthy"] = self._ekf_flags_healthy(msg.flags)
                self.telemetry["last_ekf_time"] = time.monotonic()

            elif msg_type in ("RC_CHANNELS", "RC_CHANNELS_RAW"):
                self.telemetry["rc_rssi"] = msg.rssi
                self.telemetry["rc_channel_count"] = self._rc_channel_count(msg)
                self.telemetry["rc_link_live"] = (
                    msg.rssi != 255 or self.telemetry["rc_channel_count"] > 0
                )
                self.telemetry["last_rc_channels_time"] = time.monotonic()

            elif msg_type == "EXTENDED_SYS_STATE":
                landed = getattr(msg, "landed_state", 0)
                landed_names = {
                    getattr(mavutil.mavlink, "MAV_LANDED_STATE_UNDEFINED", 0): "undefined",
                    getattr(mavutil.mavlink, "MAV_LANDED_STATE_ON_GROUND", 1): "on-ground",
                    getattr(mavutil.mavlink, "MAV_LANDED_STATE_IN_AIR", 2): "in-air",
                    getattr(mavutil.mavlink, "MAV_LANDED_STATE_TAKEOFF", 3): "takeoff",
                    getattr(mavutil.mavlink, "MAV_LANDED_STATE_LANDING", 4): "landing",
                }
                self.telemetry["landed_state"] = landed_names.get(
                    landed, "undefined"
                )
                self.telemetry["last_landed_state_time"] = time.monotonic()

            elif msg_type == "COMMAND_ACK":
                self._protocol_events.append(
                    {
                        "type": "command_ack",
                        "command": msg.command,
                        "result": msg.result,
                        "accepted": msg.result
                        == mavutil.mavlink.MAV_RESULT_ACCEPTED,
                        "received_at": time.monotonic(),
                    }
                )

            elif msg_type == "STATUSTEXT":
                # Intercept text messages from Pixhawk (Pre-Arm errors, EKF warnings, etc.)
                text = (
                    msg.text.decode("utf-8")
                    if isinstance(msg.text, bytes)
                    else msg.text
                )
                print(f"\n⚠️ [PIXHAWK MSG]: {text}")

        return self.telemetry

    def _message_from_target(self, msg, msg_type):
        """Reject telemetry/results from other systems or flight components."""
        get_system = getattr(msg, "get_srcSystem", None)
        source_system = get_system() if get_system is not None else None
        target_system = getattr(self.connection, "target_system", None)
        if source_system not in (None, 0) and target_system not in (None, 0):
            if source_system != target_system:
                return False
        if msg_type in {"HEARTBEAT", "COMMAND_ACK", "EXTENDED_SYS_STATE"}:
            get_component = getattr(msg, "get_srcComponent", None)
            source_component = get_component() if get_component is not None else None
            target_component = getattr(self.connection, "target_component", None)
            if source_component not in (None, 0) and target_component not in (None, 0):
                return source_component == target_component
        return True

    def drain_protocol_events(self):
        """Return ACK-related events parsed by the centralized receive loop."""
        events = tuple(self._protocol_events)
        self._protocol_events.clear()
        return events

    def _ekf_flags_healthy(self, flags):
        """Checks EKF status bits needed for guided local-position tests."""
        attitude = 1 << 0
        velocity_horiz = 1 << 1
        velocity_vert = 1 << 2
        pos_horiz_rel = 1 << 3
        pos_horiz_abs = 1 << 4
        pos_vert_abs = 1 << 5
        pos_vert_agl = 1 << 6
        accel_error = 1 << 11

        has_horizontal_position = bool(flags & (pos_horiz_abs | pos_horiz_rel))
        has_vertical_position = bool(flags & (pos_vert_abs | pos_vert_agl))
        required = attitude | velocity_horiz | velocity_vert
        unhealthy = accel_error
        required_ok = (flags & required) == required
        unhealthy_present = bool(flags & unhealthy)
        return (
            required_ok
            and has_horizontal_position
            and has_vertical_position
            and not unhealthy_present
        )

    def _rc_channel_count(self, msg):
        count = 0
        for index in range(1, 19):
            value = getattr(msg, f"chan{index}_raw", 0)
            if 900 <= value <= 2200:
                count += 1
        return count

    def get_pose(self, max_age_s=0.5, now_s=None):
        """Returns the latest local-NED pose and whether both pose streams are fresh."""
        now_s = now_s if now_s is not None else time.monotonic()
        local_age_s = now_s - self.telemetry.get("last_local_position_time", 0.0)
        attitude_age_s = now_s - self.telemetry.get("last_attitude_time", 0.0)
        fresh = local_age_s <= max_age_s and attitude_age_s <= max_age_s
        return {
            "pos_x_m": self.telemetry["pos_x_m"],
            "pos_y_m": self.telemetry["pos_y_m"],
            "pos_z_m": self.telemetry["pos_z_m"],
            "roll_rad": self.telemetry["roll_rad"],
            "pitch_rad": self.telemetry["pitch_rad"],
            "yaw_rad": self.telemetry["yaw_rad"],
            "local_age_s": local_age_s,
            "attitude_age_s": attitude_age_s,
            "fresh": fresh,
        }

    # -------------------------------------------------------------------------
    # COMMAND METHODS (ACTIONS)
    # -------------------------------------------------------------------------

    def set_mode(self, mode_name="GUIDED"):
        """Changes the flight mode (e.g., 'GUIDED', 'RTL', 'LAND')."""
        if mode_name not in self.connection.mode_mapping():
            print(f"[COMM] ERROR: Unknown flight mode '{mode_name}'")
            return False

        mode_id = self.connection.mode_mapping()[mode_name]

        self.connection.mav.set_mode_send(
            self.connection.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )
        print(f"[COMM] Command sent: Change mode to {mode_name}")
        return True

    def arm(self, state=True):
        """Send one nonblocking arm/disarm command."""
        arm_val = 1 if state else 0
        action = "ARM" if state else "DISARM"
        print(f"[COMM] Sending {action} command...")

        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,  # Confirmation
            arm_val,  # Param 1: 1 to arm, 0 to disarm
            0,
            0,
            0,
            0,
            0,
            0,  # Params 2-7 (unused)
        )

        return True

    def liftoff(self, altitude_m):
        """
        Commands the drone to lift off to a specified relative altitude.
        Drone MUST be armed and in GUIDED mode before sending this.
        """
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,  # Confirmation
            0,
            0,
            0,
            0,
            0,
            0,
            altitude_m,  # Param 7: Altitude in meters
        )
        print(f"[COMM] Command sent: LIFTOFF to {altitude_m}m")

    def takeoff(self, altitude_m):
        """Compatibility wrapper for liftoff()."""
        self.liftoff(altitude_m)

    def _send_land(self, precision_mode):
        self.connection.mav.command_long_send(
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,  # confirmation
            0.0,  # param1: abort altitude
            precision_mode,  # param2: opportunistic precision landing
            0.0,  # param3: unused
            float("nan"),  # param4: retain current yaw behavior
            0.0,  # param5: land at current latitude
            0.0,  # param6: land at current longitude
            0.0,  # param7: ground level
        )

    def precision_land(self):
        """Send one opportunistic precision LAND command (param2=1)."""
        precision_mode = getattr(
            mavutil.mavlink, "PRECISION_LAND_MODE_OPPORTUNISTIC", 1
        )
        self._send_land(precision_mode)
        print("[COMM] Command sent: OPPORTUNISTIC PRECISION LAND")

    def land_here(self):
        """Send one non-precision LAND command (param2=0)."""
        precision_mode = getattr(
            mavutil.mavlink, "PRECISION_LAND_MODE_DISABLED", 0
        )
        self._send_land(precision_mode)
        print("[COMM] Command sent: LAND HERE")

    def land(self):
        """Compatibility alias for opportunistic precision landing."""
        self.precision_land()

    def send_position_target_local_ned(self, dx_m, dy_m, dz_m):
        """
        Moves the drone relative to its CURRENT position and heading.

        Parameters:
        dx_m: Move Forward (positive) or Backward (negative) in meters.
        dy_m: Move Right (positive) or Left (negative) in meters.
        dz_m: Move DOWN (positive) or UP (negative) in meters.
        """
        type_mask = (
            (1 << 3)   # Ignore velocity X
            | (1 << 4) # Ignore velocity Y
            | (1 << 5) # Ignore velocity Z
            | (1 << 6) # Ignore acceleration X
            | (1 << 7) # Ignore acceleration Y
            | (1 << 8) # Ignore acceleration Z
            | (1 << 10) # Ignore yaw angle
            # Bit 11 is NOT set:
            # yaw_rate is active and equals 0 rad/s
        )

        self.connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_OFFSET_NED,  # Relative to current drone body
            type_mask,
            dx_m,
            dy_m,
            dz_m,  # Position
            0.0,
            0.0,
            0.0,  # Velocity (Ignored)
            0.0,
            0.0,
            0.0,  # Acceleration (Ignored)
            0.0,  # Yaw ignored
            0.0,  # Active yaw rate: hold 0 rad/s
        )
        print(f"[COMM] Command sent: MOVE Local [dx:{dx_m}, dy:{dy_m}, dz:{dz_m}]")

    def send_local_ned_position_target(self, x_m, y_m, z_m, log=True):
        """
        Sends one absolute local-NED position target.

        Parameters:
        x_m: North position in meters.
        y_m: East position in meters.
        z_m: Down position in meters. Negative values are above the local origin.
        """
        type_mask = (
            (1 << 3)  # Ignore velocity X
            | (1 << 4)  # Ignore velocity Y
            | (1 << 5)  # Ignore velocity Z
            | (1 << 6)  # Ignore acceleration X
            | (1 << 7)  # Ignore acceleration Y
            | (1 << 8)  # Ignore acceleration Z
            | (1 << 10)  # Ignore yaw
            # | (1 << 11)  # Ignore yaw rate
        )

        self.connection.mav.set_position_target_local_ned_send(
            int(time.monotonic() * 1000) & 0xFFFFFFFF,
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            x_m,
            y_m,
            z_m,  # Position
            0.0,
            0.0,
            0.0,  # Velocity (Ignored)
            0.0,
            0.0,
            0.0,  # Acceleration (Ignored)
            0.0,  # Yaw ignored
            0.0,  # Active yaw rate: hold 0 rad/s
        )
        if log:
            print(
                f"[COMM] Command sent: LOCAL_NED position [x:{x_m}, y:{y_m}, z:{z_m}]"
            )

    def hold_local_ned_position(self, x_m, y_m, z_m, rate_hz=10.0, duration_s=None):
        """
        Continuously resends the same local-NED position target.

        Use this for GUIDED position hold behavior when commanding from the
        companion computer. If duration_s is None, this runs until interrupted.
        """
        if rate_hz <= 0:
            raise ValueError("rate_hz must be greater than zero")

        interval_s = 1.0 / rate_hz
        start_time = time.monotonic()
        sent_count = 0

        while duration_s is None or (time.monotonic() - start_time) < duration_s:
            self.send_local_ned_position_target(x_m, y_m, z_m, log=False)
            sent_count += 1
            time.sleep(interval_s)

        return sent_count

    def send_velocity_target_body_ned(self, vx_m_s, vy_m_s, vz_m_s):
        """
        Commands the drone to move at a specific velocity relative to its own body.

        Parameters:
        vx_m_s: Forward (+) / Backward (-) speed in m/s.
        vy_m_s: Right (+) / Left (-) speed in m/s.
        vz_m_s: Down (+) / Up (-) speed in m/s.
        """
        type_mask = int(0b000011011000111)

        self.connection.mav.set_position_target_local_ned_send(
            0,  # time_boot_ms (not used)
            self.connection.target_system,
            self.connection.target_component,
            mavutil.mavlink.MAV_FRAME_BODY_NED,  # Velocity relative to drone's heading
            type_mask,
            0.0,
            0.0,
            0.0,  # Position (Ignored)
            vx_m_s,
            vy_m_s,
            vz_m_s,  # Velocity in m/s (USED)
            0.0,
            0.0,
            0.0,  # Acceleration (Ignored)
            0.0,
            0.0,  # Yaw, Yaw rate (Ignored)
        )
        print(
            f"[COMM] Command sent: VELOCITY [vx:{vx_m_s}, vy:{vy_m_s}, vz:{vz_m_s}] m/s"
        )

    def send_landing_target(
        self,
        target: tuple[float, float, float],
        initiate_landing: bool = False,
    ):
        """
        Sends the current precision-landing target position to ArduPilot.

        Parameters:
            target:
                Current landing-target position in MAV_FRAME_BODY_FRD, in meters:

                target[0] = x: forward from the drone
                target[1] = y: right from the drone
                target[2] = z: down from the drone

                Example:
                    (0.20, -0.10, 2.50)

                means the marker is:
                    20 cm forward,
                    10 cm left,
                    2.5 m below the drone.

            initiate_landing:
                When True, also sends MAV_CMD_NAV_LAND after publishing the
                target measurement. Set this to True only once, when precision
                landing should begin.

        This function must continue to be called with updated target coordinates
        while the target remains visible.
        """

        if len(target) != 3:
            raise ValueError(
                "target must contain exactly three BODY_FRD coordinates: "
                "(x_forward_m, y_right_m, z_down_m)"
            )

        x_m, y_m, z_m = map(float, target)

        if not all(math.isfinite(value) for value in (x_m, y_m, z_m)):
            raise ValueError("Landing-target coordinates must be finite numbers")

        if z_m <= 0.0:
            raise ValueError(
                "Landing target must be below the drone: BODY_FRD z must be positive"
            )

        distance_m = math.sqrt(x_m**2 + y_m**2 + z_m**2)

        self.connection.mav.landing_target_send(
            int(time.monotonic() * 1_000_000),  # time_usec
            0,  # target_num
            mavutil.mavlink.MAV_FRAME_BODY_FRD,
            0.0,  # angle_x, unused in position mode
            0.0,  # angle_y, unused in position mode
            distance_m,
            0.0,  # size_x, unused
            0.0,  # size_y, unused
            x_m,  # forward
            y_m,  # right
            z_m,  # down
            (1.0, 0.0, 0.0, 0.0),  # target orientation, unused
            mavutil.mavlink.LANDING_TARGET_TYPE_VISION_FIDUCIAL,
            1,  # position_valid
        )
        print(
            f"[COMM] Command sent: LANDING TARGET [x:{x_m:.3f}, y:{y_m:.3f}, z:{z_m:.3f}]"
        )

        if initiate_landing:
            self.land()
            print(
                "[COMM] Precision landing initiated on target "
                f"[x:{x_m:.3f}, y:{y_m:.3f}, z:{z_m:.3f}]"
            )
