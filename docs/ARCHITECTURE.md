# Architecture — Indomitus Drone

> This document is the single source of truth for the system architecture of the **Indomitus Drone** autonomous quadcopter, built for the **European Rover Challenge (ERC) 2026 — Droning Sub-Task**. It is designed to provide complete context for any developer or AI assistant working on this codebase.

---

## Table of Contents

1. [Mission Context](#1-mission-context)
2. [Hardware Stack](#2-hardware-stack)
3. [System Architecture Overview](#3-system-architecture-overview)
4. [Layer 1 — The Eyes (Simulink C++ ROS 2 Node)](#4-layer-1--the-eyes-simulink-c-ros-2-node)
5. [Layer 2 — The Brain (Python State Machine)](#5-layer-2--the-brain-python-state-machine)
6. [Layer 3 — The Muscle (Pixhawk Flight Controller)](#6-layer-3--the-muscle-pixhawk-flight-controller)
7. [Communication Protocols](#7-communication-protocols)
8. [State Machine Design](#8-state-machine-design)
9. [Coordinate Systems & Frames of Reference](#9-coordinate-systems--frames-of-reference)
10. [Repository File Map](#10-repository-file-map)
11. [External Dependencies & Filesystem Layout](#11-external-dependencies--filesystem-layout)
12. [Startup & Deployment Sequence](#12-startup--deployment-sequence)
13. [Configuration Reference](#13-configuration-reference)
14. [Implementation Status & TODO](#14-implementation-status--todo)

---

## 1. Mission Context

### 1.1 Competition Rules

The ERC Droning Sub-Task takes place inside a **10 × 10 × 4 m enclosed cage**. The drone must operate **fully autonomously** — any manual intervention during flight is penalized.

**Core mission (repeated 3 times):**
1. Lift off from a 1 × 1 m takeoff pad marked with **ArUco ID 101** (15 × 15 cm, ArUco Original dictionary).
2. Search for a landing disc (radius 0.5 m) randomly placed within 3 m of the takeoff pad, marked with **ArUco ID 102** (15 × 15 cm).
3. Precision land on the disc. Landing is successful if any part of the drone touches the disc.
4. After landing, the drone is **manually repositioned** back to the takeoff pad for the next attempt.

**Additional scoring — Probe Detection:**
- 3 probes are scattered within the 3 m radius area.
- The system must detect them and report their positions using a 1 × 1 m alphanumeric grid (e.g., sector "A2" covers coordinates 0–0.5 m to 0.5–1.0 m).
- 5 points per probe per mission (max 45 points across 3 missions).
- Probe positions remain fixed across all 3 missions, but detections are scored per-mission.

**Additional scoring — Custom Landing Platform:**
- Teams can prepare their own landing target (within 0.5 m radius disc) attachable to their rover.
- 15 points for preparing it + 15 points per successful landing on it (max 45 points).
- The custom platform does NOT have to use an ArUco marker.

### 1.2 Time Constraints

| Phase | Duration |
|-------|----------|
| Preparation (outside cage) | 15 minutes |
| Mission execution | 30 minutes |
| Total available for 3 takeoff-search-land cycles | 30 minutes |

### 1.3 Preflight Safety Requirements

The jury will verify these before flight:
- Real-time telemetry + video feed to the operator.
- Instant switch to manual/remote control on demand.
- Automatic mid-air stability holding (LOITER mode).
- Automatic landing on demand.
- Failsafe for: RC loss, GCS loss, battery low, positioning glitch.
- Polygon inclusion geofence configured for the cage dimensions.

---

## 2. Hardware Stack

### 2.1 Airframe & Propulsion

| Component | Specification |
|-----------|---------------|
| Frame | F450 quadcopter with landing gears |
| Motors | DJI 2212 920KV brushless (×4) |
| Propellers | 1045R (10 × 4.5) |
| Battery | 4S LiPo |

### 2.2 Computing

| Component | Role | Connection |
|-----------|------|------------|
| **Pixhawk 6C** | Low-level flight controller (ArduCopter firmware) | — |
| **Raspberry Pi 5** (16 GB) | Companion computer — vision, state machine, autonomy | UART to Pixhawk |
| **5V 5A UBEC** | Powers the Pi from the LiPo | — |

### 2.3 Sensors

| Sensor | Direction | Connection | Purpose |
|--------|-----------|------------|---------|
| Arducam IMX708 12MP HDR 120° | Downward | Pi CSI (ribbon cable) | ArUco/probe detection |
| Foxeer Cat 4 FPV | Forward | VTX only (NOT connected to Pi) | Pilot's view |
| Microair MTF-01 | Downward | Pixhawk serial | Optical flow for GPS-denied flight |
| Holybro M10 | Upward | Pixhawk I2C/serial | GPS + compass |
| Pixhawk IMU | Internal | Onboard | Accelerometer + gyroscope |

### 2.4 Communication Links

| Link | Hardware | Frequency | Protocol | Purpose |
|------|----------|-----------|----------|---------|
| RC Control | RadioMaster RP1 V2 | 2.4 GHz (ELRS) | ExpressLRS | Pilot manual override |
| Telemetry | 3DR Radio V5 | 433 MHz | MAVLink | Ground station data |
| Video | Race Ranger VTX | 5.8 GHz | Analog PAL/NTSC | FPV to pilot goggles/monitor |
| Pi ↔ Pixhawk | Hardwired UART | N/A | MAVLink 2.0 | Companion computer commands |

### 2.5 Video Switching

A **3-channel video switcher** toggles the VTX feed between:
1. Forward Foxeer Cat 4 camera (pilot view).
2. Raspberry Pi composite output (can show Pi desktop or downward camera feed via script).

The VTX is completely isolated from the Pixhawk — there is **no OSD overlay**.

---

## 3. System Architecture Overview

The system operates as three sequential layers, each running as an independent process:

```
┌─────────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 5                                   │
│                                                                         │
│  ┌───────────────────────┐     ROS 2 DDS      ┌──────────────────────┐ │
│  │  LAYER 1: THE EYES    │    (local topic)    │  LAYER 2: THE BRAIN  │ │
│  │                       │                     │                      │ │
│  │  Simulink C++ Node    │ ──────────────────► │  Python rclpy Node   │ │
│  │  (ArUco + Probes)     │ /erc/vision_targets │  (State Machine)     │ │
│  │                       │                     │                      │ │
│  │  Reads: Arducam       │                     │  Reads: ROS 2 topic  │ │
│  │  Publishes: coords    │                     │  Writes: MAVLink     │ │
│  └───────────────────────┘                     └──────────┬───────────┘ │
│                                                           │             │
│                                                    UART Serial          │
│                                                   /dev/ttyAMA0          │
│                                                    921600 baud          │
└────────────────────────────────────────────────────────────┼─────────────┘
                                                            │
                                                            ▼
                                               ┌────────────────────────┐
                                               │  LAYER 3: THE MUSCLE   │
                                               │                        │
                                               │  Pixhawk 6C            │
                                               │  ArduCopter + EKF3     │
                                               │                        │
                                               │  Fuses: Optical flow   │
                                               │         GPS + IMU      │
                                               │  Outputs: Motor PWM    │
                                               └────────────────────────┘
```

### Why This Architecture?

The ERC rules require that image processing and high-level decision-making use **MATLAB/Simulink**. Rather than rewriting the entire flight stack in MATLAB, we compartmentalize:

- **Simulink** handles what it's best at: optimized C++ computer vision, compiled and deployed as a standalone ROS 2 node.
- **Python** handles what it's best at: flexible state machine logic, rapid iteration, and `pymavlink` integration that's already proven in flight tests.
- **ROS 2** is the glue: a single topic (`/erc/vision_targets`) bridges the two worlds with zero coupling.

---

## 4. Layer 1 — The Eyes (Simulink C++ ROS 2 Node)

### What It Does

- Ingests raw camera frames from the downward-facing Arducam IMX708 via **V4L2** (`/dev/video0`).
- Runs ArUco marker detection for IDs 101 (takeoff pad) and 102 (landing target).
- Runs probe detection (color-based or shape-based algorithm).
- Calculates the **relative X/Y offset** of detected targets from the camera center, in meters.
- Publishes detection results to the ROS 2 topic `/erc/vision_targets`.

### Where It Lives

This node does **NOT** live in this repository. It is:
1. Designed visually in **Simulink** on the ground station laptop using the Computer Vision Toolbox and Raspberry Pi Blockset.
2. Auto-compiled to C++ by **Simulink Coder**.
3. Deployed over SSH to `~/ros2_ws/src/` on the Raspberry Pi.
4. Built on the Pi using `colcon build`.
5. Launched with `ros2 run <package_name> <node_name>`.

### ROS 2 Topic Output

**Topic:** `/erc/vision_targets`
**Message Type:** `geometry_msgs/msg/Point`

The `z` field encodes the detection type. The routing convention used by `VisionBridge`:

| `z` value | Detection Type | `x` meaning | `y` meaning |
|-----------|---------------|-------------|-------------|
| `101.0` | Takeoff pad marker | Camera-frame horizontal offset (meters, right = +) | Camera-frame vertical offset (meters, forward = +) |
| `102.0` | Landing target marker | Camera-frame horizontal offset (meters, right = +) | Camera-frame vertical offset (meters, forward = +) |
| `< 0` (e.g. `-1.0`) | Probe detected | World X position relative to takeoff pad (meters, North) | World Y position relative to takeoff pad (meters, East) |
| `0.0` | No detection / heartbeat | Ignored | Ignored |

---

## 5. Layer 2 — The Brain (Python State Machine)

### What It Does

This is the core of this repository. It:
1. Subscribes to `/erc/vision_targets` via `rclpy` to receive vision data from the Simulink node.
2. Maintains a finite state machine that transitions through mission phases.
3. Calculates flight setpoints (position or velocity targets) based on the current state and vision data.
4. Sends MAVLink commands to the Pixhawk via UART using `pymavlink`.

### Process Architecture

The Python side runs as **two OS-level processes** communicating via `multiprocessing.Queue`:

```
┌──────────────────────────────────────────┐
│           MAIN PROCESS                    │
│                                           │
│  ┌─────────────────────────────────────┐ │
│  │  rclpy Subscriber                    │ │
│  │  (vision_subscriber.py)              │ │
│  │                                      │ │
│  │  Receives: /erc/vision_targets       │ │
│  │  Updates: shared state variables     │ │
│  └──────────┬──────────────────────────┘ │
│             │                             │
│             ▼                             │
│  ┌─────────────────────────────────────┐ │
│  │  State Machine                       │ │
│  │  (state_machine.py)                  │ │
│  │                                      │ │
│  │  Evaluates: current state + vision   │ │
│  │  Produces: flight commands           │ │
│  └──────────┬──────────────────────────┘ │
│             │  command_queue.put(cmd)      │
└─────────────┼────────────────────────────┘
              │
              ▼  multiprocessing.Queue
┌──────────────────────────────────────────┐
│         COMM PROCESS (daemon)             │
│         (mavlink_node.py)                 │
│                                           │
│  - Reads command_queue                    │
│  - Dispatches to mavlink_client.py        │
│  - Reads telemetry from Pixhawk           │
│  - Pushes telemetry to telemetry_queue    │
│  - Sends GCS heartbeat at 1 Hz            │
└──────────────────────────────────────────┘
```

---

## 6. Layer 3 — The Muscle (Pixhawk Flight Controller)

### What It Does

The Pixhawk 6C runs **ArduCopter** firmware and handles everything below the application level:
- **EKF3 sensor fusion**: merges optical flow (MTF-01), GPS (Holybro M10), IMU, and compass data into a unified position/velocity/attitude estimate.
- **PID control loops**: converts position/velocity setpoints into motor PWM signals.
- **Failsafe management**: RC loss → RTL, GCS loss → LAND, battery low → LAND, EKF divergence → LAND.
- **Flight modes**: GUIDED (accepts external setpoints), LOITER (hold position), LAND, RTL.

### What It Receives from the Pi

MAVLink 2.0 messages over UART (`/dev/ttyAMA0` at 921600 baud):

| MAVLink Message | Purpose |
|-----------------|---------|
| `HEARTBEAT` (GCS type) | Keeps GCS failsafe from triggering (sent at 1 Hz) |
| `SET_MODE` | Changes flight mode (GUIDED, LOITER, LAND, RTL) |
| `MAV_CMD_COMPONENT_ARM_DISARM` | Arms/disarms motors |
| `MAV_CMD_NAV_TAKEOFF` | Commands liftoff to specified altitude |
| `MAV_CMD_NAV_LAND` | Commands landing (with optional precision landing) |
| `SET_POSITION_TARGET_LOCAL_NED` | Position or velocity setpoints in local NED frame |
| `LANDING_TARGET` | Precision landing target position in BODY_FRD frame |

### What It Sends to the Pi

| MAVLink Message | Data |
|-----------------|------|
| `HEARTBEAT` | Armed state, flight mode |
| `LOCAL_POSITION_NED` | X, Y, Z position in local NED (meters) |
| `ATTITUDE` | Roll, pitch, yaw (radians) |
| `SYS_STATUS` | Battery voltage, remaining percentage |
| `EKF_STATUS_REPORT` | EKF health flags |
| `RC_CHANNELS` | RC link status, RSSI, channel values |
| `STATUSTEXT` | Human-readable messages (pre-arm checks, errors) |
| `COMMAND_ACK` | Acknowledgment of received commands |

---

## 7. Communication Protocols

### 7.1 ROS 2 DDS (Layer 1 → Layer 2)

- **Middleware:** FastDDS (default for ROS 2 Jazzy)
- **Transport:** Shared memory (both nodes run on the same Pi)
- **Domain ID:** Configured in `config/mission_params.yaml` (default: 42)
- **QoS:** Default reliability (RELIABLE) with queue depth 10

| Topic | Message Type | Publisher | Subscriber | Rate |
|-------|-------------|-----------|------------|------|
| `/erc/vision_targets` | `geometry_msgs/msg/Point` | Simulink C++ node | `vision_subscriber.py` | ~10–30 Hz |

### 7.2 MAVLink 2.0 (Layer 2 → Layer 3)

- **Physical:** UART serial, `/dev/ttyAMA0`
- **Baud rate:** 921600
- **Protocol:** MAVLink 2.0 (`MAVLINK20=1`)
- **Source system/component:** 255/0 (identifies Pi as a GCS to ArduCopter)
- **Target system/component:** Auto-detected via initial heartbeat handshake

### 7.3 Inter-Process (within Layer 2)

- **Mechanism:** `multiprocessing.Queue` (Python standard library)
- **Queues:**
  - `telemetry_queue`: Comm process → Main process. Carries telemetry dictionaries at 10 Hz. Old entries are flushed to ensure freshness.
  - `command_queue`: Main process → Comm process. Carries timestamped command dictionaries. Commands older than 500 ms are dropped.

---

## 8. State Machine Design

The autonomous mission follows this state progression:

```
                    ┌─────────┐
                    │  IDLE   │
                    └────┬────┘
                         │ start_mission()
                         ▼
                    ┌─────────┐
                    │ TAKEOFF │
                    └────┬────┘
                         │ altitude reached
                         ▼
                    ┌─────────┐◄──────────────────┐
                    │ SEARCH  │                    │
                    └────┬────┘                    │
                         │ marker 102 detected     │
                         ▼                         │
                    ┌─────────┐                    │
                    │  ALIGN  │                    │
                    └────┬────┘                    │
                         │ centered over target    │
                         ▼                         │
                    ┌─────────┐                    │
                    │  LAND   │                    │
                    └────┬────┘                    │
                         │ touchdown confirmed     │
                         ▼                         │
                    ┌──────────┐    attempts < 3   │
                    │ COMPLETE ├────────────────────┘
                    └────┬─────┘
                         │ attempts == 3
                         ▼
                    ┌──────────┐
                    │ MISSION  │
                    │  DONE    │
                    └──────────┘
```

### State Descriptions

| State | Entry Condition | Behavior | Exit Condition |
|-------|----------------|----------|----------------|
| **IDLE** | System startup | Wait for operator command. Arm checks. | `start_mission()` called |
| **TAKEOFF** | Mission started | Set GUIDED mode, arm, send takeoff command. Monitor altitude. | Target altitude reached (within 90%) |
| **SEARCH** | At search altitude | Execute sweep pattern within 3 m radius. Continuously check `/erc/vision_targets` for marker ID 102. Also detect and log probes. | Marker 102 detected |
| **ALIGN** | Marker 102 visible | Reduce altitude. Send velocity/position corrections to center the drone over the marker. | Target centered within landing threshold |
| **LAND** | Aligned over target | Send precision landing commands (`LANDING_TARGET` messages). Continuously update target position. | Drone touches down (altitude ≈ 0, disarmed or in LAND mode) |
| **COMPLETE** | Landed | Log attempt. If attempts < 3, transition to IDLE (wait for manual reposition). | Manual trigger for next attempt, or 3 attempts done |
| **MISSION_DONE** | 3 landings complete | Report probe detection results. Disarm. | — |

### Probe Detection (During SEARCH State)

While searching for marker 102, the system simultaneously:
1. Detects probes in the camera feed (via Simulink node).
2. Maps each probe's estimated (x, y) position to the 1 × 1 m grid sector.
3. Accumulates detections across frames for confidence filtering.
4. Reports final sector IDs (e.g., "A2", "C4") at mission end.

---

## 9. Coordinate Systems & Frames of Reference

### 9.1 Local NED (Pixhawk / ArduCopter)

Used by `LOCAL_POSITION_NED` and `SET_POSITION_TARGET_LOCAL_NED`:
- **Origin:** Where the drone was when EKF initialized (typically the takeoff pad).
- **X:** North (forward from initial heading) — positive forward.
- **Y:** East — positive right.
- **Z:** Down — **negative values mean UP**.
- Example: position `(0, 0, -2.0)` means "above the origin at 2 m altitude."

### 9.2 Body NED / Body FRD (Drone-Relative)

Used by velocity commands and `LANDING_TARGET`:
- **X / Forward:** Positive in the direction the drone is facing.
- **Y / Right:** Positive to the right of the drone's heading.
- **Z / Down:** Positive downward.
- Example: `LANDING_TARGET(x=0.2, y=-0.1, z=2.5)` means "marker is 20 cm forward, 10 cm left, 2.5 m below the drone."

### 9.3 Camera Frame (Simulink Output)

The Simulink vision node outputs offsets relative to the **camera center**:
- **X:** Horizontal pixel offset converted to meters (positive = target is to the right of center).
- **Y:** Vertical pixel offset converted to meters (positive = target is ahead/above center in the image).
- Conversion from pixels to meters uses the camera intrinsic matrix from `config/camera_calibration.npz` and the estimated altitude.

### 9.4 Competition Grid

For probe detection reporting:
- The 6 × 6 m competition area is divided into 1 × 1 m sectors.
- Columns labeled A–F (left to right).
- Rows labeled 1–6 (front to back).
- Grid origin and mapping are defined in `config/mission_params.yaml`.

---

## 10. Repository File Map

Every file in this repository, its purpose, and key implementation details.

### Root Files

#### `main.py`
**Role:** Application entry point. Orchestrates all processes.

**What it does:**
1. Creates `multiprocessing.Queue` instances for telemetry and commands.
2. Spawns the MAVLink communication process (`comm_process_loop` from `mavlink_node.py`).
3. Initializes the `rclpy` vision subscriber.
4. Runs the state machine main loop.
5. Handles `Ctrl+C` graceful shutdown (sends LAND command, terminates processes).

**Key dependency:** Must be run with ROS 2 environment sourced (`source /opt/ros/jazzy/setup.bash`).

#### `requirements.txt`
**Contents:** `pymavlink`, `numpy`, `pyyaml`.

**Important:** `rclpy` is NOT listed — it comes from the system ROS 2 installation at `/opt/ros/jazzy/`, not from pip. The virtual environment must be created with `--system-site-packages` to access it.

---

### `src/comm/` — MAVLink Communication Layer

This module is **copied directly from the proven old repository** (`indomitus-drone-ros2`). It has been flight-tested and should not be modified unless a bug is found.

#### `src/comm/mavlink_client.py`
**Role:** Low-level MAVLink serial driver. Encapsulates all direct hardware communication with the Pixhawk.

**Class: `PixhawkClient`**
- **Constructor:** Opens serial connection on `/dev/ttyAMA0` at 921600 baud. Sets `source_system=255` (GCS identity) to satisfy ArduCopter's failsafe logic.
- **`wait_for_heartbeat()`:** Blocks until Pixhawk responds. Establishes `target_system` and `target_component` IDs used for all subsequent commands.
- **`request_data_streams()`:** Configures Pixhawk to stream telemetry at specified rates via `MAV_CMD_SET_MESSAGE_INTERVAL`.
- **`send_gcs_heartbeat()`:** Sends a heartbeat FROM the Pi TO the Pixhawk. Must be called at ≥1 Hz or ArduCopter triggers GCS Failsafe (RTL/LAND).
- **`get_telemetry_tick()`:** Non-blocking read of the UART buffer. Parses ALL incoming MAVLink messages, updates an internal telemetry dictionary, and prints `STATUSTEXT` messages from the Pixhawk.
- **`get_pose()`:** Returns the latest local-NED position + attitude with freshness check.

**Command methods:**
- `set_mode(mode_name)` — Changes flight mode (GUIDED, LOITER, LAND, RTL).
- `arm(state, timeout)` — Arms/disarms with ACK confirmation loop.
- `liftoff(altitude_m)` / `takeoff(altitude_m)` — Commands takeoff.
- `land()` — Commands `MAV_CMD_NAV_LAND` with opportunistic precision landing mode.
- `send_position_target_local_ned(dx, dy, dz)` — Relative body-offset position movement.
- `send_local_ned_position_target(x, y, z)` — Absolute local NED position target.
- `hold_local_ned_position(x, y, z, rate_hz, duration_s)` — Continuous position hold loop.
- `send_velocity_target_body_ned(vx, vy, vz)` — Body-frame velocity command.
- `land_on_target(target, initiate_landing)` — Sends `LANDING_TARGET` message for precision landing in BODY_FRD frame. Must be called continuously while the target is visible.

**Internal helpers:**
- `_ekf_flags_healthy(flags)` — Bitfield check for EKF3 health (attitude, velocity, position, no accel error).
- `_rc_channel_count(msg)` — Counts active RC channels (900–2200 μs range).

#### `src/comm/mavlink_node.py`
**Role:** Process-level wrapper that runs `PixhawkClient` in an isolated `multiprocessing.Process`.

**Function: `comm_process_loop(telemetry_queue, command_queue, connection_string, baudrate)`**
- Runs an infinite event loop with three responsibilities:
  1. **GCS Heartbeat** — sends at 1 Hz to prevent failsafe.
  2. **Telemetry reading** — calls `get_telemetry_tick()`, publishes the freshest telemetry dict to `telemetry_queue` at 10 Hz. Flushes stale entries before each publish.
  3. **Command dispatch** — reads `command_queue`, validates timestamp (drops commands older than 500 ms), dispatches to the appropriate `PixhawkClient` method.
- CPU relief: 5 ms sleep per loop iteration.

**Function: `dispatch_command(client, cmd)`**
- Command router. Maps `cmd["action"]` strings to `PixhawkClient` methods:
  - `"arm"` → `client.arm()`
  - `"set_mode"` → `client.set_mode()`
  - `"takeoff"` → `client.takeoff()`
  - `"land"` → `client.land()`
  - `"move_local_pos"` → `client.send_position_target_local_ned()`
  - `"set_local_position"` → `client.send_local_ned_position_target()`
  - `"move_local_vel"` → `client.send_velocity_target_body_ned()`
  - `"land_on_target"` → `client.land_on_target()`

**Function: `create_command(action, **kwargs)`**
- Helper for the state machine. Creates a command dict with an auto-injected timestamp.
- Usage: `command_queue.put(create_command("move_local_vel", vx=0.5, vy=0.0, vz=0.0))`

---

### `src/ros_bridge/` — ROS 2 Integration Layer

#### `src/ros_bridge/vision_subscriber.py` ✅ IMPLEMENTED
**Role:** ROS 2 bridge that subscribes to `/erc/vision_targets` and provides a clean polling API to the state machine. Encapsulates all `rclpy` complexity so the state machine never imports ROS 2 directly.

**Internal Class: `_VisionSubscriberNode(Node)`**
- An internal `rclpy.Node` not exposed outside the module.
- Creates a subscription to the configured topic with QoS queue depth 10.

**Public Class: `VisionBridge`**

*Constructor:* `VisionBridge(topic, grid_config, detection_timeout_s)`
- `topic` — ROS 2 topic name (default: `/erc/vision_targets`).
- `grid_config` — the `grid` section from `mission_params.yaml`. If provided, enables automatic probe-to-sector mapping.
- `detection_timeout_s` — how many seconds before a detection is considered stale (default: 0.5s).

*Key Methods:*
- **`spin_once()`** — Non-blocking. Calls `rclpy.spin_once(node, timeout_sec=0)` to process any pending DDS messages. Must be called on every main loop iteration.
- **`get_latest_target()`** → `dict | None` — Returns the most recent ArUco marker detection, or `None` if nothing detected or last detection is stale. Returns:
  ```python
  {"marker_id": 102, "x_offset_m": 0.15, "y_offset_m": -0.08, "age_s": 0.05}
  ```
- **`get_detected_probes()`** → `list[str]` — Returns sorted, deduplicated list of probe sector IDs (e.g., `["A2", "C4"]`).
- **`clear_probes()`** — Resets probe accumulator (call between mission attempts).
- **`get_message_count()`** → `int` — Total messages received (diagnostics).
- **`shutdown()`** — Destroys the internal ROS 2 node.

*Internal Callback: `_on_vision_msg(msg)`*
- Routes based on `msg.z` value:
  - `z = 101 or 102` → stores as latest marker target with timestamp.
  - `z < 0` → probe detection, passes `(msg.x, msg.y)` to `GridMapper`, accumulates sector ID.
  - `z = 0` → no detection, silently ignored.

**Key design decision:** Uses `spin_once()` (non-blocking, single-threaded) instead of `rclpy.spin()` in a thread. This avoids threading + multiprocessing + serial = debugging nightmare. The main loop calls `spin_once()` at ~50 Hz, which is fast enough for 10–30 Hz vision data.

---

### `src/navigation/` — Autonomous Flight Logic

#### `src/navigation/state_machine.py` ⬜ NOT YET IMPLEMENTED
**Role:** The core autonomous flight state machine. Implements the Search → Align → Land mission cycle.

**Planned design:**
- **`FlightState` enum:** IDLE, TAKEOFF, SEARCH, ALIGN, LAND, COMPLETE, MISSION_DONE.
- **`MissionController` class:** Holds references to `command_queue`, `telemetry_queue`, `VisionBridge`, and config. Exposes a single `update()` method called 50 times/second by the main loop.
- **`update()` is NOT about changing states** — it executes the *work* for the current state (send velocity commands, check altitude, read vision) and checks if the exit condition for that state is met. Most calls return without changing state.

**Per-state behavior:**
- **IDLE:** Wait for EKF healthy + telemetry connected, then → TAKEOFF.
- **TAKEOFF:** Arm → LOITER → GUIDED → takeoff command → monitor altitude. Condition: altitude ≥ 90% target → SEARCH. Timeout: 15s → COMPLETE (failed).
- **SEARCH:** Hover at search altitude, check vision for marker 102. Condition: marker 102 detected → ALIGN. Timeout: 60s → LAND. (Search sweep pattern to be added later.)
- **ALIGN:** Send velocity corrections using proportional controller: `vx = Kp * y_offset`, `vy = Kp * x_offset`. Condition: centered within 5 cm for 1 second → LAND. Lost target > 0.5s → SEARCH. Timeout: 10s → LAND.
- **LAND:** Send `LANDING_TARGET` messages for precision landing. Condition: altitude ≈ 0 + disarmed → COMPLETE. Timeout: 15s → COMPLETE.
- **COMPLETE:** Increment attempt counter. Attempts < 3 → IDLE. Attempts = 3 → MISSION_DONE.

---

### `src/utils/` — Utility Modules

#### `src/utils/grid_mapper.py` ✅ IMPLEMENTED
**Role:** Converts probe world coordinates (x, y) in meters to competition grid sector IDs.

**Class: `GridMapper`**

*Constructor:* `GridMapper(grid_config)` — takes the `grid` section from `mission_params.yaml`.

*Coordinate convention:*
- X axis (North in NED) → mapped to columns (A, B, C, ...).
- Y axis (East in NED) → mapped to rows (1, 2, 3, ...).
- Grid origin `(origin_x_m, origin_y_m)` is the bottom-left corner of the grid, relative to the takeoff pad center.

*Key Methods:*
- **`position_to_sector(x_m, y_m)`** → `str | None` — Converts world position to sector ID (e.g., `"A2"`). Returns `None` if outside grid boundaries. Uses floor division for positions on cell boundaries.
- **`get_grid_bounds()`** → `dict` — Returns `{x_min, x_max, y_min, y_max}` for the grid extent.

**Tested:** 14 unit tests covering basic mapping, boundary cases, out-of-bounds positions, and custom configurations. All passing.

---

### `config/` — Configuration Files

#### `config/mission_params.yaml`
**Role:** Single source of truth for all tunable mission parameters.

**Sections:**
- `startup` — Pixhawk initialization delay (seconds to wait for heartbeat + EKF convergence after boot).
- `flight` — Altitudes (takeoff, search, approach), speed limits.
- `markers` — ArUco IDs (101, 102), marker physical size, dictionary name.
- `mission` — Number of landing attempts, search radius, success radius.
- `grid` — Grid origin, cell size, column/row labels for probe reporting.
- `serial` — UART port and baud rate for Pixhawk connection.
- `ros2` — Vision topic name, DDS domain ID.
- `timeouts` — Maximum durations for each flight phase before abort.

#### `config/camera_calibration.npz`
**Role:** NumPy archive containing the Arducam IMX708 camera intrinsic parameters.

**Contents:**
- `camera_matrix` — 3×3 intrinsic matrix (focal lengths, principal point).
- `dist_coeffs` — Distortion coefficients.

**Used by:** The Simulink vision node (for ArUco pose estimation via `solvePnP`) and potentially the grid mapper (for pixel-to-meter conversion).

---

### `scripts/` — Operational Scripts

#### `scripts/install_ros2_jazzy.sh`
**Role:** The ERC organizers' ROS 2 Jazzy installation script for Raspberry Pi 5 / Debian 13 Trixie.

**Run once:** During initial Pi setup. Creates `~/ros2_ws/`, installs `ros-jazzy-ros-base`, configures `~/.bashrc`.

**Not part of normal operations.** After successful installation, this script is never run again.

#### `scripts/start_mission.sh`
**Role:** One-command mission launcher for competition day.

**What it does:**
1. Sources the ROS 2 environment.
2. Sets `ROS_DOMAIN_ID` to avoid cross-talk.
3. Launches `main.py` with `sudo -E` (for UART access, preserving environment).

#### `scripts/run_composite_output.sh`
**Role:** Launches the downward camera feed over the Pi's composite video output for the analog video switcher.

**Copied from:** The old `indomitus-drone-ros2` repository. Used when the pilot needs to see the downward camera on their FPV monitor.

---

### `tests/` — Test Modules

#### `tests/test_dummy_publisher.py` ✅ IMPLEMENTED
**Role:** Simulates the Simulink vision node by publishing fake target coordinates to `/erc/vision_targets` at 10 Hz.

**Supports 5 simulation modes** (selected via `--mode` CLI argument):

| Mode | What it simulates | Tests |
|------|-------------------|-------|
| `circle` (default) | Marker 102 drifting in a 0.3 m radius circle | ALIGN convergence — state machine must follow a moving target |
| `static` | Marker 102 at a fixed (0.15, -0.10) m offset | Steady-state ALIGN + LAND transition |
| `approach` | Marker 102 exponentially converging to center over ~10 s | Successful alignment sequence |
| `probes-only` | Only probe detections (z = -1.0), no markers | SEARCH timeout + probe sector accumulation |
| `intermittent` | Marker 102 visible for 3 s, gone for 2 s, repeating | Lost-target recovery (ALIGN → SEARCH → ALIGN) |

**Usage:** `python3 tests/test_dummy_publisher.py --mode circle --topic /erc/vision_targets`

#### `tests/test_grid_mapper.py` ✅ IMPLEMENTED
**Role:** 14 unit tests for the `GridMapper` class. Covers basic mapping, cell boundaries, out-of-bounds positions, custom configs, and edge cases. Runs standalone or with pytest.

---

### `docs/` — Documentation

#### `docs/SETUP.md`
**Role:** Complete Raspberry Pi 5 configuration guide. Covers OS verification, ROS 2 installation, `rclpy` setup, UART permissions, and MATLAB Blockset connection.

#### `docs/ARCHITECTURE.md`
**Role:** This file. The master architecture reference.

---

## 11. External Dependencies & Filesystem Layout

### On the Raspberry Pi

```
/opt/ros/jazzy/                          # System-wide ROS 2 installation
├── setup.bash                           # Environment setup script
├── lib/python3.13/site-packages/        # rclpy, std_msgs, geometry_msgs, etc.
└── lib/demo_nodes_cpp/                  # Talker/listener for testing

/home/marko/
├── indomitus-drone/                     # THIS REPOSITORY (git clone)
│   └── .venv/                           # Python venv (--system-site-packages)
├── ros2_ws/                             # ROS 2 colcon workspace
│   ├── src/                             # Simulink-deployed C++ packages go here
│   ├── build/                           # Compiled output
│   └── install/                         # Runnable ROS 2 nodes
└── ros2_jazzy_install_logs/             # Installation logs
```

### Key Distinction

| Directory | Managed By | Contains | Git-Tracked? |
|-----------|-----------|----------|--------------|
| `~/indomitus-drone/` | You (developer) | Python source code, configs, scripts, docs | ✅ Yes |
| `~/ros2_ws/` | Simulink Blockset + colcon | Compiled C++ ROS 2 nodes from Simulink | ❌ No |
| `/opt/ros/jazzy/` | APT package manager | System ROS 2 libraries | ❌ No |

---

## 12. Startup & Deployment Sequence

### Competition Day Sequence

```
Step 1: Power on Pi, SSH in as marko
Step 2: Verify Pixhawk UART connection is live
Step 3: Start the Simulink vision node
           ros2 run <simulink_pkg> <node_name> &
Step 4: Start the mission
           cd ~/indomitus-drone
           ./scripts/start_mission.sh
Step 5: Operator confirms telemetry + video feed
Step 6: Operator triggers mission start (or the script auto-starts)
Step 7: Drone executes: Takeoff → Search → Align → Land (×3)
Step 8: System reports probe detections
```

### Development Sequence

```
Step 1: SSH in as marko
Step 2: source ~/.bashrc  (loads ROS 2 env)
Step 3: cd ~/indomitus-drone && source .venv/bin/activate

# Terminal 1 — Run dummy vision publisher (simulates Simulink)
python3 tests/test_dummy_publisher.py

# Terminal 2 — Run the main application
sudo -E python3 main.py

# Terminal 3 — Monitor ROS 2 topics
ros2 topic echo /erc/vision_targets
ros2 topic hz /erc/vision_targets
```

---

## 13. Configuration Reference

### ArduCopter Parameters (set via Mission Planner)

These are configured on the Pixhawk, not in this codebase, but are critical to the system:

| Parameter | Value | Purpose |
|-----------|-------|---------|
| `SERIAL2_PROTOCOL` | 2 (MAVLink2) | Pi UART connection protocol |
| `SERIAL2_BAUD` | 921600 | Matches Python `baudrate` |
| `PLND_ENABLED` | 1 | Enable precision landing |
| `PLND_TYPE` | 1 (Companion) | Landing target from companion computer |
| `PLND_EST_TYPE` | 0 (Raw) | Use raw sensor data for landing target |
| `FLTMODE_CH` | 5 (or configured) | RC channel for flight mode switching |
| `GCS_TYPE` | 1 | Accept GCS heartbeats from companion |

### Environment Variables

| Variable | Value | Set By |
|----------|-------|--------|
| `MAVLINK20` | `1` | `mavlink_client.py` (in code) |
| `ROS_DOMAIN_ID` | `42` | `start_mission.sh` |
| `ROS_DISTRO` | `jazzy` | `~/.bashrc` (auto-set by ROS 2 setup) |

---

## 14. Implementation Status & TODO

### Current Status (as of 2026-08-08)

| File | Status | Notes |
|------|--------|-------|
| `src/comm/mavlink_client.py` | ✅ Complete | 580 lines, flight-tested, copied from old repo |
| `src/comm/mavlink_node.py` | ✅ Complete | Process loop + command dispatcher, copied from old repo |
| `src/ros_bridge/vision_subscriber.py` | ✅ Complete | VisionBridge class with marker tracking + probe accumulation |
| `src/utils/grid_mapper.py` | ✅ Complete | GridMapper class, 14 unit tests passing |
| `src/navigation/state_machine.py` | ⬜ Empty | Core autonomous flight logic — highest priority |
| `main.py` | ⬜ Empty | Application entry point wiring everything together |
| `tests/test_dummy_publisher.py` | ✅ Complete | 5 simulation modes for bench testing |
| `tests/test_grid_mapper.py` | ✅ Complete | 14 tests, all passing |
| `config/mission_params.yaml` | ✅ Complete | All parameters defined including startup delay |
| `docs/ARCHITECTURE.md` | ✅ Complete | This file |
| `docs/SETUP_RASPBERRY.md` | ✅ Complete | Raspberry Pi 5 setup guide |
| `scripts/start_mission.sh` | ✅ Complete | One-command launcher |
| `scripts/install_ros2_jazzy.sh` | ✅ Complete | ROS 2 installer, already run successfully on Pi |

### TODO — Files to Implement

#### `src/navigation/state_machine.py` — Priority: HIGH

The core autonomous flight state machine. Must implement:

1. **`FlightState` enum** — IDLE, TAKEOFF, SEARCH, ALIGN, LAND, COMPLETE, MISSION_DONE.
2. **`MissionController` class** with an `update()` method called ~50 times/second.
3. **Per-state `_update_*()` methods** — each executes the work for that state and checks exit conditions.
4. **State transition logic** — see Section 8 for the full state diagram and transition conditions.
5. **Telemetry refresh** — drain `telemetry_queue` on each tick, keep only the latest.

#### `main.py` — Priority: HIGH

The application entry point. Must implement:

1. Load `config/mission_params.yaml` with PyYAML.
2. Call `rclpy.init()`.
3. Create `multiprocessing.Queue` instances for telemetry and commands.
4. Spawn the MAVLink comm process (`comm_process_loop` from `mavlink_node.py`).
5. Wait `startup.pixhawk_init_delay_s` seconds for Pixhawk connection.
6. Create `VisionBridge` instance.
7. Create `MissionController` instance.
8. Run main loop at ~50 Hz: call `mission.update()` until `MISSION_DONE`.
9. Handle `Ctrl+C` graceful shutdown: send LAND command, terminate processes.

### TODO — Features to Add Later

#### Search Sweep Pattern — Priority: MEDIUM

The SEARCH state currently just hovers and waits for the marker to appear.
Given the 120° wide-angle lens at 2 m altitude (~7 × 5 m coverage), this
may actually work without a sweep. However, if testing shows the camera
can't see the disc from the takeoff position, implement one of:

- **Expanding square spiral** — move in an expanding square pattern around the takeoff point.
- **Grid sweep** — visit a grid of waypoints within the 3 m search radius.
- **Random walk** — move to random positions within bounds (simplest but least efficient).

Use `send_local_ned_position_target()` to command waypoint movements.

#### PID Controller for ALIGN — Priority: MEDIUM

The ALIGN state currently uses a simple proportional (P-only) controller:
```python
vx = Kp * y_offset
vy = Kp * x_offset
```

This may cause oscillation if `Kp` is too high, or slow convergence if too low.
Improve to full PID if flight testing shows instability:

- **P** — proportional to offset (already implemented).
- **I** — integral of offset over time (corrects steady-state drift from wind).
- **D** — derivative of offset change rate (dampens oscillation).

Note: The Pixhawk's internal GUIDED-mode PID also contributes damping,
so the outer loop (our controller) may work fine as P-only. Test first.

#### Precision Landing Refinement — Priority: MEDIUM

The LAND state must continuously send `LANDING_TARGET` MAVLink messages
to the Pixhawk with the marker's position in BODY_FRD frame. The frame
conversion from camera offsets to body frame needs:

```python
target_frd = (
    vision_target["y_offset_m"],     # camera Y → body forward
    vision_target["x_offset_m"],     # camera X → body right
    -telemetry["pos_z_m"]            # altitude (NED z is negative when up)
)
```

#### Custom ROS 2 Message Type — Priority: LOW

Currently using `geometry_msgs/msg/Point` with the `z` field encoding
detection type (marker ID, probe flag, or no-detection). This works but
is fragile. A proper approach would be a custom `.msg` definition:

```
# VisionTarget.msg
float64 x_offset       # meters
float64 y_offset       # meters
int32   marker_id      # 101, 102, or 0
bool    is_probe       # true if this is a probe detection
float64 confidence     # 0.0–1.0
```

This requires creating a ROS 2 package with message generation. Only worth
doing if the Simulink team builds it into their model.

#### Simulink Vision Model — Priority: HIGH (parallel track)

The Simulink model must be designed in MATLAB and deployed to `~/ros2_ws/`.
It needs to:

1. Capture camera frames via V4L2 (`/dev/video0`).
2. Detect ArUco markers (IDs 101, 102) using Computer Vision Toolbox.
3. Estimate marker pose using `solvePnP` with `config/camera_calibration.npz`.
4. Detect probes (color/shape-based algorithm — TBD).
5. Publish results to `/erc/vision_targets` as `geometry_msgs/msg/Point`.

This is a separate development track done in MATLAB, not in this Python repo.
Until the Simulink model is ready, use `tests/test_dummy_publisher.py` as
a stand-in.

#### Web UI / Telemetry Dashboard — Priority: LOW

The old repo had a `web_ui/` module with a Flask server for real-time
telemetry display. Evaluate whether this is needed for competition.
The pilot already has FPV video and Mission Planner telemetry, so a
custom dashboard may be unnecessary.
