# Architecture — Indomitus Drone

> This document is the single source of truth for the system architecture of the **Indomitus Drone** autonomous quadcopter, built for the **European Rover Challenge (ERC) 2026 — Droning Sub-Task**. It provides complete, up-to-date context on the hardware, Simulink ROS 2 nodes, Python execution bridge, and flight stack.

---

## Table of Contents

1. [Mission Context](#1-mission-context)
2. [Hardware Stack](#2-hardware-stack)
3. [System Architecture Overview](#3-system-architecture-overview)
4. [Layer 1 — Simulink C++ ROS 2 Nodes (Vision & Mission Supervisor)](#4-layer-1--simulink-c-ros-2-nodes-vision--mission-supervisor)
5. [Layer 2 — Python Execution Bridge & MAVLink Driver](#5-layer-2--python-execution-bridge--mavlink-driver)
6. [Layer 3 — Pixhawk Flight Controller (ArduCopter)](#6-layer-3--pixhawk-flight-controller-arducopter)
7. [Communication Protocols & ROS 2 Topics](#7-communication-protocols--ros-2-topics)
8. [Mission State Machine & Flight Behavior](#8-mission-state-machine--flight-behavior)
9. [Coordinate Systems & Frames of Reference](#9-coordinate-systems--frames-of-reference)
10. [Repository File Map](#10-repository-file-map)
11. [External Dependencies & Filesystem Layout](#11-external-dependencies--filesystem-layout)
12. [Startup & Deployment Sequence](#12-startup--deployment-sequence)
13. [Configuration Reference](#13-configuration-reference)

---

## 1. Mission Context

### 1.1 Competition Rules

The ERC Droning Sub-Task takes place inside a **10 × 10 × 4 m enclosed cage** (or outdoor netted arena). The drone must operate **fully autonomously** during each flight attempt — manual intervention during autonomous execution aborts scoring for that attempt.

**Core mission (3 separate single-run attempts):**
1. Lift off from a 1 × 1 m takeoff pad marked with **ArUco ID 101** (15 × 15 cm, `DICT_ARUCO_ORIGINAL`).
2. Search for a landing disc (radius 0.5 m) placed within 3 m of the takeoff pad, marked with **ArUco ID 102** (15 × 15 cm).
3. Precision land on the disc. Landing is successful if any part of the drone touches the disc.
4. After touchdown and disarm, the program exits (`COMPLETE`). The drone is **manually repositioned** back to the takeoff pad, and the mission script is restarted for the next attempt.

**Additional scoring — Probe Detection:**
- Up to 3 probes are scattered within the 3 m radius area.
- Detected world positions relative to the takeoff pad can be mapped to a 1 × 1 m alphanumeric grid (columns `A–F`, rows `1–6`, e.g., `"A2"`).

**Additional scoring — Custom Landing Platform:**
- Teams may prepare their own landing target (within a 0.5 m radius disc) attachable to their rover.

### 1.2 Time Constraints

| Phase | Duration |
|-------|----------|
| Preparation (outside cage) | 15 minutes |
| Mission execution (all 3 takeoff–search–land runs) | 30 minutes |

### 1.3 Preflight Safety Requirements

Verified by the jury before flight:
- Real-time telemetry + video feed to the operator.
- Hardware **status LED indicator** clearly distinguishing manual standby (blinking Green) from autonomous flight (blinking Red).
- Instant switch to manual/remote control on demand (pilot RC mode toggle immediately disengages Python/Simulink autonomy).
- Automatic mid-air stability holding (`LOITER` mode) and automatic landing on demand (`LAND` / `BRAKE`).
- Failsafes for RC loss, GCS heartbeat loss, low battery, EKF glitch, and arena drift.
- Inclusion geofence configured in Mission Planner (the sole reason an onboard GPS receiver is installed) plus an 8 m local-origin software geofence.

---

## 2. Hardware Stack

### 2.1 Airframe & Propulsion

| Component | Specification |
|-----------|---------------|
| Frame | F450 quadcopter with landing gears |
| Motors | DJI 2212 920KV brushless (×4) |
| Propellers | 1045R (10 × 4.5) |
| Battery | 4S LiPo |

### 2.2 Computing & Status Indication

| Component | Role | Connection |
|-----------|------|------------|
| **Pixhawk 6C** | Low-level flight controller (ArduCopter firmware) | — |
| **Raspberry Pi 5** (16 GB) | Companion computer — runs Simulink C++ ROS 2 nodes, camera pipeline, web HUD, and Python bridge | UART (`/dev/ttyAMA0`) to Pixhawk; Composite Video to VTX switcher |
| **5V 5A UBEC** | Powers the Raspberry Pi 5 from the 4S LiPo | — |
| **WS2812 5050 RGB LED Module** (7 LEDs) | Safety autonomy status indicator (Green blink = Manual/Standby, Red blink = Autonomous) | Raspberry Pi 5 Hardware SPI MOSI (`GPIO 10` / Physical Pin 19, `/dev/spidev0.0`) |

### 2.3 Sensors

| Sensor | Direction | Connection | Purpose |
|--------|-----------|------------|---------|
| **Arducam IMX708** (12MP HDR 120°) | Downward | Pi CSI ribbon cable | Computer vision (ArUco 101/102 detection & pose estimation; currently run in **cropped `640 × 480`** mode — see note below) |
| **Foxeer Cat 4 FPV** | Forward | **3-ch Video Switcher / VTX ONLY** (NOT connected to Pi or Pixhawk) | Human pilot's forward FPV view only — **cannot be used for CV** |
| **Microair MTF-01** | Downward | Pixhawk serial | Optical flow + rangefinder for indoor / GPS-denied position hold & navigation |
| **Holybro M10** | Upward | Pixhawk I2C/serial | **Used ONLY for the Mission Planner geofence** (and compass); NOT used for autonomous navigation |
| **Pixhawk IMU** | Internal | Onboard | Accelerometer + gyroscope for EKF3 attitude/motion fusion |

> [!IMPORTANT]
> - **Cropped CV Camera Resolution (Small Effective FOV):** Although the Arducam IMX708 lens has a physical **120° wide-angle FOV** on its full `4608 × 2592` sensor, the current pipeline captures at **`640 × 480`** without forcing a full-sensor `libcamera` binning mode (and `camera_calibration.npz` was calibrated on a cropped `1280 × 720` mode with ~72.5° × 45.3° FOV). As a result, `rpicam-vid` uses a **center-cropped region of the sensor**, giving a much narrower ground footprint (~2 × 1.8 m at 2.2 m altitude) rather than the full 120° wide angle. This is why the drone currently flies a multi-waypoint search grid instead of seeing the entire arena from hover.
> - **GPS Role:** The Holybro M10 GPS module is installed **strictly to satisfy the Mission Planner geofence requirement**. All local position estimates (`LOCAL_POSITION_NED`) used during autonomous flight rely on the **Microair MTF-01 optical flow + rangefinder** fused with the onboard IMU in EKF3.

### 2.4 Communication Links

| Link | Hardware | Frequency | Protocol | Purpose |
|------|----------|-----------|----------|---------|
| RC Control | RadioMaster RP1 V2 | 2.4 GHz (ELRS) | ExpressLRS | Pilot manual control & instant autonomy override |
| Telemetry | 3DR Radio V5 | 433 MHz | MAVLink | Mission Planner ground station telemetry & geofence |
| Video (FPV / Pi UI) | Race Ranger VTX | 5.8 GHz | Analog PAL/NTSC | Live video to ground monitor / FPV goggles |
| Pi ↔ Pixhawk | Hardwired UART | N/A | MAVLink 2.0 (`921600` baud) | Companion computer telemetry & flight commands |

### 2.5 Video Switching & Ground UI Access via VTX

A **3-channel video switcher** feeds the 5.8 GHz Race Ranger VTX and lets the operator toggle live between:
1. **Forward Foxeer Cat 4 FPV camera:** Completely isolated from both the Raspberry Pi 5 and Pixhawk 6C. Used exclusively by the safety pilot for manual FPV flying; unavailable to any software or CV pipeline.
2. **Raspberry Pi 5 composite video output:** Transmits the live **Raspberry Pi desktop UI** directly to the ground station screen over the analog VTX link. This provides a zero-network fallback in the field — by simply plugging a wireless/USB mouse into the Raspberry Pi, the operator can view the desktop via VTX to switch Wi-Fi/hotspot connections, inspect logs, or launch the autonomous mission directly without needing SSH.

The VTX path is completely isolated from the Pixhawk — there is **no OSD overlay**. When Wi-Fi is connected, operators can also view the downward CV camera + live target overlay in a browser via `scripts/vision_web_streamer.py` (`http://<pi-ip>:5000`).

---

## 3. System Architecture Overview

To satisfy the ERC requirement that both **computer vision** and **high-level mission decision-making** run in **MATLAB/Simulink**, the companion computer runs **two compiled Simulink C++ ROS 2 nodes** alongside a **Python execution & hardware bridge**:

```
┌────────────────────────────────────────────────────────────────────────────────────────┐
│                                    RASPBERRY PI 5                                      │
│                                                                                        │
│  [Arducam IMX708]                                                                      │
│         │ (rpicam-vid + GStreamer tee + v4l2loopback via start_vision.sh)              │
│         ├───────────────────────────────► /dev/video11 ──► vision_web_streamer.py      │
│         ▼                                                   (HTTP MJPEG HUD :5000)     │
│    /dev/video10                                                                        │
│         │                                                                              │
│         ▼                                                                              │
│  ┌──────────────────────────┐   /erc/vision_targets    ┌────────────────────────────┐  │
│  │ SIMULINK NODE 1 (EYES)   │  (Float64MultiArray[7])  │ SIMULINK NODE 2 (BRAIN)    │  │
│  │ erc_vision_node (15 Hz)  ├─────────────────────────►│ erc_mission_node (15 Hz)   │  │
│  │                          │            │             │                            │  │
│  │ Detects ArUco 101 & 102  │            │             │ Supervisory State Machine  │  │
│  │ Outputs: marker_id,      │            │             │ States: 0..4 (Int32)       │  │
│  │   tvec[1x3], rvec[1x3]   │            │             └──────▲──────────────┬──────┘  │
│  └──────────────────────────┘            │                    │              │         │
│                                          │     /erc/drone_telemetry   /erc/mission_state
│                                          │     (geometry_msgs/Pose)   (std_msgs/Int32) │
│                                          ▼                    │              ▼         │
│                                ┌──────────────────────────────┴──────────────┐         │
│                                │         PYTHON EXECUTION BRIDGE             │         │
│                                │  main.py (50 Hz)                            │         │
│                                │  - VisionBridge (rclpy pub/sub)             │         │
│                                │  - MissionController (executes states,      │         │
│                                │    48-step search rings, safety overrides)  │         │
│                                │  - LEDController (SPI WS2812 Green/Red)     │         │
│                                └──────────────────────┬──────────────────────┘         │
│                                                       │ multiprocessing.Queue          │
│                                ┌──────────────────────▼──────────────────────┐         │
│                                │         MAVLINK COMM DAEMON                 │         │
│                                │  mavlink_node.py + mavlink_client.py        │         │
│                                └──────────────────────┬──────────────────────┘         │
│                                                       │ UART /dev/ttyAMA0 @ 921600     │
└───────────────────────────────────────────────────────┼────────────────────────────────┘
                                                        ▼
                                       ┌────────────────────────────────┐
                                       │       PIXHAWK 6C (MUSCLE)      │
                                       │  ArduCopter + EKF3             │
                                       │  - Optical flow (MTF-01) + IMU │
                                       │  - Built-in Precision Landing  │
                                       │  - GPS (M10) -> Geofence ONLY  │
                                       └────────────────────────────────┘
```

---

## 4. Layer 1 — Simulink C++ ROS 2 Nodes (Vision & Mission Supervisor)

Both Simulink models are stored in [`matlab/`](../matlab/), auto-coded to C++ via **Simulink Coder / ROS Toolbox**, and built into `~/ros2_ws/` on the Raspberry Pi using [`matlab/build_ros2_model.sh`](../matlab/build_ros2_model.sh) (`colcon build`). All ROS 2 nodes communicate on **`ROS_DOMAIN_ID = 27`**.

### 4.1 Camera Loopback Pipeline (`scripts/start_vision.sh`)

Because Linux V4L2 allows only one process to open a hardware camera node exclusively, [`scripts/start_vision.sh`](../scripts/start_vision.sh) loads `v4l2loopback` and creates two virtual video devices fed by a single `rpicam-vid` + GStreamer `tee` pipeline (**center-cropped `640 × 480`** @ 15 FPS, 16 ms shutter — does not currently use the full 120° sensor area):
- **`/dev/video10` (`SimulinkCam`):** Read by `erc_vision_node`.
- **`/dev/video11` (`WebStreamCam`):** Read by [`scripts/vision_web_streamer.py`](../scripts/vision_web_streamer.py), which overlays live `VisionBridge` target vectors onto an MJPEG web stream at `http://<pi-ip>:5000`.

### 4.2 Node 1: `erc_vision_node` (`matlab/erc_vision_node.slx`)

**Role:** Real-time ArUco detection and 6-DoF pose estimation at **15 Hz**.

**How it works internally:**
1. **V4L2 Video Capture block:** Reads planar `R`, `G`, `B` matrices (`640 × 480`) from `/dev/video10` every `1/15` s.
2. **Calibration Parameters:** Loaded at build/init time from `config/camera_calibration.npz` via [`matlab/load_calibration_npz.m`](../matlab/load_calibration_npz.m), scaling `focalLength`, `principalPoint`, `radialDistortion`, and `tangentialDistortion` to `[480, 640]`.
3. **MATLAB Function (`detectAndEstimate`):**
   - Reconstructs the 3-channel RGB frame and builds a `cameraIntrinsics` object.
   - Runs `readArucoMarker(frame, "DICT_ARUCO_ORIGINAL", intrinsics, 0.15)` for 15 cm markers.
   - **Priority logic:** Searches detected IDs for landing target **`102`** first; falls back to takeoff pad **`101`** if `102` is not visible; ignores any other IDs. If nothing is detected, outputs all zeros (`marker_id = 0`).
   - Computes the 3D translation vector and converts the 3×3 pose rotation matrix into a `1×3` rotation vector via `rotmat2vec3d`.
   - Transforms lateral camera offsets directly into the drone's **`BODY_FRD`** coordinate convention (`tvec = [forward_m, right_m, down_m]`).
4. **Vector Packing & ROS 2 Publish (`/erc/vision_targets`):**
   - Simulink concatenates the **3 output variables** — `marker_id` (`1×1` scalar), `tvec` (`1×3` vector), and `rvec` (`1×3` vector) — into a single **length-7 `double` array**:
     ```text
     data = [marker_id, tx, ty, tz, rx, ry, rz]
     ```
   - Published on `/erc/vision_targets` as `std_msgs/msg/Float64MultiArray` (`CurrentLength = 7`).

### 4.3 Node 2: `erc_mission_node` (`matlab/erc_mission_node.slx`)

**Role:** High-level supervisory finite state machine running at **15 Hz**. All autonomous mission state transitions originate from this node.

**How it works internally:**
1. **ROS 2 Subscriptions:**
   - Subscribes to `/erc/drone_telemetry` (`geometry_msgs/msg/Pose` published by Python) and uses a `Bus Selector` to unpack 7 telemetry signals: `alt`, `is_armed`, `ekf_healthy`, `origin_dist`, `connected`, `manual_override`, and `waypoints_exhausted`.
   - Subscribes to `/erc/vision_targets` (`std_msgs/msg/Float64MultiArray`) and uses a `Selector` block (index `1`) to extract `marker_id`.
2. **MATLAB Function (`Mission_State_Supervisor`):**
   - Maintains persistent state (`current_state`), per-state elapsed timer (`state_timer_s`), and a debounce counter for marker `102` (`detect_confirm_ticks`: increments by `+1` when `marker_id == 102`, decays by `-2` down to `0` when absent, requiring `15` ticks to confirm target lock).
   - **Global safety checks:**
     - If `manual_override > 0.5` → immediately transitions to `4` (`COMPLETE`) to disengage autonomy.
     - If `origin_dist > 8.0` m → immediately transitions to `3` (`DESCEND`) to force emergency landing.
   - **State transition guards:**
     - `0` (`IDLE`) → `1` (`TAKEOFF`): when `connected`, `ekf_healthy`, and `origin_dist <= 6.0` m.
     - `1` (`TAKEOFF`) → `2` (`SEARCH`): when `alt >= 0.85 * 1.5` m (`1.275` m); or → `3` (`DESCEND`) on 30 s timeout.
     - `2` (`SEARCH`) → `3` (`DESCEND`): when marker `102` is confirmed (`detect_confirm_ticks >= 15`), or when `waypoints_exhausted` / 320 s search timeout expires.
     - `3` (`DESCEND`) → `4` (`COMPLETE`): when touchdown is confirmed (`alt < 0.25` m and `is_armed == 0`) or after 30 s descent timeout once disarmed.
3. **ROS 2 Publish (`/erc/mission_state`):**
   - Transmits **a single number (`int32`)** — the current state ID (`0`, `1`, `2`, `3`, or `4`) — on `/erc/mission_state` (`std_msgs/msg/Int32`).

---

## 5. Layer 2 — Python Execution Bridge & MAVLink Driver

Python runs as **two OS-level processes** (plus a background LED thread) communicating via `multiprocessing.Queue`:

```
┌──────────────────────────────────────────────────────────┐
│                 MAIN PROCESS (main.py, 50 Hz)            │
│                                                          │
│  ┌────────────────────────────────────────────────────┐  │
│  │ VisionBridge (src/ros_bridge/vision_subscriber.py) │  │
│  │  - Subscribes: /erc/vision_targets (7-float array) │  │
│  │  - Subscribes: /erc/mission_state  (Int32 state)   │  │
│  │  - Publishes:  /erc/drone_telemetry (Pose)         │  │
│  └─────────────────────────┬──────────────────────────┘  │
│                            ▼                             │
│  ┌────────────────────────────────────────────────────┐  │
│  │ MissionController (src/navigation/state_machine.py)│  │
│  │  - Follows Simulink state commands (0..4)          │  │
│  │  - Executes takeoff sequence, 48-step search grid, │  │
│  │    and 15 Hz LANDING_TARGET precision descent      │  │
│  │  - Enforces 8m geofence & RC override safety       │  │
│  └───────────┬────────────────────────────┬───────────┘  │
│              │                            ▼              │
│              │             ┌──────────────────────────┐  │
│              │             │ LEDController (SPI thread)│  │
│              │             │ Green=Manual, Red=Auto   │  │
│              │             └──────────────────────────┘  │
└──────────────┼───────────────────────────────────────────┘
               │ command_queue / telemetry_queue
               ▼
┌──────────────────────────────────────────────────────────┐
│           COMM PROCESS (src/comm/mavlink_node.py)        │
│  - Sends GCS heartbeat at 1 Hz (source_system=254)       │
│  - Reads Pixhawk UART telemetry -> telemetry_queue (10Hz)│
│  - Validates (<500ms age + armed/mode guards) & sends    │
│    MAVLink commands via PixhawkClient                    │
└──────────────────────────────────────────────────────────┘
```

---

## 6. Layer 3 — Pixhawk Flight Controller (ArduCopter)

The Pixhawk 6C runs **ArduCopter** firmware and manages flight dynamics:
- **EKF3 sensor fusion:** Fuses **Microair MTF-01 optical flow + rangefinder** and onboard IMU/compass into a local position/velocity/attitude estimate (`LOCAL_POSITION_NED`). *(Holybro M10 GPS is used strictly for the Mission Planner geofence, not indoor autonomy.)*
- **Built-in Precision Landing (`PLND_ENABLED=1`, `PLND_TYPE=1`):** During `LAND` mode, ArduCopter consumes `LANDING_TARGET` messages (`MAV_FRAME_BODY_FRD`) sent by the Raspberry Pi and steers the drone directly onto ArUco marker 102 using its internal PID loops — eliminating the need for a custom outer-loop alignment controller in Python.
- **Flight modes used:** `LOITER` (optical-flow arming & manual hold), `GUIDED` (autonomous takeoff & search waypoints), `LAND` (precision descent), `BRAKE` (emergency stop on geofence breach).

### MAVLink Messages (Pi ↔ Pixhawk)

| Direction | Message | Purpose |
|-----------|---------|---------|
| Pi → Pixhawk | `HEARTBEAT` (`source_system=254`) | 1 Hz companion heartbeat preventing GCS failsafe without colliding with Mission Planner (`255`) |
| Pi → Pixhawk | `SET_MODE` | Switches between `LOITER`, `GUIDED`, `LAND`, `BRAKE` |
| Pi → Pixhawk | `MAV_CMD_COMPONENT_ARM_DISARM` | Arms motors during takeoff sequence |
| Pi → Pixhawk | `MAV_CMD_NAV_TAKEOFF` | Climbs to `flight.takeoff_altitude_m` |
| Pi → Pixhawk | `MAV_CMD_NAV_LAND` | Initiates descent with precision landing active |
| Pi → Pixhawk | `SET_POSITION_TARGET_LOCAL_NED` | Relative body-offset 1 m search moves (`MAV_FRAME_BODY_OFFSET_NED`) |
| Pi → Pixhawk | `LANDING_TARGET` | Streams marker 102 `BODY_FRD` `[x, y, z]` coordinates (`LANDING_TARGET_TYPE_VISION_FIDUCIAL`) |
| Pixhawk → Pi | `HEARTBEAT`, `LOCAL_POSITION_NED`, `ATTITUDE`, `SYS_STATUS`, `EKF_STATUS_REPORT`, `RC_CHANNELS`, `STATUSTEXT` | Telemetry at 10–20 Hz |

---

## 7. Communication Protocols & ROS 2 Topics

### 7.1 ROS 2 DDS (`ROS_DOMAIN_ID = 27`)

- **Middleware:** FastDDS (ROS 2 Jazzy on Debian Trixie arm64)
- **Domain ID:** `27` (set in `config/mission_params.yaml`, `scripts/start_vision.sh`, and terminal launch commands)
- **QoS:** `RELIABLE`, `KEEP_LAST`, depth `10`

| Topic | Message Type | Publisher | Subscriber(s) | Payload Format |
|-------|-------------|-----------|---------------|----------------|
| `/erc/vision_targets` | `std_msgs/msg/Float64MultiArray` | `erc_vision_node` (15 Hz) | `VisionBridge`, `erc_mission_node`, `vision_web_streamer.py` | 7 floats: `[marker_id, tx, ty, tz, rx, ry, rz]` (`101`, `102`, `0` = none, `<0` = probe) |
| `/erc/drone_telemetry` | `geometry_msgs/msg/Pose` | `VisionBridge` (50 Hz) | `erc_mission_node` | `position.{x: alt, y: armed, z: ekf_healthy}`, `orientation.{x: origin_dist, y: connected, z: manual_override, w: waypoints_exhausted}` |
| `/erc/mission_state` | `std_msgs/msg/Int32` | `erc_mission_node` (15 Hz) | `VisionBridge` | Single integer state ID: `0=IDLE`, `1=TAKEOFF`, `2=SEARCH`, `3=DESCEND`, `4=COMPLETE` |

### 7.2 MAVLink 2.0 & Multiprocessing Queues

- **UART:** `/dev/ttyAMA0` @ `921600` baud (`MAVLINK20=1`, `source_system=254`, `source_component=0`).
- **Command safety in `mavlink_node.py`:** Drops commands older than `500 ms` and blocks motion/landing commands if the drone is disarmed or not in `GUIDED` (except `send_landing_target` / `land_on_target`, which are permitted in `LAND` mode).

---

## 8. Mission State Machine & Flight Behavior

Each invocation of `main.py` executes **a single mission attempt** supervised by `erc_mission_node`:

```
  ┌──────────┐   Pixhawk + EKF healthy   ┌──────────┐   alt >= 85% target   ┌──────────┐
  │ 0: IDLE  ├──────────────────────────►│1: TAKEOFF├──────────────────────►│2: SEARCH │
  └──────────┘   (+ pilot sets GUIDED)   └────┬─────┘                       └────┬─────┘
                                              │ timeout (30s)                    │ marker 102 confirmed
                                              ▼                                  │ (or sweep exhausted)
                                         ┌──────────┐◄───────────────────────────┘
                                         │3: DESCEND│
                                         └────┬─────┘
                                              │ alt < 0.25m & disarmed
                                              ▼
                                         ┌──────────┐
                                         │4:COMPLETE│  (main.py exits cleanly)
                                         └──────────┘
```

### Detailed Execution per Phase (`MissionController`)

1. **Startup & Mode Gate:**
   - `main.py` waits `startup.start_delay_s` (`8 s` stationary countdown) and `startup.pixhawk_init_delay_s` (`6 s` for heartbeat + EKF).
   - `MissionController` stays gated (`_mission_innitialized = False`, LED blinking **Green**) until the operator switches the drone into `GUIDED` mode, preventing accidental motion on boot.
2. **`0: IDLE`:** Verifies Pixhawk heartbeat, EKF3 health flags, and local origin stability while streaming telemetry to Simulink.
3. **`1: TAKEOFF`:** Executes a 5-step arming & liftoff sequence: `LOITER` → wait 2 s → `ARM` (with 5 s retry) → `GUIDED` → wait 1 s → `takeoff(takeoff_altitude_m)`. LED switches to blinking **Red**.
4. **`2: SEARCH`:**
   - Hovers 3 s at center to stabilize, then flies a pre-computed **48-move expanding-ring serpentine pattern** covering a 7 × 7 m grid around the origin in 1 m body-frame steps (`Ring 1: 3×3`, `Ring 2: 5×5`, `Ring 3: 7×7`; 4 s travel + 3 s scan dwell per waypoint).
   - Whenever marker `102` is seen during `SEARCH`, Python immediately pre-warms ArduPilot's precision landing estimator with `send_landing_target([tx, ty, tz])`.
5. **`3: DESCEND`:**
   - Sends `MAV_CMD_NAV_LAND` on entry.
   - Streams `LANDING_TARGET` (`BODY_FRD`) updates throttled to **15 Hz** while above **`0.25 m`** altitude; below `0.25 m`, target updates stop so the drone commits cleanly to touchdown without close-range camera oscillation.
   - Supports recovery back to `SEARCH` (`GUIDED` + climb) if commanded by Simulink.
6. **`4: COMPLETE` (or Safety Override):**
   - Entered upon touchdown + disarm, or immediately if the pilot switches RC flight mode out of `GUIDED`/`LAND` (manual takeover). LED returns to **Green**, and `main.py` shuts down cleanly.
   - **8 m Software Geofence:** If horizontal distance from origin exceeds `8.0 m` in flight, Python immediately commands `BRAKE` → `LAND` and forces `DESCEND`.

---

## 9. Coordinate Systems & Frames of Reference

1. **Local NED (`LOCAL_POSITION_NED`):** Origin initialized where EKF3 boots (takeoff pad). `+X` = North/Forward, `+Y` = East/Right, `+Z` = Down (`pos_z_m` is negative above ground; `_get_altitude()` negates it).
2. **Body FRD (`MAV_FRAME_BODY_FRD` & `erc_vision_node` `tvec`):** Drone-relative frame used by `LANDING_TARGET` and output directly by `erc_vision_node`:
   - `tx` (`+X`): Forward from camera/drone center (meters).
   - `ty` (`+Y`): Right of camera/drone center (meters).
   - `tz` (`+Z`): Downward distance to the marker below the drone (meters, must be `> 0`).
3. **Competition Probe Grid (`GridMapper`):** Maps `(x_m, y_m)` local NED offsets from takeoff pad to 1 × 1 m sectors `A1–F6` using `grid.origin_x_m` and `grid.origin_y_m` in `config/mission_params.yaml`.

---

## 10. Repository File Map

```text
indomitus-drone/
├── main.py                              # Single-mission orchestrator (50 Hz loop, standby delay, Ctrl+C LAND)
├── requirements.txt                     # Python venv dependencies (pymavlink, numpy, pyyaml, opencv, SPI/LED)
├── config/
│   ├── mission_params.yaml              # Tunable flight, marker, grid, serial, ROS 2 (domain 27), timeout & LED params
│   ├── camera_calibration.npz           # Active Arducam IMX708 intrinsics & distortion coefficients
│   └── camera_calibration_old.npz       # Backup calibration file
├── docs/
│   ├── ARCHITECTURE.md                  # System architecture reference (this document)
│   ├── IMPROVEMENT_PLAN.md              # Audit of current code/architecture issues & roadmap
│   ├── SETUP_RASPBERRY.md               # Raspberry Pi 5 OS (Debian Trixie) + ROS 2 Jazzy setup guide
│   ├── usefull_commands.md              # 3-terminal quick-launch commands for flight operations
│   └── archive/                         # Archived legacy guides, specs, and retired helper scripts
├── hardware/
│   ├── README.md                        # Hardware directory overview
│   ├── cad/                             # 3D CAD models and printable mounts
│   ├── schematics/                      # Wiring diagrams and electrical schematics
│   └── datasheets/                      # Component datasheets and BOM
├── matlab/
│   ├── erc_vision_node.slx              # Simulink ROS 2 vision model (/dev/video10 -> /erc/vision_targets)
│   ├── erc_mission_node.slx             # Simulink ROS 2 supervisory state machine (-> /erc/mission_state)
│   ├── load_calibration_npz.m           # Loads & scales camera_calibration.npz into Simulink workspace
│   ├── build_ros2_model.sh              # Extracts Simulink .tgz codegen archive & runs colcon build in ~/ros2_ws
│   ├── detectAndEstimate.m              # Standalone MATLAB prototype of ArUco detection function
│   ├── calibrate_camera.m               # Checkerboard calibration script
│   ├── generate_markers.m               # Generates ArUco 101 & 102 PNGs (aruco_101.png, aruco_102.png)
│   ├── desktop_prototype_static.m       # Desktop static-image ArUco test
│   └── desktop_prototype_webcam.m       # Desktop live-webcam ArUco test
├── scripts/
│   ├── start_vision.sh                  # Launches v4l2loopback tee + erc_vision_node + vision_web_streamer.py
│   ├── vision_web_streamer.py           # Multi-threaded HTTP MJPEG HUD server (:5000) reading /dev/video11
│   └── install_ros2_jazzy.sh            # Idempotent ROS 2 Jazzy C++ & rclpy installer for RPi OS Trixie
├── src/
│   ├── comm/
│   │   ├── mavlink_client.py            # PixhawkClient low-level pymavlink driver (UART, streams, LANDING_TARGET)
│   │   └── mavlink_node.py              # Isolated comm process loop, 1 Hz heartbeat, command safety guards
│   ├── navigation/
│   │   └── state_machine.py             # MissionController (5 states, Simulink sync, 48-step search, safety)
│   ├── ros_bridge/
│   │   └── vision_subscriber.py         # VisionBridge (subscribes vision + state, publishes Pose telemetry)
│   └── utils/
│       ├── grid_mapper.py               # World (x, y) -> competition sector (A1..F6) converter
│       └── led_indicator.py             # LEDController (RPi 5 Hardware SPI WS2812 driver + rpi_ws281x/mock fallback)
└── tests/                               # Unit tests and hardware/flight test scripts
```

---

## 11. External Dependencies & Filesystem Layout on Raspberry Pi

| Path on Raspberry Pi | Purpose | Git-Tracked Here? |
|----------------------|---------|-------------------|
| `~/indomitus-drone/` | Python flight bridge, scripts, configs, Simulink source models | Yes |
| `~/ros2_ws/` | Colcon workspace holding compiled C++ `erc_vision_node` & `erc_mission_node` | No (built via `build_ros2_model.sh`) |
| `/opt/ros/jazzy/` | System ROS 2 Jazzy installation (`rclpy`, `std_msgs`, `geometry_msgs`) | No (installed via `install_ros2_jazzy.sh`) |
| `~/test_logs/` | Timestamped runtime logs created automatically by `scripts/start_vision.sh` | No |

> **Virtual Environment Note:** `.venv` on the Raspberry Pi must be created with `python3 -m venv .venv --system-site-packages` so it can import both system ROS 2 packages (`rclpy`) and pip packages (`pymavlink`, `spidev`, `opencv-python`, etc.).

---

## 12. Startup & Deployment Sequence

### Running a Mission (3 Terminals — see [`docs/usefull_commands.md`](usefull_commands.md))

1. **Terminal 1 — Vision Pipeline & Web HUD:**
   Runs `./scripts/start_vision.sh`, which creates `/dev/video10` & `/dev/video11`, starts `erc_vision_node` (`ROS_DOMAIN_ID=27`), and starts `vision_web_streamer.py` on port `5000`.
2. **Terminal 2 — Simulink Mission Supervisor:**
   Sources `~/ros2_ws/install/setup.bash`, exports `ROS_DOMAIN_ID=27`, and runs `ros2 run erc_mission_node erc_mission_node`.
3. **Terminal 3 — Python Flight Mission:**
   Activates `.venv`, exports `ROS_DOMAIN_ID=27`, and runs `python3 main.py`. Once the pilot switches to `GUIDED`, the drone executes one full `IDLE → TAKEOFF → SEARCH → DESCEND → COMPLETE` cycle.

*(If operating without Wi-Fi/SSH, the operator can switch the VTX to the Raspberry Pi composite video channel and run these terminals directly on the Pi desktop using a connected mouse.)*

---

## 13. Configuration Reference

### Key ArduCopter Parameters (Pixhawk 6C)

| Parameter | Value | Meaning |
|-----------|-------|---------|
| `SERIAL2_PROTOCOL` | `2` | MAVLink 2 on TELEM2 (wired to Pi UART `/dev/ttyAMA0`) |
| `SERIAL2_BAUD` | `921` | 921600 baud |
| `PLND_ENABLED` | `1` | Enable precision landing |
| `PLND_TYPE` | `1` | Companion computer (`MAVLink LANDING_TARGET`) |
| `FENCE_ENABLE` | `1` | Mission Planner geofence enabled (uses Holybro M10 GPS) |
