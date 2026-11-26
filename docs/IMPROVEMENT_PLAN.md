# Architecture & Codebase Improvement Plan — Indomitus Drone

> Comprehensive engineering audit of the current **Indomitus Drone** codebase (`chore/drone/working_code_`), documenting existing bugs, architectural debt, future flight-strategy improvements, and a prioritized roadmap for the next development cycle.

---

## Table of Contents

- [Part I — Core Architecture & Code Problems](#part-i--core-architecture--code-problems)
  - [1.1 Startup Deadlock, Split-Brain State Authority & Safety Race Conditions](#11-startup-deadlock-split-brain-state-authority--safety-race-conditions)
  - [1.2 Configuration Sprawl, Magic Numbers & YAML-vs-Simulink Contradictions](#12-configuration-sprawl-magic-numbers--yaml-vs-simulink-contradictions)
  - [1.3 Simulink Recompilation Bottleneck (No Runtime Parameterization)](#13-simulink-recompilation-bottleneck-no-runtime-parameterization)
  - [1.4 MATLAB-Dependent Camera Calibration & Aspect-Ratio Scaling Skew](#14-matlab-dependent-camera-calibration--aspect-ratio-scaling-skew)
  - [1.5 ROS 2 Message Type Hijacking, Multi-Terminal Startup & `sudo` Rules](#15-ros-2-message-type-hijacking-multi-terminal-startup--sudo-rules)
  - [1.6 Blocking Calls & 1-Second Freezes in the MAVLink Layer](#16-blocking-calls--1-second-freezes-in-the-mavlink-layer)
  - [1.7 Unstructured `print()` Output, Missing Disk/Blackbox Logs & Web Streamer Overhead](#17-unstructured-print-output-missing-diskblackbox-logs--web-streamer-overhead)
  - [1.8 Outdated & Broken Test Suite](#18-outdated--broken-test-suite)
- [Part II — Future Flight Strategy & Algorithmic Enhancements](#part-ii--future-flight-strategy--algorithmic-enhancements)
  - [2.1 Option A — Refined Search Pattern: Closed-Loop `LOCAL_POSITION_NED` Bounded to `search_radius_m`](#21-option-a--refined-search-pattern-closed-loop-local_position_ned-bounded-to-search_radius_m)
  - [2.2 Option B — Searchless Static Arena Scan: Uncropped High-Resolution Capture Below 4 m](#22-option-b--searchless-static-arena-scan-uncropped-high-resolution-capture-below-4-m)
  - [2.3 Precision Landing Stability: IMU Roll/Pitch Tilt Compensation](#23-precision-landing-stability-imu-rollpitch-tilt-compensation)
- [Part III — Prioritized Implementation Roadmap](#part-iii--prioritized-implementation-roadmap)

---

## Part I — Core Architecture & Code Problems

Each item below describes an existing bug or architectural flaw in the codebase, immediately followed by its concrete fix.

---

### 1.1 Startup Deadlock, Split-Brain State Authority & Safety Race Conditions

#### Problem
1. **Startup Deadlock in `MissionController` (introduced in commit `dd8065c`):**
   - In [`src/navigation/state_machine.py`](../src/navigation/state_machine.py), `__init__` sets `self.state = FlightState.IDLE` (line 89) and `self._mission_innitialized = False` (line 149, also misspelled).
   - On every tick of `update()` (lines 176–192):
     1. `_check_external_override_or_failsafe()` is called first, and its very first guard (line 297) executes:
        ```python
        if not self.telem or self.state in (FlightState.IDLE, FlightState.COMPLETE):
            return False
        ```
        Because `self.state` starts in `IDLE`, lines 302–305 (`if self._mission_innitialized == False: ... self._mission_innitialized = True`) are **100% unreachable dead code**.
     2. Execution returns to `update()` line 179 (`if self._mission_innitialized == False: ... return`), exiting **before** `_sync_simulink_state()` (line 192) can ever run. As a result, `main.py` stays permanently locked in `IDLE`.
   - Furthermore, gating startup on `mode == "GUIDED"` conflicts with `_update_takeoff()` Phase 0, which immediately commands `LOITER` to arm with optical flow.
2. **Split-Brain Geofence Race Condition Between Python and Simulink:**
   - Both Python (`_check_geofence()` at 50 Hz) and Simulink (`erc_mission_node` at 15 Hz) enforce the `8.0 m` geofence independently.
   - If Python triggers `_check_geofence()` first (`BRAKE` → `LAND`, setting `self.state = FlightState.DESCEND`), on the very next 50 Hz tick — before the slower 15 Hz Simulink node sees `origin_dist > 8.0` — `_sync_simulink_state()` sees Simulink still publishing `2` (`SEARCH`), misinterprets it as a lost-marker descent abort (lines 266–273), and **commands `GUIDED` + climbs back up during an emergency geofence breach**.
3. **Missing Simulink Heartbeat Failsafe:**
   - When `/erc/mission_state` goes stale (`> 0.5 s` in `get_simulink_state()`), `_sync_simulink_state()` logs `"Bad connection with Simulink: None"` and returns without taking action. If `erc_mission_node` crashes mid-flight, the drone remains unsupervised in its last active state.

#### Proposed Fix
- **Remove the broken `_mission_innitialized` gate** so `IDLE` can synchronize with Simulink cleanly, and only arm the manual-override mode check *after* `_update_takeoff()` confirms `GUIDED` mode on Pixhawk (fixing the mode-transition race `dd8065c` originally tried to solve).
- **Enforce strict single-owner authority with a latched Python safety override:**
  - *Simulink (`erc_mission_node`)* owns all normal mission transitions (`IDLE → TAKEOFF → SEARCH → DESCEND → COMPLETE`).
  - *Python (`MissionController`)* owns an **irreversible `_safety_abort_latched` flag** triggered by geofence breach, RC manual takeover, or Simulink heartbeat loss (`> 1.5 s` while airborne). Once latched, Python forces `LAND` (or yields to RC pilot mode) and ignores all subsequent Simulink state messages for the remainder of the run.

---

### 1.2 Configuration Sprawl, Magic Numbers & YAML-vs-Simulink Contradictions

#### Problem
1. **Hardcoded constants scattered across modules instead of [`config/mission_params.yaml`](../config/mission_params.yaml):**
   - [`src/navigation/state_machine.py`](../src/navigation/state_machine.py): `MAX_DISTANCE_FROM_ORIGIN_M = 8.0`, `SEARCH_CONFIRM_TICKS = 5` (unused), `_search_dwell_s = 3.0`, `_search_travel_s = 4.0`, `LANDING_TARGET_SEND_INTERVAL_S = 1/15` (stale comment says `200ms`), `LANDING_TARGET_MIN_ALT_M = 0.25` (comment says `0.5m`), `origin_dist > 5.0` (while the log prints `> 1.0m`), and takeoff step delays (`2.0 s`, `5.0 s`, `1.0 s`).
   - [`src/comm/mavlink_node.py`](../src/comm/mavlink_node.py) & [`src/comm/mavlink_client.py`](../src/comm/mavlink_client.py): `GCS_HEARTBEAT_INTERVAL_SEC = 1.0`, `TELEMETRY_PUBLISH_RATE_HZ = 10.0`, `COMMAND_MAX_AGE_SEC = 0.5`, `source_system = 254`, stream rates (`10/20 Hz`), timeouts (`15.0 s`, `3.0 s`).
   - [`src/ros_bridge/vision_subscriber.py`](../src/ros_bridge/vision_subscriber.py): Hardcodes `MARKER_ID_ORIGIN = 101`, `MARKER_ID_LANDING = 102`, `DEFAULT_DETECTION_TIMEOUT_S = 0.3`, and Simulink state timeout `0.5 s` instead of reading `markers` and `timeouts` from config.
   - [`main.py`](../main.py), [`scripts/start_vision.sh`](../scripts/start_vision.sh), & [`scripts/vision_web_streamer.py`](../scripts/vision_web_streamer.py): Hardcode `50 Hz` loop rate, `ROS_DOMAIN_ID=27`, `640x480 @ 15 FPS`, shutter `16000`, `/dev/video10` & `/dev/video11`, HTTP port `5000`, HUD scale `350`, a hardcoded IP (`10.20.18.63`), and a mismatched `/dev/video10` error message when opening `/dev/video11`.
2. **Active contradiction between YAML and compiled Simulink code:**
   - `mission_params.yaml` sets `flight.takeoff_altitude_m: 2.2` (Python commands a climb to **2.2 m**), **while `erc_mission_node.slx` hardcodes `TARGET_ALT_M = 1.5` (`85% = 1.275 m`)** — causing Simulink to switch from `TAKEOFF` to `SEARCH` prematurely at `1.275 m` while the drone is still climbing.
3. **Dead YAML keys:**
   - Nearly half of `mission_params.yaml` (`search_altitude_m`, `landing_approach_altitude_m`, `max_horizontal_speed_m_s`, `max_vertical_speed_m_s`, `mission.*`, `timeouts.*`, `yaw_alignment.*`) is **never read anywhere at runtime**, giving a false impression that editing YAML changes flight behavior.

#### Proposed Fix
- Create a single typed, validated configuration loader (`src/config.py` using Python `dataclasses` or `Pydantic`) backed by `config/mission_params.yaml`.
- Move **all** tunable constants (loop rates, MAVLink rates/timeouts, landing target rate & altitude cutoff, search dwell/travel, camera resolution/FPS/shutter, geofence radius) into `config/mission_params.yaml` and forbid module-level magic numbers.
- Delete or wire up every dead YAML key, and have Python set `os.environ.setdefault("ROS_DOMAIN_ID", str(cfg.ros2.domain_id))` before importing `rclpy`.

---

### 1.3 Simulink Recompilation Bottleneck (No Runtime Parameterization)

#### Problem
Both [`matlab/erc_vision_node.slx`](../matlab/erc_vision_node.slx) and [`matlab/erc_mission_node.slx`](../matlab/erc_mission_node.slx) compile every threshold (`TARGET_ALT_M`, `TAKEOFF_ALT_COEFF`, `TOUCHDOWN_ALT_M`, `REQUIRED_CONFIRM_TICKS`, `MAX_ORIGIN_DRIFT_IDLE_M`, `GEOFENCE_MAX_DIST_M`, `TIMEOUT_*`), marker size/IDs (`0.15 m`, `101`, `102`), and camera intrinsics directly into C++ binaries on a laptop. Testing a single parameter change on the field requires MATLAB + Simulink Coder + SCP transfer + `colcon build` on the Raspberry Pi.

#### Proposed Fix
- Replace hardcoded constants and workspace variables inside both `.slx` models with **Simulink ROS 2 `Get Parameter` (`ros2lib/Get Parameter`) blocks** (declaring scalar and array parameters with sensible defaults).
- Store Simulink node parameters in a standard ROS 2 YAML parameter file (either unified with `config/mission_params.yaml` or auto-exported from it before launch) and pass it at startup with:
  ```bash
  ros2 run erc_mission_node erc_mission_node --ros-args --params-file config/ros2_params.yaml
  ros2 run erc_vision_node erc_vision_node --ros-args --params-file config/ros2_params.yaml
  ```
- Modifying flight altitudes, debounce frame counts, timeouts, marker sizes, or camera calibration on the Raspberry Pi will then require **zero Simulink recompilations** — just edit YAML and restart the nodes.

---

### 1.4 MATLAB-Dependent Camera Calibration & Aspect-Ratio Scaling Skew

#### Problem
1. **MATLAB-bound calibration pipeline:** Calibration currently relies on [`matlab/calibrate_camera.m`](../matlab/calibrate_camera.m) (hardcoded `25 mm` checkerboard, outputs `.mat`) and [`matlab/load_calibration_npz.m`](../matlab/load_calibration_npz.m) (which calls Python `numpy` inside MATLAB at compile time to bake `config/camera_calibration.npz` into `erc_vision_node`). Recalibrating on the field without MATLAB is impossible.
2. **33% focal-length scaling skew (`16:9` → `4:3` bug):** `config/camera_calibration.npz` was calibrated at `1280 × 720` (16:9), whereas runtime captures `640 × 480` (4:3). In `load_calibration_npz.m` (lines 31–37), `fx` is multiplied by `640 / 1280 = 0.50` while `fy` is multiplied by `480 / 720 = 0.667`. Because `rpicam-vid` changes aspect ratio by **cropping** horizontally rather than stretching pixels, scaling `fx` and `fy` by different ratios introduces a **33% non-square pixel distortion** (`fx = 436.8` vs `fy = 574.9`), skewing 3D `tvec` horizontal and depth estimates.

#### Proposed Fix
- Replace `calibrate_camera.m` and `load_calibration_npz.m` with a standalone **Python + OpenCV calibration CLI script** (`scripts/calibrate_camera.py`) that runs directly on the Raspberry Pi:
  - Captures checkerboard (or ChArUco) frames directly from `/dev/video10` **at the exact runtime sensor mode and resolution** (eliminating any aspect-ratio scaling math), or processes a saved image folder.
  - Computes `focal_length [fx, fy]`, `principal_point [cx, cy]`, `radial_distortion [k1, k2, k3]`, `tangential_distortion [p1, p2]`, `image_size [rows, cols]`, and RMS reprojection error.
  - Writes those arrays directly into the ROS 2 YAML config file so `erc_vision_node` loads them dynamically on startup via **ROS 2 Parameters** (`ros2lib/Get Parameter`).

---

### 1.5 ROS 2 Message Type Hijacking, Multi-Terminal Startup & `sudo` Rules

#### Problem
1. **Hijacking `geometry_msgs/Pose` & `Float64MultiArray` as untyped structs:**
   - `VisionBridge.publish_telemetry()` ([`src/ros_bridge/vision_subscriber.py`](../src/ros_bridge/vision_subscriber.py), lines 180–204) packs 7 unrelated scalars/booleans (`alt`, `is_armed`, `ekf_healthy`, `origin_dist`, `connected`, `manual_override`, `waypoints_exhausted`) into `Pose.position` and `Pose.orientation` — generating mathematically invalid quaternions and unreadable `ros2 topic echo` output.
   - `/erc/vision_targets` uses an unlabeled 7-float array (`[marker_id, tx, ty, tz, rx, ry, rz]`) with **no timestamp header** and a **1-target-per-frame limit**: if marker `101`, marker `102`, and competition probes appear in the same camera frame, `erc_vision_node` drops everything except a single detection.
2. **Manual 3-terminal launch & `sudo` FastDDS hazards:**
   - Operators must manually open 3 terminals, source environments, and export `ROS_DOMAIN_ID=27` in each.
   - While kernel setup (`sudo modprobe v4l2loopback`, `apt`) legitimately requires `sudo`, running any ROS 2 node process under `sudo` (as legacy `start_mission.sh` did) creates root-owned FastDDS shared-memory lock files in `/dev/shm` that can block non-root ROS 2 nodes (`erc_vision_node`, `erc_mission_node`) from communicating.

#### Proposed Fix
- Create a lightweight custom ROS 2 interface package (`erc_msgs`, supported in Simulink via `ros2genmsg`):
  - `VisionDetection.msg` & `VisionDetectionArray.msg` (with `std_msgs/Header`, allowing `erc_vision_node` to publish **all** visible ArUco markers + probes in a single frame).
  - `DroneTelemetry.msg` (explicit named fields: `float64 altitude_m`, `bool is_armed`, `bool ekf_healthy`, etc.).
  - `MissionState.msg` (explicit state `uint8` + state name/reason).
- Consolidate the 3-terminal workflow into **one unified launch script / `ros2 launch` file** that starts the `v4l2loopback` pipeline (keeping passwordless `sudo` strictly scoped to `/sbin/modprobe` & `/sbin/rmmod` via `/etc/sudoers.d/v4l2loopback`) and launches all ROS 2 nodes (`erc_vision_node`, `erc_mission_node`, `vision_web_streamer`, `main.py`) as the **same non-root Linux user** (with `dialout`, `spi`, `gpio`, `video` group permissions).

---

### 1.6 Blocking Calls & 1-Second Freezes in the MAVLink Layer

#### Problem
1. **1-second full communication freeze on rejected commands:**
   - In [`src/comm/mavlink_node.py`](../src/comm/mavlink_node.py) (lines 120–133), `dispatch_command()` raises a `ValueError` if a command arrives while disarmed or before `telem["mode"]` reflects `"GUIDED"`. The outer `except Exception:` block (lines 115–117) catches it and executes **`time.sleep(1)`** — freezing **all** UART telemetry reads, command processing, and the 1 Hz GCS heartbeat for a full second.
2. **Blocking `arm()` loop drops telemetry packets:**
   - In [`src/comm/mavlink_client.py`](../src/comm/mavlink_client.py) (lines 289–323), `PixhawkClient.arm()` blocks for up to `3.0 s` reading `recv_match()` in a private loop that only checks `STATUSTEXT` and `COMMAND_ACK`, **silently discarding every `HEARTBEAT`, `LOCAL_POSITION_NED`, `ATTITUDE`, and `EKF_STATUS_REPORT` packet** arriving during arming.
3. **Missing UART link-loss detection & unverified commands:**
   - `telemetry["connected"]` is set to `True` once at boot (`wait_for_heartbeat()`) and **never set back to `False`** if the Pixhawk reboots or the UART wire disconnects mid-flight (`last_heartbeat_time` age is never checked).
   - `set_mode()`, `takeoff()`, and `land()` do not track `COMMAND_ACK` rejections from ArduPilot.

#### Proposed Fix
- Replace `raise ValueError` + `time.sleep(1)` in `mavlink_node.py` with structured warning logs and clean command rejection without sleeping the comm loop.
- Make `PixhawkClient` **100% non-blocking**: route *all* UART reads through a single `get_telemetry_tick()` parser (which also records `COMMAND_ACK` results into a dictionary), so `arm()`, `set_mode()`, `takeoff()`, and `land()` never steal or drop incoming telemetry packets.
- Continuously update `telemetry["connected"] = (time.time() - last_heartbeat_time) < heartbeat_timeout_s` (e.g., `2.0 s`) on every tick so UART link loss is detected immediately.

---

### 1.7 Unstructured `print()` Output, Missing Disk/Blackbox Logs & Web Streamer Overhead

#### Problem
1. **Unsaved, unstructured `print()` output & no flight blackbox:**
   - `main.py`, `state_machine.py`, `mavlink_client.py`, and `led_indicator.py` use ad-hoc `print()` calls without timestamps or consistent formatting, and **`main.py` output is never saved to disk**.
   - There is no machine-readable blackbox recording (`CSV` / `JSONL` / `rosbag2`) of synchronized pose, mode, Simulink state, and vision `tvec`/`rvec` — making post-flight root-cause analysis of landing offsets or mode overrides guesswork.
2. **Wasted CPU in `vision_web_streamer.py`:**
   - [`scripts/vision_web_streamer.py`](../scripts/vision_web_streamer.py) reads `/dev/video11`, draws OpenCV HUD overlays, and compresses JPEG frames ~30 times per second even when **0 browser clients are connected**, wasting Raspberry Pi CPU during autonomous flight. It also uses a hardcoded `350 px/m` visual arrow instead of projecting the 3D `tvec` through the camera intrinsic matrix `K`.

#### Proposed Fix
- Implement a central logger (`src/utils/logger.py`) using Python's `logging` module with **dual output**:
  1. **Real-time terminal output (`StreamHandler`):** Keeps live operator visibility in SSH/VTX terminals, upgraded to a clean, uniform format:
     - *Event lines:* `[HH:MM:SS.mmm] [INFO ] [STATE  ] IDLE -> TAKEOFF (Simulink=1)`
     - *Periodic 1–2 Hz status line:* `[HH:MM:SS.mmm] [TELEM] State=SEARCH(2) | Mode=GUIDED | Armed=Y | EKF=OK | Alt=2.18m | Pos=(+1.02,-0.05)m | WP=4/21 | Mkr=102(+0.12,-0.04)m | Batt=15.9V`
  2. **Persistent disk log (`FileHandler`):** Automatically saves the exact terminal transcript to `~/test_logs/<run_timestamp>/mission.log`, alongside a high-rate `flight_blackbox.csv` (10–20 Hz) logging raw telemetry + vision vectors for post-flight plotting.
- Track active HTTP client count in `vision_web_streamer.py` so `camera_thread` idles when `active_clients == 0`, and project `tvec` onto the HUD image using actual camera intrinsics `(fx, fy, cx, cy)`.

---

### 1.8 Outdated & Broken Test Suite

#### Problem
Nearly all files in [`tests/`](../tests/) are out of sync with the current codebase:
- [`tests/test_simulink_state_machine.py`](../tests/test_simulink_state_machine.py) currently **fails** (`test_simulink_command_touchdown_to_complete`) and bypasses `IDLE` initialization by manually mutating `self.mission.state`.
- [`tests/test_dummy_publisher.py`](../tests/test_dummy_publisher.py) still publishes the legacy `geometry_msgs/Point` message instead of the 7-element `Float64MultiArray` (or future `erc_msgs`).
- [`tests/test_vision.py`](../tests/test_vision.py) contains a raw shell one-liner (`python3 -c "..."`) inside a `.py` file (invalid Python syntax) referencing removed dict keys (`x_offset_m`).

#### Proposed Fix
- Rewrite unit tests around the updated config, non-blocking `PixhawkClient` (using a mock MAVLink connection), and full `IDLE → TAKEOFF → SEARCH → DESCEND → COMPLETE` mission lifecycle (including safety latches and timeouts).
- Update `test_dummy_publisher.py` to match the real vision/supervisor ROS 2 topics so bench testing works without cameras, and add a lightweight GitHub Actions CI workflow running `pytest` and `ruff` on every PR.

---

## Part II — Future Flight Strategy & Algorithmic Enhancements

Unlike Part I (bugs and architectural debt), the items below are **design & flight-strategy improvements** worth evaluating and benchmarking on the next drone iteration.

---

### 2.1 Option A — Refined Search Pattern: Closed-Loop `LOCAL_POSITION_NED` Bounded to `search_radius_m`

#### Current Limitation
`_update_search()` currently commands 48 relative `1 m` moves in **`MAV_FRAME_BODY_OFFSET_NED`** (*"move 1 m relative to current body position and heading"*) on a blind **open-loop timer** (`4.0 s` travel + `3.0 s` dwell = `7.0 s` × 48 = `336 s`, which exceeds the `320 s` search timeout) over a `7 × 7 m` square:
- **Compounding step & heading errors:** Because each step is relative to wherever the previous step happened to stop, any controller tracking shortfall (e.g., reaching `0.85 m` before the timer fires) or small yaw drift (`3–5°`) **accumulates across all 48 moves**, shifting and rotating the search pattern.
- **Out-of-bounds corners:** A `7 × 7 m` square reaches corners at $\sqrt{3^2 + 3^2} = 4.24\text{ m}$ from the pad — `1.24 m` outside the competition's `3.0 m` landing-disc radius and closer to the cage net.

#### Proposed Solution
1. **Command waypoints in `MAV_FRAME_LOCAL_NED` relative to recorded pad origin `(x0, y0)`:**
   - Pixhawk's EKF3 maintains a fixed local coordinate system (`pos_x_m` = North/initial-forward, `pos_y_m` = East/initial-right, `pos_z_m` = Down). Recording `(x0, y0)` right before liftoff and sending absolute targets `(x0 + wx, y0 + wy, -search_alt)` via `send_local_ned_position_target()` eliminates cumulative controller shortfall and yaw-rotation drift across steps.
   - *Important physical caveat:* Because indoor EKF3 integrates optical flow (`MTF-01`) + IMU without external GPS/VIO, Pixhawk's underlying `(pos_x_m, pos_y_m)` estimate still experiences slow optical-flow drift over time. However, using `LOCAL_POSITION_NED` removes the much larger **48-step compounding control error** (and if takeoff marker `101` is re-observed during flight, its known `(0, 0)` position can even be used to recalibrate EKF drift).
2. **Closed-loop waypoint arrival detection (with timeout fallback):**
   - Continuously compute horizontal distance to the active waypoint: $\text{dist} = \sqrt{(x - x_{\text{wp}})^2 + (y - y_{\text{wp}})^2}$.
   - As soon as `dist < arrival_radius_m` (e.g., `0.20 m`) and horizontal speed settles, run a brief stabilization scan dwell (`1.0–1.5 s` for blur-free frames) and immediately advance to the next waypoint — keeping a `max_travel_time_s` (`5.0 s`) fallback so the drone never stalls if steady-state offset is `0.25 m`.
3. **Circular `search_radius_m` bounding:**
   - Filter and order candidate grid/ring coordinates so every waypoint satisfies $wx^2 + wy^2 \le R_{\text{search}}^2$ (`3.0 m` minus half the camera ground footprint), eliminating wasted diagonal excursions to `4.24 m` and cutting waypoint count roughly in half.

---

### 2.2 Option B — Searchless Static Arena Scan: Uncropped High-Resolution Capture Below 4 m

#### Current Limitation
Although the **Arducam IMX708** lens has a **120° wide-angle FOV** (`102° × 67°` active sensor area at `4608 × 2592` / `2304 × 1296`), the current pipeline runs `rpicam-vid --width 640 --height 480` without selecting a full-sensor `libcamera` mode. This forces a **center-cropped sensor readout** with a tiny ground footprint (~`2.0 × 1.8 m` at `2.2 m` altitude), which is the only reason a long 48-point search pattern was needed in the first place.

#### Proposed Solution
Eliminate the multi-waypoint search pattern altogether by trading frame rate for **full-sensor field of view + higher resolution** from a **static high hover**:
1. **Unlock the full 120° sensor in `rpicam-vid`:**
   - Pass `--mode 2304:1296` (full-sensor 2×2 binned readout, zero sensor crop) or `--mode 4608:2592` and output **`1920 × 1080` or `2304 × 1296`** frames (recalibrated at that exact mode with `scripts/calibrate_camera.py`).
2. **Climb to a static observation hover at `3.2–3.5 m` (safely below the `4.0 m` cage ceiling):**
   - With the uncropped `102° × 67°` FOV at `3.4 m` altitude, a single stationary frame covers **`~8.4 m × 4.5 m`** on the floor! (If the narrower axis doesn't cover the full `6 m` diameter at once, a single in-place `90°` yaw turn covers 100% of the `3.0 m` radius arena without any horizontal translation.)
   - At `2304 × 1296`, a `15 × 15 cm` ArUco marker at `3.5 m` spans **`~55–65 pixels`**, well above `DICT_ARUCO_ORIGINAL`'s ~`20–25 px` minimum decoding size — especially because hovering stationary eliminates motion blur.
3. **Two-stage static-detect → direct-approach:**
   - Even if Simulink processes `2304 × 1296` frames slower (`2–5 Hz` instead of `15 Hz`), the drone is hovering still in `SEARCH` — spending `2–4 seconds` to lock onto marker `102` (and all 3 probes simultaneously!) is vastly faster and safer than a 3-minute sweep.
   - Once `(tx, ty)` is acquired from hover, command **one closed-loop `LOCAL_POSITION_NED` move** directly over `(x + tx, y + ty)` while descending to `1.5 m`, then engage high-rate (`15 Hz` ROI/downscaled or precision-landing) `DESCEND` onto the disc.

---

### 2.3 Precision Landing Stability: IMU Roll/Pitch Tilt Compensation

#### Current Limitation
`erc_vision_node` computes `tvec` in the **camera optical frame** and maps `[y_cam, -x_cam, z_cam]` directly to `MAV_FRAME_BODY_FRD` assuming the camera always points straight down toward gravity. Whenever the drone pitches or rolls by angle $\theta$ (e.g., `5–8°` while braking or correcting wind/turbulence at `2.2 m` altitude), the rigid camera tilts with the airframe, shifting the apparent marker offset by $h \cdot \tan(\theta) \approx 20\text{–}30\text{ cm}$ and causing `LANDING_TARGET` oscillation during descent.

#### Proposed Solution
- Rotate the camera-frame vector `tvec` by the drone's instantaneous `roll_rad` and `pitch_rad` from Pixhawk `ATTITUDE` telemetry (either in Python before calling `send_landing_target()` or by passing roll/pitch to Simulink) to compute the true gravity-leveled horizontal offset to the marker, or send `LANDING_TARGET` in `MAV_FRAME_LOCAL_NED` — yielding smooth, oscillation-free descents without needing aggressive rate throttling.

---

## Part III — Prioritized Implementation Roadmap

Execute these changes in order on dedicated feature branches after merging the current baseline into `main`:

1. **Fix `MissionController` Startup Deadlock & Add Irreversible Safety Latch *(Critical — Flight Blocker)*:**
   Remove the unreachable `_mission_innitialized` gate in `state_machine.py`, fix the `TAKEOFF` mode-transition check, add a Simulink heartbeat-loss timeout, and latch Python emergency `LAND`/manual overrides so Simulink can never revert an emergency descent back to `GUIDED`.
2. **Refactor `PixhawkClient` & `mavlink_node.py` to Be 100% Non-Blocking *(Critical — Flight Safety)*:**
   Eliminate `time.sleep(1)` on rejected commands, remove the blocking `recv_match` loop inside `arm()` so telemetry packets are never dropped, add heartbeat-age `connected` monitoring, and track `COMMAND_ACK` responses asynchronously.
3. **Unify All Constants into `config/mission_params.yaml` via a Typed Config Loader *(High)*:**
   Build `src/config.py`, replace all hardcoded magic numbers across Python and shell scripts, resolve the `2.2 m` vs `1.5 m` takeoff altitude mismatch, and clean up dead YAML fields.
4. **Add ROS 2 Parameters (`ros2lib/Get Parameter`) to Both Simulink Models *(High)*:**
   Parameterize `erc_vision_node.slx` and `erc_mission_node.slx` so altitudes, timeouts, debounce ticks, marker sizes, and camera intrinsics load dynamically at startup via `--ros-args --params-file` with zero recompilation.
5. **Build Standalone Python Camera Calibration CLI (`scripts/calibrate_camera.py`) *(High)*:**
   Calibrate directly on the Raspberry Pi at the exact runtime `rpicam-vid` sensor mode and resolution (fixing the `16:9 → 4:3` `fx/fy` scaling bug) and export intrinsics straight into the ROS 2 parameter YAML.
6. **Implement Dual Structured Logging + Flight Blackbox & Idle-Aware Web Streamer *(Medium)*:**
   Replace raw `print()` with structured real-time terminal output (`StreamHandler` + 1 Hz status summary) mirrored automatically to `~/test_logs/<timestamp>/mission.log` and `flight_blackbox.csv`, and pause JPEG encoding in `vision_web_streamer.py` when no browser is connected.
7. **Introduce `erc_msgs` Custom ROS 2 Messages & Single-Command Launch *(Medium)*:**
   Replace hijacked `Pose` / `Float64MultiArray` payloads with named ROS 2 messages supporting multi-marker + multi-probe detections per frame, and wrap the 3-terminal startup into one non-root `ros2 launch` workflow.
8. **Rebuild Unit Test Suite & CI (`pytest` + `ruff`) *(Medium)*:**
   Update `test_simulink_state_machine.py` and `test_dummy_publisher.py` to match the real ROS 2 contracts and run automated checks on every pull request.
9. **Benchmark Next-Gen Search & Precision Landing Strategies *(Future R&D)*:**
   Add roll/pitch tilt compensation to `LANDING_TARGET`, then flight-test **Option B (Uncropped high-res static hover scan below 4 m)** against **Option A (Closed-loop `LOCAL_POSITION_NED` bounded search)** to pick the fastest, most reliable competition strategy.
