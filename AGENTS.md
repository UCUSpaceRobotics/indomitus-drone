# Repository Guide

## Architecture

- `main.py` is the mission entrypoint and runs `MissionController` plus `VisionBridge` at 50 Hz. MAVLink I/O runs separately in `comm_process_loop` via `multiprocessing.Queue`.
- `src/navigation/` owns flight-state logic; `src/comm/` owns Pixhawk commands and telemetry; `src/ros_bridge/` owns ROS 2 message handling.
- `matlab/erc_vision_node.slx` and `matlab/detectAndEstimate.m` are vision sources. `matlab/src/erc_vision_node/`, `matlab/*_ert_rtw/`, `matlab/slprj/`, `.slxc`, `.tgz`, and MEX files are generated artifacts; regenerate rather than hand-edit them.
- Runtime tuning belongs in `config/mission_params.yaml`. `main.py` resolves this path relative to repository root, not current working directory.

## Environment and Runtime

- Target is Raspberry Pi OS Debian Trixie arm64 with ROS 2 Jazzy. `rclpy` comes from `/opt/ros/jazzy`, not `requirements.txt`; create `.venv` with `--system-site-packages`.
- Start mission with `./scripts/start_mission.sh`. It sources `/opt/ros/jazzy/setup.bash` and `~/ros2_ws/install/setup.bash`, sets `ROS_DOMAIN_ID=42`, activates `.venv`, and uses `sudo -E .venv/bin/python3` so UART access does not lose ROS/Python environment.
- `main.py` and `test_square_flight.py` can command real motors through `/dev/ttyAMA0`; never use them as routine verification. `test_square_flight.py` intentionally has no ROS dependency but still requires Pixhawk hardware.
- Vision deployment lives in external `~/ros2_ws`. Generated archive deployment uses `matlab/build_ros2_model.sh ARCHIVE.tgz ~/ros2_ws`; script consumes/removes archive after extraction, then runs `colcon build --packages-up-to <package>`.

## Focused Verification

- Grid mapper: `python3 -m pytest tests/test_grid_mapper.py -v` (or dependency-free `python3 tests/test_grid_mapper.py`).
- Square controller: `python3 -m unittest tests.test_square_controller -v`.
- LED controller: `python3 -m unittest tests.test_led_indicator -v`; implementation falls back to mock mode when Pi LED libraries/hardware are unavailable.
- Do not run broad `pytest tests/`: `test_publisher.py`, `test_subscriber.py`, and `test_dummy_publisher.py` are long-running ROS 2 bench tools, not unit tests.
- ROS vision bench publisher: source ROS Jazzy first, then run `python3 tests/test_dummy_publisher.py --mode approach` (other modes documented in that file).
- Generated ROS node: from sourced `~/ros2_ws`, run `colcon build --packages-select erc_vision_node`, then `ros2 run erc_vision_node erc_vision_node`.

## Protocol and Safety Invariants

- `/erc/vision_targets` uses `geometry_msgs/msg/Point`: `z=101/102` means marker with camera-frame offsets in `x/y`; `z<0` means probe with world position in `x/y`; `z=0` means no detection.
- Pixhawk local NED uses positive X forward/north, positive Y right/east, and positive Z down; altitude above origin therefore has negative Z.
- Create queued flight commands through `src.comm.mavlink_node.create_command`; comm process drops commands older than 500 ms.
- Keep MAVLink GCS heartbeat at least 1 Hz. Removing or blocking it can trigger ArduCopter GCS failsafe.
- Test camera/body axis or sign changes with props removed before flight. Current landing mapping is camera Y to body forward and camera X to body right.

See `docs/ARCHITECTURE.md` for deeper system context, but prefer current code, scripts, and config when it conflicts with prose.
