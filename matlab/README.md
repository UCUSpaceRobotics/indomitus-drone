# MATLAB & Simulink Workspace

This directory contains the Simulink ROS 2 models and supporting MATLAB scripts for the **Indomitus Drone (ERC 2026)**.

## Simulink ROS 2 Models

1. [`erc_vision_node.slx`](erc_vision_node.slx) — Reads 640×480 RGB frames from `/dev/video10` at 15 Hz, detects ArUco markers `101` / `102`, and publishes `[marker_id, tx, ty, tz, rx, ry, rz]` (`std_msgs/Float64MultiArray`) on `/erc/vision_targets`.
2. [`erc_mission_node.slx`](erc_mission_node.slx) — Subscribes to `/erc/drone_telemetry` and `/erc/vision_targets` at 15 Hz, evaluates mission transition & safety guards, and publishes the active state ID (`0..4`, `std_msgs/Int32`) on `/erc/mission_state`.

## Helper Scripts

- [`load_calibration_npz.m`](load_calibration_npz.m) — Loads `../config/camera_calibration.npz` and scales intrinsics/distortion parameters to the runtime capture resolution.
- [`build_ros2_model.sh`](build_ros2_model.sh) — Unpacks a Simulink Coder `.tgz` archive on the Raspberry Pi and builds the ROS 2 C++ package in `~/ros2_ws` with `colcon build`.
- [`detectAndEstimate.m`](detectAndEstimate.m) — Standalone MATLAB reference function for ArUco detection and pose estimation.
- [`calibrate_camera.m`](calibrate_camera.m) — Checkerboard camera calibration script.
- [`generate_markers.m`](generate_markers.m) — Generates printable PNG images for ArUco markers 101 ([`aruco_101.png`](aruco_101.png)) and 102 ([`aruco_102.png`](aruco_102.png)) using `DICT_ARUCO_ORIGINAL`.
- [`desktop_prototype_static.m`](desktop_prototype_static.m) & [`desktop_prototype_webcam.m`](desktop_prototype_webcam.m) — Desktop prototyping scripts for static images and live webcams.

For full system integration details, see [`../docs/ARCHITECTURE.md`](../docs/ARCHITECTURE.md) (legacy step-by-step build notes are archived in [`../docs/archive/`](../docs/archive/)).
