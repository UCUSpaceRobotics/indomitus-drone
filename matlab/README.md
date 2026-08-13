# MATLAB & Simulink Vision Model Workspace

This directory contains standalone MATLAB scripts and function definitions for building and testing the Simulink-based ArUco vision model for ERC 2026.

## Files Overview

1. [`generate_markers.m`](file:///c:/Users/Marko/personal/Competitions/ERC_2026/drone/drone_code/indomitus-drone/matlab/generate_markers.m): Generates PNG images for ArUco markers 101 (Takeoff Pad) and 102 (Landing Target) using `DICT_ARUCO_ORIGINAL`.
2. [`calibrate_camera.m`](file:///c:/Users/Marko/personal/Competitions/ERC_2026/drone/drone_code/indomitus-drone/matlab/calibrate_camera.m): Scripted camera calibration using checkerboard images in `./calibration_images/`.
3. [`desktop_prototype_static.m`](file:///c:/Users/Marko/personal/Competitions/ERC_2026/drone/drone_code/indomitus-drone/matlab/desktop_prototype_static.m): Tests ArUco marker detection & pose estimation on static images.
4. [`desktop_prototype_webcam.m`](file:///c:/Users/Marko/personal/Competitions/ERC_2026/drone/drone_code/indomitus-drone/matlab/desktop_prototype_webcam.m): Tests real-time marker detection and tracking on a desktop webcam.
5. [`detectAndEstimate.m`](file:///c:/Users/Marko/personal/Competitions/ERC_2026/drone/drone_code/indomitus-drone/matlab/detectAndEstimate.m): The codegen-compatible MATLAB Function block code used inside the Simulink model (`erc_vision_node.slx`).

For complete detailed documentation, refer to [`docs/simulink_vision_guide.md`](file:///c:/Users/Marko/personal/Competitions/ERC_2026/drone/drone_code/indomitus-drone/docs/simulink_vision_guide.md).
