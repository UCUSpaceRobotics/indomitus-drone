# Simulink Vision Node — Deployment & Operations Guide

This document provides a complete reference for running, rebuilding, and maintaining the **Simulink ArUco Vision Node (`erc_vision_node`)** on the Raspberry Pi 5.

---

## 1. System Overview

```
┌────────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 5                                  │
│                                                                        │
│  ┌──────────────────────┐   ROS 2 Topic (/erc/vision_targets)          │
│  │ erc_vision_node      │   geometry_msgs/msg/Point                    │
│  │ (Compiled Simulink)  ├──────────────────────────────┐               │
│  └──────────▲───────────┘                              │               │
│             │ V4L2 YUYV (via /dev/video10)             ▼               │
│  ┌──────────┴───────────┐                    ┌──────────────────┐      │
│  │ GStreamer Bridge     │                    │ Python main.py   │      │
│  │ (rpicam-vid pipe)    │                    │ (VisionBridge)   │      │
│  └──────────▲───────────┘                    └────────┬─────────┘      │
│             │ CSI / libcamera                         │ UART           │
│  ┌──────────┴───────────┐                    ┌────────▼─────────┐      │
│  │ Arducam IMX708       │                    │ Pixhawk 6C       │      │
│  │ (Downward Camera)    │                    │ (Flight Control) │      │
│  └──────────────────────┘                    └──────────────────┘      │
└────────────────────────────────────────────────────────────────────────┘
```

- **Capture Rate:** 15 FPS
- **Resolution:** 640 × 480
- **Pixel Format:** YUYV (passed to `/dev/video10` via GStreamer `YUY2`)
- **ROS 2 Topic:** `/erc/vision_targets`
- **ROS 2 Domain ID:** `42`
- **Message Type:** `geometry_msgs/msg/Point`
  - `x`: Horizontal offset from camera optical center (meters, positive = right)
  - `y`: Vertical offset from camera optical center (meters, positive = forward)
  - `z`: Detected Marker ID (`101.0` = Takeoff Pad, `102.0` = Landing Target, `0.0` = None)

---

## 2. Daily Run Procedure (After Raspberry Pi Reboot)

Whenever the Raspberry Pi restarts, follow these steps across 3 terminals:

### Terminal 1: Start the Camera Loopback Bridge

The Raspberry Pi 5 uses `libcamera`/`rpicam` for CSI cameras. Simulink requires a standard V4L2 device. We use `v4l2loopback` with GStreamer to bridge the two:

```bash
# 1. Load the loopback kernel module (creates /dev/video10 and /dev/video11)
sudo modprobe v4l2loopback video_nr=10,11 card_label="SimulinkCam","WebStreamCam" exclusive_caps=1,1

# 2. Start the streaming pipeline with dual outputs (tee)
rpicam-vid --width 640 --height 480 --framerate 15 \
    --codec yuv420 --output - --timeout 0 | \
    gst-launch-1.0 fdsrc ! \
    rawvideoparse width=640 height=480 format=i420 framerate=15/1 ! \
    videoconvert ! video/x-raw,format=YUY2 ! tee name=t \
    t. ! queue ! v4l2sink device=/dev/video10 \
    t. ! queue ! v4l2sink device=/dev/video11
```
> [!IMPORTANT]
> The format **MUST be `format=YUY2`**. MathWorks' V4L2 capture code specifically queries for `V4L2_PIX_FMT_YUYV` (`YUY2` in GStreamer). If set to `RGB`, the node will fail with `Invalid argument` during resolution validation.

---

### Terminal 2: Run the Vision Node

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 run erc_vision_node erc_vision_node
```

*Note: Warnings about `VIDIOC_QUERYCTRL: Invalid argument` and `'Brightness' is not supported` are expected because `/dev/video10` is a virtual loopback device without physical analog controls.*

---

### Terminal 3: Verify the Topic & Detections

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42

# Check publish frequency (~15 Hz expected)
ros2 topic hz /erc/vision_targets

# Echo live detections
ros2 topic echo /erc/vision_targets
```

Expected output when **Marker 101 or 102** is visible under the camera:
```yaml
x: 0.08639202246686338
y: 0.09024998929632806
z: 101.0
```
When no marker is visible:
```yaml
x: 0.0
y: 0.0
z: 0.0
```

---

## 3. One-Time System Setup & Prerequisites

If setting up a fresh Raspberry Pi 5 from scratch:

```bash
# 1. Install system dependencies
sudo apt update && sudo apt install -y \
    libopencv-dev \
    v4l2loopback-dkms \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    v4l-utils g++ cmake gdb

# 2. Deploy MathWorks Support Libraries (libmwraspiperipheral.so)
# From MATLAB Command Window on your computer:
r = raspi('10.20.18.63', 'marko', 'your_password')
# This deploys libmwraspiperipheral.so to /usr/local/lib and /opt/MATLAB/...
```

---

## 4. Upstream Bug Explanation & Code Patch

### The Bug (Empty Scene Segfault)
When generating C++ code from `[ids, ~, poses] = readArucoMarker(...)`, MATLAB Coder produces an unchecked conversion from raw rotation/translation matrices to `rigidtform3d` structs:

```cpp
// Generated code in erc_vision_node.cpp:
erc_vision_node_B.poseLength = erc_vision_node_B.transVectors.size(1);
for (erc_vision_node_B.rejectionLength = 0; erc_vision_node_B.rejectionLength < 3; ...) {
    // CRASH: Indexing rotMatrices[0] and transVectors[0] when size is 0!
    erc_vision_node_B.camMatrix[...] = erc_vision_node_B.rotMatrices[...];
    erc_vision_node_B.transVectors_data[...] = erc_vision_node_B.transVectors[...];
}
```
When no marker is detected in the frame, `poseLength == 0` and `transVectors` has 0 elements. Accessing `transVectors[0]` results in an instant `SIGSEGV (Segmentation Fault)`.

---

### The Python Patch Script

If you regenerate code and copy a new `erc_vision_node` to the Pi, apply this patch script to `erc_vision_node.cpp` before building:

```bash
python3 -c "
path = '/home/marko/ros2_ws/src/erc_vision_node/src/erc_vision_node.cpp'
with open(path, 'r') as f:
    content = f.read()

target = '  erc_vision_node_B.poseLength = erc_vision_node_B.transVectors.size(1);\n  for (erc_vision_node_B.rejectionLength = 0;'
replacement = '  erc_vision_node_B.poseLength = erc_vision_node_B.transVectors.size(1);\n  if (erc_vision_node_B.poseLength > 0) {\n  for (erc_vision_node_B.rejectionLength = 0;'

if target in content:
    content = content.replace(target, replacement, 1)
    end_target = '        varargout_3_Data[erc_vision_node_B.n - 1] = erc_vision_node_B.rhs_Data[0];\n      }\n    }\n  }\n}'
    end_replacement = '        varargout_3_Data[erc_vision_node_B.n - 1] = erc_vision_node_B.rhs_Data[0];\n      }\n    }\n  }\n  } else {\n    varargout_3_Data.set_size(1, 0);\n  }\n}'
    if end_target in content:
        content = content.replace(end_target, end_replacement, 1)
        with open(path, 'w') as f:
            f.write(content)
        print('SUCCESS: Patch applied!')
    else:
        print('ERROR: End pattern not found')
else:
    print('ERROR: Start pattern not found')
"
```

After running the patch, recompile:
```bash
cd ~/ros2_ws
colcon build --packages-select erc_vision_node
```

---

## 5. How to Fix the Simulink Model Directly

To avoid needing any manual C++ patches in the future, modify the `detectAndEstimate` MATLAB Function block in Simulink:

### Current Code (Has Codegen Bug):
```matlab
% Directly requesting poses causes unchecked translation in C++
[ids, ~, poses] = readArucoMarker(frame, intrinsics, markerSizeMeters);
if isempty(ids)
    return;
end
```

### Improved Clean Code:
Separate detection from pose estimation so that pose calculation only runs when `targetIdx > 0`:

```matlab
function [x_offset, y_offset, marker_id] = detectAndEstimate(R, G, B)
%#codegen

% ── Reconstruct RGB Frame ──
frame = cat(3, R, G, B);

% ── Defaults ──
x_offset  = 0.0;
y_offset  = 0.0;
marker_id = 0.0;

% ── Detect Corner Locations Only (Safe for 0 detections) ──
[ids, locs] = readArucoMarker(frame, "DICT_ARUCO_ORIGINAL");

if isempty(ids)
    return;
end

% ── Priority Selection: 102 (Landing) > 101 (Takeoff) ──
targetIdx = 0;
for i = 1:numel(ids)
    if ids(i) == 102
        targetIdx = i;
        break;
    end
end

if targetIdx == 0
    for i = 1:numel(ids)
        if ids(i) == 101
            targetIdx = i;
            break;
        end
    end
end

if targetIdx == 0
    return;
end

% ── Camera Intrinsics ──
focalLength    = [530.0, 530.0];
principalPoint = [320.0, 240.0];
imageSize      = [480, 640];
intrinsics     = cameraIntrinsics(focalLength, principalPoint, imageSize);

markerSizeMeters = 0.15;
halfSize = markerSizeMeters / 2.0;

% 3D object points of marker corners in marker coordinate frame
worldPoints = [
    -halfSize,  halfSize, 0;
     halfSize,  halfSize, 0;
     halfSize, -halfSize, 0;
    -halfSize, -halfSize, 0
];

% ── Estimate Pose Only for Selected Target Marker ──
targetCorners = squeeze(locs(targetIdx, :, :));
pose = estworldpose(targetCorners, worldPoints, intrinsics);

x_offset  = pose.Translation(1);
y_offset  = pose.Translation(2);
marker_id = double(ids(targetIdx));

end
```

---

## 6. Integration with Python Mission Controller (`main.py`)

The Python state machine connects to the Simulink node via `VisionBridge` (`src/ros_bridge/vision_subscriber.py`):

```bash
# Terminal 1: Camera loopback bridge
# Terminal 2: erc_vision_node

# Terminal 3: Run the flight controller
cd ~/indomitus-drone
source ~/.bashrc && source .venv/bin/activate
export ROS_DOMAIN_ID=42
python3 main.py
```

### Quick Diagnostic Script:
Test receiving messages into Python without starting the whole mission:
```bash
python3 -c "
import rclpy, time
rclpy.init()
from src.ros_bridge.vision_subscriber import VisionBridge

bridge = VisionBridge(topic='/erc/vision_targets', grid_config={
    'origin_x_m': -2.5, 'origin_y_m': -0.5, 'cell_size_m': 1.0,
    'columns': ['A','B','C','D','E','F'], 'rows': [1,2,3,4,5,6]
})

print('Listening for vision targets (5 seconds)...')
start = time.time()
while time.time() - start < 5.0:
    bridge.spin_once()
    t = bridge.get_latest_target()
    if t:
        print(f'Detected: ID={t[\"marker_id\"]} at X={t[\"x_offset_m\"]:+.3f}m, Y={t[\"y_offset_m\"]:+.3f}m, Age={t[\"age_s\"]:.2f}s')
    time.sleep(0.1)

print(f'Total messages received: {bridge.get_message_count()}')
bridge.shutdown()
rclpy.shutdown()
"
```

---

## 7. Future Work & Enhancements

1. **Lens Calibration (Crucial for Flight Precision):**
   - Calibrate the actual Arducam IMX708 using MATLAB's `cameraCalibrator` app with a 9×7 checkerboard.
   - Replace placeholder `focalLength = [530, 530]` and `principalPoint = [320, 240]` with actual calibration parameters.
   - Add radial distortion parameters (`k1, k2, k3`) to `cameraIntrinsics` to eliminate wide-angle lens distortion.

2. **Camera Exposure & Lighting Adjustments:**
   - Modify `rpicam-vid` arguments in the GStreamer pipeline to prevent motion blur and overexposure outdoors:
     `--shutter 2000 --gain 1.0 --awb indoor` (or `outdoor`).

3. **Autostart Systemd Service:**
   - Create `/etc/systemd/system/camera-bridge.service` so the GStreamer loopback pipeline launches automatically on boot.

4. **Probe Detection Extension:**
   - Add HSV color thresholding branch in the MATLAB function to detect competition probes and publish them with negative `z` values (e.g. `z = -1.0`).
