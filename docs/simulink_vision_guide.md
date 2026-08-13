# Simulink Vision Model — Implementation Guide

> **Purpose:** Complete technical reference and step-by-step guide for building, deploying, and integrating the Simulink-based ArUco vision pipeline for the ERC 2026 Droning Sub-Task.
>
> **Target audience:** Developers and AI assistants working on this project.
>
> **MATLAB version required:** R2024b or newer (for `readArucoMarker` codegen support).

---

# PART 1 — Technical Model Specification

---

## 1. Model Purpose

The Simulink model runs on the Raspberry Pi 5 as a standalone ROS 2 node. It performs one function: **capture camera frames, detect ArUco markers, estimate their 3D pose, and publish the result to a ROS 2 topic.**

The Python state machine (`main.py`) subscribes to this topic and uses the data to fly the drone.

---

## 2. System Context

```
┌──────────────────────────────────────────────────────────────────────┐
│                        RASPBERRY PI 5                                │
│                                                                      │
│  ┌─────────────────────┐     ROS 2 Topic      ┌──────────────────┐  │
│  │ Simulink Vision Node│  /erc/vision_targets  │ Python main.py   │  │
│  │ (C++ compiled)      │─────────────────────►│ (State Machine)  │  │
│  │                     │  geometry_msgs/Point   │                  │  │
│  └─────────┬───────────┘                       └────────┬─────────┘  │
│            │ V4L2                                        │ UART       │
│  ┌─────────▼───────────┐                       ┌────────▼─────────┐  │
│  │ Arducam IMX708      │                       │ Pixhawk 6C       │  │
│  │ (downward-facing)   │                       │ (flight control) │  │
│  └─────────────────────┘                       └──────────────────┘  │
└──────────────────────────────────────────────────────────────────────┘
```

**Separation of concerns:**
- The Simulink node does NOT send commands to the Pixhawk.
- The Python code does NOT touch the camera.
- They communicate only through the ROS 2 topic.

---

## 3. Vision Pipeline — Detailed Data Flow

```
Stage 1              Stage 2              Stage 3              Stage 4
CAPTURE              DETECT               ESTIMATE POSE        PUBLISH
─────────────────    ─────────────────    ─────────────────    ─────────────
V4L2 Video      ──►  readArucoMarker ──►  Extract tvec    ──►  ROS 2
Capture block        (MATLAB Function     from rigidtform3d    Publish block
                      block)              (MATLAB Function
                                           block)

Signal:              Signal:              Signal:              Signal:
uint8 [480×640×3]    ids:  int32[N×1]     x_off: double        Point msg:
(RGB frame)          locs: double[N×4×2]  y_off: double          .x = x_off
                     poses: rigidtform3d  mk_id: double          .y = y_off
                                                                  .z = mk_id
```

---

## 4. Stage Details

### Stage 1: Camera Capture

**Block:** `V4L2 Video Capture` (Simulink Support Package for Raspberry Pi Hardware)

> [!WARNING]
> **Raspberry Pi 5 issue:** The Pi 5 uses a new Media Controller API that breaks direct CSI camera access through V4L2. The standard V4L2 block cannot directly access the Arducam IMX708 on Pi 5. **You must use a GStreamer virtual device workaround** (see Section 7).

**Block parameters:**

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Device | `/dev/video10` | Virtual device created by GStreamer loopback (see Section 7) |
| Image size | `640 x 480` | Sufficient for ArUco at 2m altitude. Smaller = faster processing |
| Image color space | `RGB` | Required for color-based probe detection later |
| Sample time | `1/15` | 15 FPS. Balance between detection rate and CPU load |
| Data type | `uint8` | Standard image format |

**Output signal:** `uint8` matrix of size `[480 × 640 × 3]`

---

### Stage 2: ArUco Detection + Pose Estimation

**Block:** `MATLAB Function` (Simulink built-in)

**What it does:** Takes the raw RGB frame, runs ArUco detection, and estimates 3D pose for priority marker.

**Code generation:** `readArucoMarker` supports C/C++ code generation from **R2024b onward**. The generated code links to OpenCV 4.7.0 libraries. These must be present on the Raspberry Pi at compile time.

**MATLAB Function block code:**

```matlab
function [x_offset, y_offset, marker_id] = detectAndEstimate(frame)
%#codegen
%
% Detects ArUco markers in the camera frame and estimates the position
% of the highest-priority marker relative to the camera center.
%
% Inputs:
%   frame - uint8 [480×640×3] RGB image from V4L2 Video Capture
%
% Outputs:
%   x_offset  - double. Horizontal offset in meters (positive = right of center)
%   y_offset  - double. Vertical offset in meters (positive = below center)
%   marker_id - double. Detected marker ID (101, 102) or 0 if none found

    % ── Camera intrinsics (from calibration) ──
    % These are compile-time constants. Replace with YOUR calibration values.
    focalLength    = [530.0, 530.0];     % [fx, fy] in pixels
    principalPoint = [320.0, 240.0];     % [cx, cy] in pixels — center of 640×480
    imageSize      = [480, 640];         % [rows, cols]

    intrinsics = cameraIntrinsics(focalLength, principalPoint, imageSize);

    % ── Marker parameters ──
    markerSizeMeters = 0.15;  % 15 cm physical marker size

    % ── Default output: no detection ──
    x_offset  = 0.0;
    y_offset  = 0.0;
    marker_id = 0.0;

    % ── Detect ArUco markers ──
    % readArucoMarker returns:
    %   ids   - int32 column vector of detected marker IDs
    %   locs  - Nx4x2 double array of corner pixel coordinates
    %   poses - array of rigidtform3d objects (camera-to-marker transforms)
    [ids, ~, poses] = readArucoMarker(frame, intrinsics, markerSizeMeters);

    if isempty(ids)
        return;  % No markers found
    end

    % ── Priority selection ──
    % Marker 102 (landing target) has higher priority than 101 (takeoff pad).
    targetIdx = 0;

    % Look for marker 102 first
    for i = 1:length(ids)
        if ids(i) == 102
            targetIdx = i;
            break;
        end
    end

    % If 102 not found, look for marker 101
    if targetIdx == 0
        for i = 1:length(ids)
            if ids(i) == 101
                targetIdx = i;
                break;
            end
        end
    end

    if targetIdx == 0
        return;  % No relevant markers found (ignore unknown IDs)
    end

    % ── Extract translation vector ──
    % poses(i).Translation = [tx, ty, tz] in meters
    % In camera coordinate frame:
    %   tx = right of camera center (positive = right)
    %   ty = below camera center (positive = down)
    %   tz = forward from camera (positive = away from camera, i.e., distance)
    tvec = poses(targetIdx).Translation;

    x_offset  = tvec(1);           % Right of center (meters)
    y_offset  = tvec(2);           % Below center (meters)
    marker_id = double(ids(targetIdx));  % 101 or 102
end
```

**Output signals:**

| Signal | Type | Dimensions | Range | Meaning |
|--------|------|-----------|-------|---------|
| `x_offset` | `double` | scalar | −2.0 to +2.0 m | Horizontal offset (right = positive) |
| `y_offset` | `double` | scalar | −2.0 to +2.0 m | Vertical offset (down = positive) |
| `marker_id` | `double` | scalar | 0, 101, or 102 | Which marker was detected (0 = none) |

---

### Stage 3: ROS 2 Publish

**Block:** `Publish` (ROS Toolbox → ROS 2 library)

**What it does:** Packs the three scalar outputs into a `geometry_msgs/Point` message and publishes to the ROS 2 network.

**Block parameters:**

| Parameter | Value |
|-----------|-------|
| Topic | `/erc/vision_targets` |
| Message type | `geometry_msgs/Point` |
| QoS History | `KeepLast` |
| QoS Depth | `10` |

**Message field mapping:**

| Point field | Connected signal | Meaning |
|-------------|-----------------|---------|
| `x` | `x_offset` | Horizontal offset in meters |
| `y` | `y_offset` | Vertical offset in meters |
| `z` | `marker_id` | Marker ID (0, 101, 102) |

**ROS 2 message convention (shared with Python side):**

| `z` value | Interpretation by Python `VisionBridge` |
|-----------|----------------------------------------|
| `101.0` | Takeoff pad marker detected |
| `102.0` | Landing target marker detected |
| `0.0` | No detection (frame processed but nothing found) |
| Negative (e.g., `-1.0`) | Probe detected (x, y contain world position) |

---

## 5. Code Generation Compatibility Matrix

Every function used inside `MATLAB Function` blocks must support C/C++ code generation. Here is the verification status:

| Function | Codegen Support | Since Version | Notes |
|----------|----------------|---------------|-------|
| `readArucoMarker` | ✅ Yes | R2024b | Links to OpenCV 4.7.0 |
| `cameraIntrinsics` | ✅ Yes | R2022b | Use compile-time constants for constructor args |
| `rigidtform3d.Translation` | ✅ Yes | R2022b | Property access on pose output |
| `isempty` | ✅ Yes | Always | Built-in |
| `double()` | ✅ Yes | Always | Type casting |
| `undistortImage` | ⚠️ Partial | R2018a | NOT supported inside MATLAB Function blocks. Use standalone `codegen` only |
| `estworldpose` | ✅ Yes | R2024a | Alternative to `readArucoMarker` pose output |

> [!IMPORTANT]
> `undistortImage` is NOT supported inside Simulink MATLAB Function blocks. However, `readArucoMarker` handles lens distortion internally when you provide a `cameraIntrinsics` object with distortion coefficients. **You do NOT need a separate undistortion step.**

---

## 6. Simulink Model Configuration

### Solver Settings

| Parameter | Value | Rationale |
|-----------|-------|-----------|
| Type | Fixed-step | Required for code generation and real-time execution |
| Solver | `discrete (no continuous states)` | No continuous dynamics in this model |
| Fixed-step size | `1/15` | Matches camera frame rate (15 FPS) |

### Code Generation Settings

| Parameter | Value |
|-----------|-------|
| System target file | `ert.tlc` (Embedded Coder) |
| Language | C++ |
| Hardware board | Raspberry Pi |
| Device address | `<Pi IP address>` |
| Username | `marko` |
| Build action | Build, load, and run |

### ROS 2 Configuration

| Parameter | Value |
|-----------|-------|
| ROS version | ROS 2 |
| ROS 2 distribution | Jazzy |
| Domain ID | 42 |
| Node name | `simulink_vision_node` |

---

## 7. Raspberry Pi 5 Camera Workaround

The Pi 5's new ISP (Image Signal Processor) architecture requires using `libcamera` instead of direct V4L2 access. The Simulink V4L2 block cannot use `libcamera` natively. The workaround:

**Create a GStreamer pipeline that captures from the camera via `libcamera` and outputs to a virtual V4L2 device.**

### One-time setup on the Pi:

```bash
# Install v4l2loopback kernel module
sudo apt install v4l2loopback-dkms

# Load the module (creates /dev/video10 as a virtual device)
sudo modprobe v4l2loopback video_nr=10 card_label="SimulinkCam" exclusive_caps=1
```

### Start the camera bridge (run before launching the Simulink node):

```bash
# GStreamer pipeline: libcamera → v4l2loopback virtual device
libcamera-vid --width 640 --height 480 --framerate 15 \
    --codec yuv420 --output - --timeout 0 | \
    gst-launch-1.0 fdsrc ! \
    rawvideoparse width=640 height=480 format=i420 framerate=15/1 ! \
    videoconvert ! video/x-raw,format=RGB ! \
    v4l2sink device=/dev/video10
```

### Verify it works:

```bash
# Check the virtual device exists
v4l2-ctl --device=/dev/video10 --all

# Grab a test frame
ffmpeg -f v4l2 -i /dev/video10 -frames:v 1 test_frame.jpg
```

The Simulink V4L2 block then reads from `/dev/video10` as if it were a standard USB camera.

> [!TIP]
> Create a systemd service or startup script to run the GStreamer bridge automatically at boot. This ensures the virtual camera is always available when the Simulink node starts.

---

## 8. Output Protocol — Interface with Python

The Simulink node and the Python `VisionBridge` class communicate through a shared contract:

**Topic:** `/erc/vision_targets`
**Message type:** `geometry_msgs/msg/Point`
**Publishing rate:** 15 Hz (matches camera frame rate)

**Frame conventions:**

```
                Camera Frame (looking down)
                ┌─────────────────────┐
                │                     │
                │     ·──────► +X     │
                │     │  (right)      │
                │     │               │
                │     ▼ +Y            │
                │     (down/forward)  │
                │                     │
                └─────────────────────┘

   msg.x = offset right of camera center (meters)
   msg.y = offset below camera center (meters)
   msg.z = marker ID (101, 102, 0, or negative for probes)
```

**The Python VisionBridge maps these to its internal dict:**
```python
target = {
    "marker_id": int(msg.z),       # 101 or 102
    "x_offset_m": msg.x,           # Right of center
    "y_offset_m": msg.y,           # Below center (≈ forward of drone)
    "age_s": time.time() - timestamp
}
```

---

# PART 2 — Step-by-Step Implementation

---

## Step 1: Verify MATLAB Toolbox Licenses

Before starting, confirm all required toolboxes are installed. In the MATLAB Command Window:

```matlab
>> ver
```

**Required toolboxes (all must appear in the output):**

- [ ] MATLAB (base)
- [ ] Simulink
- [ ] Simulink Coder
- [ ] Embedded Coder
- [ ] MATLAB Coder
- [ ] Computer Vision Toolbox
- [ ] ROS Toolbox
- [ ] Simulink Support Package for Raspberry Pi Hardware

If any are missing, install them via the MATLAB Add-On Explorer (`Home → Add-Ons → Get Add-Ons`) or contact your license administrator.

**Verify `readArucoMarker` exists and supports codegen:**

```matlab
>> help readArucoMarker
>> doc readArucoMarker
% Scroll to "Extended Capabilities" → confirm "C/C++ Code Generation" is listed
```

---

## Step 2: Camera Calibration

Camera calibration computes the intrinsic parameters (focal length, principal point, distortion coefficients) needed for accurate pose estimation.

### 2.1 Print a calibration pattern

```matlab
% Generate and print a checkerboard pattern
% Use a standard 9×7 checkerboard with 25mm squares
% Print on A4 paper, ensure "Actual Size" (no scaling) in print dialog
```

Alternatively, download a pre-made pattern from [MATLAB Camera Calibration documentation](https://www.mathworks.com/help/vision/ug/camera-calibration.html).

### 2.2 Capture calibration images

On the Raspberry Pi, capture 15–20 images of the checkerboard from different angles:

```bash
# On the Pi — take photos from different angles and distances
for i in $(seq 1 20); do
    libcamera-still -o ~/calibration/calib_$i.jpg --width 640 --height 480
    echo "Captured image $i. Reposition the checkerboard, then press Enter."
    read
done
```

Transfer the images to your laptop:

```bash
# On your laptop
scp marko@<pi-ip>:~/calibration/*.jpg ./calibration_images/
```

### 2.3 Run calibration in MATLAB

**Option A: Using the Camera Calibrator app (interactive, recommended for first time):**

```matlab
>> cameraCalibrator
% 1. Click "Add Images" → select all calibration images
% 2. Set square size to 25mm (or your actual square size)
% 3. Click "Calibrate"
% 4. Review reprojection errors — remove images with error > 1 pixel
% 5. Click "Export Camera Parameters" → saves to workspace as 'cameraParams'
```

**Option B: Scripted calibration:**

```matlab
% Detect checkerboard corners in all images
imageDir = './calibration_images/';
images = imageDatastore(imageDir);
[imagePoints, boardSize] = detectCheckerboardPoints(images.Files);

% Define world coordinates of the checkerboard corners
squareSize = 25;  % millimeters
worldPoints = patternWorldPoints("checkerboard", boardSize, squareSize);

% Get image size from the first image
I = readimage(images, 1);
imageSize = [size(I, 1), size(I, 2)];

% Calibrate
cameraParams = estimateCameraParameters(imagePoints, worldPoints, ...
    'ImageSize', imageSize, ...
    'EstimateSkew', false, ...
    'EstimateTangentialDistortion', true, ...
    'NumRadialDistortionCoefficients', 3);

% Display results
showReprojectionErrors(cameraParams);
figure; showExtrinsics(cameraParams);

% Print the values you need for the Simulink model
fprintf('Focal Length: [%.1f, %.1f]\n', cameraParams.Intrinsics.FocalLength);
fprintf('Principal Point: [%.1f, %.1f]\n', cameraParams.Intrinsics.PrincipalPoint);
fprintf('Image Size: [%d, %d]\n', cameraParams.Intrinsics.ImageSize);
fprintf('Radial Distortion: [%.6f, %.6f, %.6f]\n', cameraParams.Intrinsics.RadialDistortion);

% Save for later use
save('cameraParams.mat', 'cameraParams');
```

### 2.4 Record the calibration values

After calibration, write down these numbers — they go into the MATLAB Function block:

```
Focal Length:     [fx, fy] = [_____, _____]     pixels
Principal Point:  [cx, cy] = [_____, _____]     pixels
Image Size:       [rows, cols] = [480, 640]
Radial Distortion: [k1, k2, k3] = [_____, _____, _____]
```

---

## Step 3: Print ArUco Markers for Testing

Generate and print ArUco markers for desktop and field testing.

```matlab
% Generate marker 101 (takeoff pad)
marker101 = generateArucoMarker("DICT_ARUCO_ORIGINAL", 101, 600);
imwrite(marker101, 'aruco_101.png');

% Generate marker 102 (landing target)
marker102 = generateArucoMarker("DICT_ARUCO_ORIGINAL", 102, 600);
imwrite(marker102, 'aruco_102.png');

% Display for verification
figure;
subplot(1,2,1); imshow(marker101); title('Marker 101 (Takeoff Pad)');
subplot(1,2,2); imshow(marker102); title('Marker 102 (Landing Target)');
```

Print both markers at **exactly 15 cm × 15 cm** (measure with a ruler after printing). The physical size must match `markerSizeMeters = 0.15` in the code.

---

## Step 4: Desktop MATLAB Prototype (Verify the Math)

Before building the Simulink model, verify the complete pipeline works as a plain MATLAB script.

### 4.1 Test with a photo of the marker

Take a photo of the printed marker 102 with your webcam or phone. Save it as `test_marker.jpg`.

```matlab
% Load image and calibration
I = imread('test_marker.jpg');
load('cameraParams.mat');

% Detect and estimate pose
markerSize = 0.15;  % 15 cm in meters
[ids, locs, poses] = readArucoMarker(I, cameraParams.Intrinsics, markerSize);

% Display results
fprintf('Detected %d markers\n', length(ids));
for i = 1:length(ids)
    tvec = poses(i).Translation;
    fprintf('  Marker %d: x=%.3f m, y=%.3f m, z=%.3f m (distance)\n', ...
        ids(i), tvec(1), tvec(2), tvec(3));
end

% Visualize detections
figure; imshow(I); hold on;
for i = 1:length(ids)
    corners = squeeze(locs(i, :, :));  % 4×2 matrix
    plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'g-', 'LineWidth', 2);
    text(corners(1,1), corners(1,2)-10, sprintf('ID: %d', ids(i)), ...
        'Color', 'green', 'FontSize', 14, 'FontWeight', 'bold');
end
title('ArUco Detection Result');
```

**What to verify:**
- The marker ID is correctly detected (101 or 102).
- The translation vector makes physical sense:
  - If the marker is 30 cm to the right of center, `tvec(1)` ≈ 0.30.
  - If the marker is 1 m away from the camera, `tvec(3)` ≈ 1.0.
- Corner overlays align with the actual marker corners in the image.

### 4.2 Test with live webcam (desktop)

```matlab
cam = webcam;
markerSize = 0.15;
load('cameraParams.mat');

figure;
while true
    frame = snapshot(cam);
    [ids, locs, poses] = readArucoMarker(frame, cameraParams.Intrinsics, markerSize);

    imshow(frame); hold on;
    for i = 1:length(ids)
        corners = squeeze(locs(i, :, :));
        plot([corners(:,1); corners(1,1)], [corners(:,2); corners(1,2)], 'g-', 'LineWidth', 2);
        tvec = poses(i).Translation;
        text(corners(1,1), corners(1,2)-10, ...
            sprintf('ID:%d  x:%.2f y:%.2f z:%.2f', ids(i), tvec(1), tvec(2), tvec(3)), ...
            'Color', 'yellow', 'FontSize', 12, 'FontWeight', 'bold', 'BackgroundColor', 'black');
    end
    hold off;
    drawnow;
end
```

Hold the printed marker in front of your webcam. Verify the offsets update in real-time and match the marker's physical position.

---

## Step 5: Build the Simulink Model

### 5.1 Create a new model

1. In MATLAB: `File → New → Simulink Model`
2. Save as `erc_vision_node.slx`
3. Open **Modeling tab → Model Settings** (or press `Ctrl+E`)

### 5.2 Configure solver

In Model Settings:
- **Solver → Type:** `Fixed-step`
- **Solver → Solver:** `discrete (no continuous states)`
- **Solver → Fixed-step size:** `1/15`

### 5.3 Configure hardware

In Model Settings:
- **Hardware Implementation → Hardware board:** `Raspberry Pi`
- **Hardware Implementation → Device Address:** `<your Pi's IP>`
- **Hardware Implementation → Username:** `marko`
- **Hardware Implementation → Password:** `<your password>`
- **Hardware Implementation → Build directory:** `~/ros2_ws/src`

### 5.4 Enable ROS 2

1. Go to the **Apps** tab in the Simulink toolstrip
2. Click **ROS Toolbox** (or **Robot Operating System**)
3. In the **ROS** tab that appears, set **ROS Version** to **ROS 2**
4. Click **Configure ROS 2 Network** → set Domain ID to `42`

### 5.5 Add the V4L2 Video Capture block

1. Open the Simulink Library Browser (`View → Library Browser`)
2. Navigate to: `Simulink Support Package for Raspberry Pi Hardware → Raspberry Pi`
3. Drag `V4L2 Video Capture` onto the canvas
4. Double-click the block and configure:
   - **Device name:** `/dev/video10` (the virtual device from Section 7)
   - **Image size:** `640 x 480`
   - **Image color space:** `RGB`
   - **Data type:** `uint8`
   - **Sample time:** `1/15`

### 5.6 Add the MATLAB Function block

1. From the Library Browser: `Simulink → User-Defined Functions → MATLAB Function`
2. Drag it onto the canvas and connect the V4L2 output to its input
3. Double-click to open the function editor
4. Replace the default code with the `detectAndEstimate` function from Part 1, Section 4 — Stage 2
5. **Update the camera intrinsic values** in the code with YOUR calibration results from Step 2

### 5.7 Add the ROS 2 Publish block

1. From the Library Browser: `ROS Toolbox → ROS 2 → Publish`
2. Drag it onto the canvas
3. Double-click and configure:
   - **Topic source:** `Specify your own`
   - **Topic:** `/erc/vision_targets`
   - **Message type:** `geometry_msgs/Point`

### 5.8 Create the message bus

The ROS 2 Publish block expects a `Point` bus as input. You need to create it:

1. From the Library Browser: `ROS Toolbox → ROS 2 → Blank Message`
2. Drag it onto the canvas
3. Configure it for message type: `geometry_msgs/Point`
4. This creates a blank Point message bus

Then use `Bus Assignment` blocks (or a `Bus Creator`) to map your MATLAB Function outputs into the message:

```
MATLAB Function                     Bus Creator          Publish
─────────────                       ───────────          ───────
x_offset ──────────────────────► x ─┐
y_offset ──────────────────────► y ─┼──► Point bus ──► /erc/vision_targets
marker_id ─────────────────────► z ─┘
```

**Alternative approach using a second MATLAB Function block** (simpler):

Instead of Bus Creator wiring, add a small MATLAB Function block that creates the message:

```matlab
function msg = createPointMsg(x_off, y_off, mk_id)
%#codegen
    msg = rosmessage("geometry_msgs/Point", "DataFormat", "struct");
    msg.X = x_off;
    msg.Y = y_off;
    msg.Z = mk_id;
end
```

> [!NOTE]
> The exact wiring method depends on your MATLAB version. In R2024b+, the `Publish` block may accept direct signal inputs or require a bus. Consult the ROS Toolbox documentation for your specific version.

### 5.9 Final canvas layout

Your finished model should look like this:

```
┌──────────┐    ┌─────────────────┐    ┌────────────────┐    ┌──────────┐
│  V4L2    │    │ MATLAB Function │    │  Bus Creator   │    │  ROS 2   │
│  Video   │───►│ detectAndEstim  │───►│  x → Point.X   │───►│  Publish │
│  Capture │    │ ate()           │    │  y → Point.Y   │    │          │
│          │    │                 │    │  z → Point.Z   │    │  Topic:  │
│ 640×480  │    │ Outputs:        │    │                │    │ /erc/    │
│ RGB      │    │  x_offset       │    └────────────────┘    │ vision_  │
│ uint8    │    │  y_offset       │                          │ targets  │
│ @15fps   │    │  marker_id      │                          └──────────┘
└──────────┘    └─────────────────┘

                ┌─────────────────┐
                │  Display        │  (for desktop debugging — remove before deploy)
                │  (optional)     │
                └─────────────────┘
```

---

## Step 6: Desktop Simulation

Before deploying to the Pi, test the model logic on your desktop.

### 6.1 Replace V4L2 with a test image source

The V4L2 block only works on the Pi. For desktop simulation:

1. Comment out (or delete) the V4L2 block
2. Add a `From Workspace` block or `Image From File` block
3. Feed it a test image containing an ArUco marker
4. Connect it to the MATLAB Function block

```matlab
% Create a test image variable in the base workspace
I = imread('test_marker.jpg');
I = imresize(I, [480, 640]);  % Ensure correct size
simInput = timeseries(uint8(I), 0);  % Single frame at t=0
```

### 6.2 Run the simulation

1. Click the **Run** button (or press `Ctrl+T`)
2. Check the Display blocks for output values
3. Verify: `marker_id` shows 102 (or 101), `x_offset` and `y_offset` are reasonable

### 6.3 Test with multiple images

Create a sequence of test images with the marker at different positions:
- Center of frame → offsets should be near zero
- Left side → x_offset should be negative
- Right side → x_offset should be positive
- Far from camera → z (distance) should be larger

---

## Step 7: Deploy to Raspberry Pi

### 7.1 Prerequisite: Ensure OpenCV 4.7+ is on the Pi

The generated code from `readArucoMarker` links to OpenCV. Install it:

```bash
# On the Raspberry Pi
sudo apt update
sudo apt install libopencv-dev
pkg-config --modversion opencv4    # Should show 4.7.x or newer
```

### 7.2 Prerequisite: Start the camera bridge

```bash
# On the Raspberry Pi — Terminal 1
sudo modprobe v4l2loopback video_nr=10 card_label="SimulinkCam" exclusive_caps=1

libcamera-vid --width 640 --height 480 --framerate 15 \
    --codec yuv420 --output - --timeout 0 | \
    gst-launch-1.0 fdsrc ! \
    rawvideoparse width=640 height=480 format=i420 framerate=15/1 ! \
    videoconvert ! video/x-raw,format=RGB ! \
    v4l2sink device=/dev/video10
```

### 7.3 Restore the V4L2 block

Switch back from the desktop test source to the V4L2 Video Capture block in your Simulink model.

### 7.4 Deploy

**Automated deployment:**

1. In the Simulink **ROS** tab, set **Deploy to:** `Remote Device`
2. Click **Manage Remote Device** → verify IP, username, password
3. Click **Build & Run**
4. Simulink will:
   - Generate C++ code
   - Transfer files to `~/ros2_ws/src/erc_vision_node/` on the Pi
   - Run `colcon build` on the Pi
   - Start the node

**Manual deployment (if automated fails):**

1. In Simulink, click **Generate Code** (not Build & Run)
2. Find the generated code folder (check MATLAB console for path)
3. Copy it to the Pi:
   ```bash
   scp -r ./erc_vision_node_ert_rtw/ marko@<pi-ip>:~/ros2_ws/src/erc_vision_node/
   ```
4. On the Pi:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/jazzy/setup.bash
   colcon build --packages-select erc_vision_node
   source install/setup.bash
   ros2 run erc_vision_node erc_vision_node
   ```

### 7.5 Verify the deployed node

On the Pi (Terminal 2):

```bash
source ~/.bashrc

# Check the node is running
ros2 node list
# Should show: /simulink_vision_node

# Check the topic is publishing
ros2 topic list
# Should show: /erc/vision_targets

# Check the publish rate
ros2 topic hz /erc/vision_targets
# Should show: ~15 Hz

# Read a message
ros2 topic echo /erc/vision_targets --once
# Should show: x: 0.0, y: 0.0, z: 0.0  (if no marker visible)
```

### 7.6 Test with a real marker

Hold the printed ArUco marker (ID 102) under the camera:

```bash
# Watch the values update in real-time
ros2 topic echo /erc/vision_targets
```

**Expected:** `z` field changes to `102.0`, `x` and `y` show the offset in meters.

Move the marker:
- To the right → `x` becomes positive
- To the left → `x` becomes negative
- Away from camera → values get smaller (farther = more centered in FOV)

---

## Step 8: Integration Test with Python State Machine

### 8.1 Full system test

With the Simulink vision node running (from Step 7), start the Python mission controller:

```bash
# Terminal 1: GStreamer camera bridge (already running from Step 7.2)
# Terminal 2: Simulink ROS 2 node (already running from Step 7.4)

# Terminal 3: Python mission controller
cd ~/indomitus-drone
source ~/.bashrc && source .venv/bin/activate
python3 main.py
```

### 8.2 What to expect

1. `main.py` starts and enters IDLE state
2. If Pixhawk is not connected: stays in IDLE (normal)
3. If Pixhawk IS connected: proceeds through TAKEOFF → SEARCH
4. When the camera sees marker 102, the state machine should print:
   ```
   [STATE] Landing target DETECTED at offset (+0.150, -0.080)m
   [STATE] SEARCH -> ALIGN
   ```

### 8.3 Verify the handoff

The key moment is when the Simulink detection reaches the Python state machine. Run this quick diagnostic:

```bash
# Terminal A: Simulink node publishing
# Terminal B: Check both sides see the same data

# Python side — does VisionBridge receive messages?
cd ~/indomitus-drone
source ~/.bashrc && source .venv/bin/activate
python3 -c "
import rclpy, time
rclpy.init()
from src.ros_bridge.vision_subscriber import VisionBridge
bridge = VisionBridge(topic='/erc/vision_targets', grid_config={
    'origin_x_m': -2.5, 'origin_y_m': -0.5, 'cell_size_m': 1.0,
    'columns': ['A','B','C','D','E','F'], 'rows': [1,2,3,4,5,6]
})
start = time.time()
while time.time() - start < 5:
    bridge.spin_once()
    t = bridge.get_latest_target()
    if t: print(f'  Marker {t[\"marker_id\"]} at ({t[\"x_offset_m\"]:+.3f}, {t[\"y_offset_m\"]:+.3f})')
    time.sleep(0.1)
print(f'Messages: {bridge.get_message_count()}')
bridge.shutdown(); rclpy.shutdown()
"
```

If this shows marker detections, the full pipeline is working: Camera → Simulink → ROS 2 → Python.

---

## Appendix A: Probe Detection (Future Extension)

Probe detection can be added as a second processing branch in the MATLAB Function block:

```matlab
% After ArUco detection, check for probes by color
% Probes are colored objects (specific color TBD by competition)
%
% Approach:
% 1. Convert frame to HSV
% 2. Threshold for probe color range
% 3. Find connected components
% 4. Compute centroid in pixel coordinates
% 5. Project to world coordinates using camera height
% 6. Publish as Point(x=world_x, y=world_y, z=-1.0)  ← negative z = probe
```

This is a separate development task. The marker detection pipeline must work first.

---

## Appendix B: Troubleshooting

| Problem | Cause | Solution |
|---------|-------|---------|
| `readArucoMarker` not found | MATLAB version too old | Upgrade to R2024a+ (detection) or R2024b+ (codegen) |
| Code generation error for `readArucoMarker` | MATLAB version < R2024b | Upgrade to R2024b+ |
| V4L2 block: "Invalid argument" on Pi 5 | Pi 5 CSI camera architecture | Use GStreamer virtual device (Section 7) |
| Marker detected but pose is wrong | Bad calibration or wrong marker size | Re-calibrate camera; verify marker is exactly 15cm |
| `colcon build` fails on Pi | Missing OpenCV headers | Run `sudo apt install libopencv-dev` |
| ROS 2 topic not visible | Domain ID mismatch | Set `ROS_DOMAIN_ID=42` in both Simulink and terminal |
| Model generates code but won't build | Incompatible compiler on Pi | Ensure `g++` is installed: `sudo apt install g++` |
| Detection is slow (< 5 FPS) | Resolution too high | Reduce to 640×480 or 320×240 |
| Marker not detected at 2m distance | Marker too small in frame | Ensure marker is 15cm. At 640×480 + 120° lens, a 15cm marker at 2m is ~30 pixels wide — should work but is near the limit |
