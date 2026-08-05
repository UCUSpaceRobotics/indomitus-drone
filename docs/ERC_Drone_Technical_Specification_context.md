# ERC Droning Sub-Task: Autonomous Quadcopter Technical Specification

## 1. System Architecture & Hardware Stack

### 1.1 Frame & Propulsion
* **Chassis:** F450 quadcopter frame equipped with landing gears.
* **Custom Mounts:** Custom-manufactured holder integrated into the frame to securely mount the computer vision camera, optical flow sensor, and LiPo battery.
* **Motors:** DJI 2212 920KV brushless motors.
* **Propellers:** 1045R (10x4.5) propellers.
* **Power Source:** 4S LiPo battery pack.

### 1.2 Computing & Control
* **Low-Level Flight Controller:** Pixhawk 6C running **ArduCopter** firmware (configured via Mission Planner). Manages real-time flight dynamics, motor PWM output, and low-level stabilization.
* **Companion Computer:** Raspberry Pi 5 (16 GB RAM). Serves as the central processing unit for computer vision, high-level autonomous navigation, and state management. Powered via a 5V 5A UBEC.

### 1.3 Sensor Suite & Navigation
* **Vision (Downward):** Arducam IMX708 12MP HDR 120° Wide Angle Camera Module for Raspberry Pi. Directly connected to the Pi to provide real-time imagery of the ground for marker and probe detection.
* **Vision (Forward/Pilot):** Foxeer Cat 4 FPV camera. Dedicated purely for the pilot's forward view; it is **not** connected to the Raspberry Pi.
* **Odometry:** Microair MTF-01 optical flow sensor (pointing downward) for stable, GPS-denied autonomous hovering and flight.
* **Global Positioning:** Holybro M10 GPS and Compass module.

### 1.4 Communication & Video Routing
* **RC Receiver:** RadioMaster RP1 V2 ExpressLRS 2.4GHz Nano Receiver.
* **Telemetry Link:** 3DR Radio V5 Telemetry 433MHz module transmitting MAVLink data to the ground station.
* **Video Transmitter (VTX):** Race Ranger VTX transmitting on the 5.8 GHz band, powered by a 9V mini DC-DC converter. *Note: The VTX is completely isolated from the Pixhawk, meaning there is no onboard OSD (On-Screen Display) overlay.*
* **Video Switcher Logic:** A 3-channel video switcher toggles the VTX feed between two analog sources:
    1.  The forward-facing Foxeer Cat 4 FPV camera.
    2.  The composite video output from the Raspberry Pi.
* **Composite Video Behavior:** By default, the Pi's composite output displays the standard Raspberry Pi OS desktop environment. To transmit the downward camera's live feed over this analog link, a dedicated script must be launched on the Pi.

## 2. Software Architecture

### 2.1 Communication Protocol
* The Raspberry Pi and Pixhawk communicate via a hardwired **UART connection**.
* Messages are exchanged using the **MAVLink protocol**.

### 2.2 Control Logic & Simulation
* The high-level software on the Raspberry Pi is structured as a **State Machine**, with all core logic and computer vision pipelines written entirely in Python (utilizing `pymavlink` for communication and `cv2` for image processing).
* MATLAB/Simulink is exclusively utilized for Software-In-The-Loop (SITL) simulation and testing. To validate the system, our Python code is launched and evaluated against a simulated MATLAB environment before physical deployment.
* The primary camera library utilized on the Raspberry Pi to interface with the downward-facing Arducam hardware is `rpicam`.

### 2.3 Sensor Fusion
* State estimation and sensor fusion are handled entirely by the **Pixhawk firmware**. The built-in Extended Kalman Filter (EKF3) natively merges optical flow, GPS, and IMU data without requiring manual filter setup on the companion computer.

## 3. ERC Droning Sub-Task Mission Parameters & Rules
Based on the ERC 2026 rulebook, the drone must accomplish the following objectives within a 10 x 10 x 4 m enclosed cage. Drones must be assembled by teams; no COTS (Commercial Off-The-Shelf) drones are allowed.

### 3.1 Preflight Safety Checks
During the preflight check, teams must present the implementation of the following features:
* Operator connection providing real-time feedback with a view of critical telemetry flight parameters and video feed.
* Ability for the drone operator to switch to manual/remote control mode on demand (when requested by Jury).
* Automatic mid-air stability holding.
* Initialization of automatic landing on demand (when requested by Jury).
* Fail-safe mechanisms for loss of connection with remote control/ground station, and battery/power loss or positioning system glitches.
* Polygon inclusion geofencing configured for the competition area (must be large enough to avoid false triggers).

### 3.2 Mission Execution
* **Time Constraints:** Each team has 15 minutes to prepare to the task (outside the cage) and 30 minutes to execute the task.
* **Flight Mode:** The mission is to be performed only in automated mode. Any manual interventions during the flight will be penalized.
* **Core Objective:** The mission consists of lift-off, detecting the landing spot, and precision landing at the landing spot. This sequence is to be repeated 3 times. After landing, the drone will be manually placed back at the lift-off spot.
* **Operating Area:** The effective area of competition is a disc with a radius of 3 m in the middle of the cage.
* **Take-off:** The lift-off spot is a 1 x 1 m square in the middle of the cage, marked with marker ID 101 (15 x 15 cm) from the ArUco original library.
* **Landing Target:** Provided by organizers, it is a disc with a radius of 0.5 m, randomly positioned within the 3 meters radius around the lift-off spot for each of the 3 missions. It is marked with ArUco ID 102 (15 x 15 cm). The landing is considered successful if any part of the drone is touching the landing target.

### 3.3 Probe Detection & Additional Scoring
* **Probe Detection:** Teams can earn additional points for detecting and estimating the positions of 3 probes scattered within the 3 meters radius around the lift-off spot. 
* **Probe Persistence:** The position of the probes will remain unchanged between the 3 missions, but detections in each of the missions are scored individually. Teams receive 5 points per detection per mission (up to a total of 45 points).
* **Grid Localization:** The area of competition is virtually divided into 1 x 1 m sectors. The probe detection system must point to locations of the probes using IDs of those sectors (e.g., sector A2 covers coordinates 0m; 0.5m to 0.5m; 1m).
* **Custom Landing Platform (Optional):** Teams can earn additional points by preparing their own landing targets attachable to their rover (confined within a disc with a radius of 0.5 m). The platform does not have to use an ArUco marker. Teams can receive 15 points for preparing the platform and 15 points for each successful landing on it (up to a total of 45 points).

## 4. Current Development Status & Required Implementations

The physical hardware stack is mapped out, and the core ArduCopter firmware is established. The following features are currently pending implementation:

1.  **State Indicator:** Integration of a physical light indicator to visually broadcast the drone's current mode (Autonomous vs. Manual).
2.  **Computer Vision Pipeline:** 
    *   ArUco marker detection (IDs 101 and 102).
    *   Marker position estimation translated into relative coordinates.
    *   Probe detection algorithm and mapping to the 1x1m alphanumeric grid sectors (A1, B2, etc.).
3.  **Autonomous Navigation Logic:**
    *   Marker search algorithm (to sweep the 3m radius).
    *   Relative coordinate tracking.
    *   Precision landing control loops based on visual target alignment.
    *   Implement geofence
    *   Configure MATLAB SIL simulation