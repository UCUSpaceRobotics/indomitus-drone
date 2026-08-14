# Indomitus Drone — ERC 2026

Autonomous quadcopter system for the **European Rover Challenge 2026 — Droning Sub-Task**.

## Architecture

| Layer | Technology | Role |
|-------|-----------|------|
| **Eyes** | Simulink → compiled C++ ROS 2 node | ArUco detection, probe detection, coordinate estimation |
| **Brain** | Python + `rclpy` | State machine (Search → Align → Land), flight decisions |
| **Muscle** | `pymavlink` → Pixhawk 6C (ArduCopter) | Flight dynamics, EKF3 sensor fusion, motor control |

```
[Arducam IMX708] → [Simulink C++ ROS 2 Node] → /erc/vision_targets → [Python State Machine] → [pymavlink] → [Pixhawk 6C]
```

## Hardware

- Frame: F450 with DJI 2212 920KV motors
- FC: Pixhawk 6C (ArduCopter)
- Computer: Raspberry Pi 5 (16 GB)
- Camera: Arducam IMX708 (downward)
- Odometry: Microair MTF-01 optical flow
- GPS: Holybro M10

## Quick Start (on the Raspberry Pi)

```bash
# 1. Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

source ~/.bashrc           # loads ROS 2 Jazzy environment
source .venv/bin/activate  # activates the Python venv

# 2. Run the mission
cd ~/indomitus-drone
sudo -E python3 main.py
```

## Setup

See [docs/SETUP.md](docs/SETUP.md) for full Raspberry Pi configuration guide.

## License

MIT — see [LICENSE](LICENSE) for details.

## Team

UCU Space Robotics — Indomitus
