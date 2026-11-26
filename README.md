# Indomitus Drone — ERC 2026

Autonomous quadcopter system developed by **UCU Space Robotics — Indomitus** for the **European Rover Challenge (ERC) 2026 — Droning Sub-Task**.

## System Architecture

| Layer | Component | Technology | Role |
|-------|-----------|------------|------|
| **Vision ("Eyes")** | `erc_vision_node` | Simulink → C++ ROS 2 node | 15 Hz ArUco 101/102 detection & 6-DoF pose (`[marker_id, tvec, rvec]`) |
| **Supervisor ("Brain")** | `erc_mission_node` | Simulink → C++ ROS 2 node | 15 Hz high-level mission state machine (`IDLE → TAKEOFF → SEARCH → DESCEND → COMPLETE`) |
| **Execution Bridge** | `main.py` | Python + `rclpy` + `pymavlink` | 48-step expanding-ring search, 15 Hz `LANDING_TARGET` stream, safety overrides, SPI status LED |
| **Flight Control ("Muscle")** | Pixhawk 6C | ArduCopter + EKF3 | Optical-flow indoor navigation, built-in precision landing, motor control |

```text
[Arducam IMX708] ──► v4l2loopback (/dev/video10, /dev/video11)
                         ├──► vision_web_streamer.py (HTTP MJPEG HUD :5000)
                         └──► erc_vision_node ──(/erc/vision_targets)──► erc_mission_node
                                     │                                         │
                                     ▼                                 (/erc/mission_state)
                                  main.py ◄────────────────────────────────────┘
                               (VisionBridge + MissionController + LEDController)
                                     │
                           UART /dev/ttyAMA0 (MAVLink 2.0)
                                     ▼
                            [Pixhawk 6C — ArduCopter]
```

## Hardware Highlights

- **Airframe:** F450 quadcopter, DJI 2212 920KV motors, 1045R props, 4S LiPo
- **Flight Controller:** Pixhawk 6C (ArduCopter) with **Microair MTF-01** optical flow + rangefinder
- **Companion Computer:** Raspberry Pi 5 (16 GB) powered via 5V 5A UBEC
- **Vision & FPV:** Downward **Arducam IMX708** (CSI, for CV); isolated forward **Foxeer Cat 4** FPV camera + **Raspberry Pi composite desktop output** wired through a 3-channel 5.8 GHz VTX switcher
- **Safety & Geofence:** **WS2812 7-LED SPI module** (Green = Manual, Red = Autonomous) and **Holybro M10 GPS** (used strictly for the Mission Planner geofence)

## Repository Structure

- [`config/`](config/) — Mission parameters (`mission_params.yaml`, `ROS_DOMAIN_ID=27`) and camera calibration `.npz` files
- [`docs/`](docs/) — [ARCHITECTURE.md](docs/ARCHITECTURE.md), [SETUP_RASPBERRY.md](docs/SETUP_RASPBERRY.md), [usefull_commands.md](docs/usefull_commands.md), and `archive/`
- [`hardware/`](hardware/) — CAD models, electrical schematics, and hardware datasheets
- [`matlab/`](matlab/) — Simulink models (`erc_vision_node.slx`, `erc_mission_node.slx`), calibration loader, and ROS 2 build script
- [`scripts/`](scripts/) — Vision pipeline launcher (`start_vision.sh`), HTTP HUD streamer (`vision_web_streamer.py`), and ROS 2 Jazzy installer
- [`src/`](src/) — Python flight controller bridge (`comm/`, `navigation/`, `ros_bridge/`, `utils/`)
- [`tests/`](tests/) — Unit tests and hardware test scripts

## Quick Start (on Raspberry Pi 5)

Run each component in a separate terminal (see [docs/usefull_commands.md](docs/usefull_commands.md)):

```bash
# Terminal 1 — Camera tee + Simulink vision node + HTTP stream (:5000)
cd ~/indomitus-drone
chmod +x scripts/start_vision.sh
./scripts/start_vision.sh
```

```bash
# Terminal 2 — Simulink supervisory state machine node
cd ~/ros2_ws/
source install/setup.bash
export ROS_DOMAIN_ID=27
ros2 run erc_mission_node erc_mission_node
```

```bash
# Terminal 3 — Python execution bridge & MAVLink interface
source .venv/bin/activate
export ROS_DOMAIN_ID=27
python3 main.py
```

## Documentation

- **System Architecture:** [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)
- **Improvement Plan & Technical Debt Roadmap:** [docs/IMPROVEMENT_PLAN.md](docs/IMPROVEMENT_PLAN.md)
- **Raspberry Pi 5 & ROS 2 Jazzy Setup:** [docs/SETUP_RASPBERRY.md](docs/SETUP_RASPBERRY.md)
- **Quick Launch Commands:** [docs/usefull_commands.md](docs/usefull_commands.md)

## License

MIT — see [LICENSE](LICENSE) for details.
