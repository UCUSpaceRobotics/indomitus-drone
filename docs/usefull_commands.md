# Useful Commands — Indomitus Drone

Quick-reference commands for launching the autonomous drone stack on the Raspberry Pi 5. Run each step in a **separate SSH terminal** in the order shown below.

| Terminal | Component | Role |
|----------|-----------|------|
| **Terminal 1** | Vision Pipeline (`start_vision.sh`) | Starts `v4l2loopback` camera tee, `erc_vision_node`, and the HTTP web HUD streamer |
| **Terminal 2** | Mission Supervisor (`erc_mission_node`) | Runs the compiled Simulink high-level state supervisor node on ROS 2 |
| **Terminal 3** | Flight Controller Bridge (`main.py`) | Runs the Python state machine, LED controller, and MAVLink Pixhawk interface |

---

## 1. Start Vision Bridge (Terminal 1)

Initializes the virtual camera loopback devices (`/dev/video10`, `/dev/video11`), launches the Simulink C++ vision node (`erc_vision_node`), and starts the live web stream HUD on port `5000`.

```bash
cd ~/indomitus-drone
chmod +x scripts/start_vision.sh
./scripts/start_vision.sh
```

---

## 2. Start ROS 2 Mission Node (Terminal 2)

Sources the local ROS 2 workspace and runs the Simulink supervisory state machine node (`erc_mission_node`) on Domain ID `27`.

```bash
cd ~/ros2_ws/
source install/setup.bash
export ROS_DOMAIN_ID=27
ros2 run erc_mission_node erc_mission_node
```

---

## 3. Start Mission Execution (Terminal 3)

Activates the Python virtual environment and launches the main autonomous flight orchestrator (`main.py`) on Domain ID `27`.

```bash
source .venv/bin/activate
export ROS_DOMAIN_ID=27
python3 main.py
```
