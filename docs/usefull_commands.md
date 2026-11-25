Start vision bridge

```bash
cd ~/indomitus-drone
chmod +x scripts/start_vision.sh
./scripts/start_vision.sh
```

Start ros2
```bash
cd ~/ros2_ws/
source install/setup.bash
export ROS_DOMAIN_ID=27
ros2 run erc_mission_node erc_mission_node
```
Start mission

```bash
source .venv/bin/activate
export ROS_DOMAIN_ID=27
python3 main.py
```