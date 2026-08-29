Start vision bridge

```
cd ~/indomitus-drone
chmod +x scripts/start_vision.sh
./scripts/start_vision.sh
```

Start mission

```
source .venv/bin/activate
export ROS_DOMAIN_ID=27
python3 main.py
```