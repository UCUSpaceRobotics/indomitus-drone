#!/usr/bin/env bash
#
# start_vision_test.sh
#
# Starts, in one shot:
#   1. v4l2loopback module + rpicam-vid -> gst-launch tee pipeline (video10/video11)
#   2. erc_vision_node (ROS2, ros2_ws)
#   3. vision_web_streamer.py (indomitus-drone venv)
#
# Logs go to ~/test_logs/<timestamp>/*.log
# Ctrl+C stops everything (pipeline, node, streamer, and unloads the module).
#
# ONE-TIME SETUP (avoids a sudo password prompt mid-run):
#   Add a NOPASSWD rule for modprobe/rmmod so this script can run unattended:
#     sudo visudo -f /etc/sudoers.d/v4l2loopback
#   and put in it (replace <user>):
#     <user> ALL=(root) NOPASSWD: /sbin/modprobe v4l2loopback*, /sbin/rmmod v4l2loopback
#   Without this, the script will just prompt you for your password once at start.

set -o pipefail

ROS_DOMAIN_ID_VAL=27
ROS2_WS="$HOME/ros2_ws"
DRONE_WS="$HOME/indomitus-drone"

LOG_DIR="$HOME/test_logs/$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"

PIPELINE_PID=""
NODE_PID=""
STREAMER_PID=""

cleanup() {
  echo ""
  echo "Shutting down..."

  for pid in "$STREAMER_PID" "$NODE_PID" "$PIPELINE_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -TERM "-$pid" 2>/dev/null # kill whole process group (setsid)
    fi
  done
  sleep 1
  for pid in "$STREAMER_PID" "$NODE_PID" "$PIPELINE_PID"; do
    if [[ -n "$pid" ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "-$pid" 2>/dev/null
    fi
  done

  sudo rmmod v4l2loopback 2>/dev/null
  echo "Done. Logs kept at $LOG_DIR"
}
trap cleanup EXIT INT TERM

# --- cache sudo credentials up front so it doesn't interrupt later ---
sudo -v

# --- 1. camera loopback + streaming pipeline ---
echo "[1/3] Loading v4l2loopback and starting camera pipeline..."
if lsmod | grep -q '^v4l2loopback'; then
  sudo rmmod v4l2loopback 2>/dev/null
fi
sudo modprobe v4l2loopback video_nr=10,11 \
  card_label="SimulinkCam","WebStreamCam" exclusive_caps=1,1

setsid bash -c '
    rpicam-vid --width 640 --height 480 --framerate 15 \
        --codec yuv420 --output - --timeout 0 | \
    gst-launch-1.0 fdsrc ! \
        rawvideoparse width=640 height=480 format=i420 framerate=15/1 ! \
        videoconvert ! video/x-raw,format=YUY2 ! tee name=t \
        t. ! queue ! v4l2sink device=/dev/video10 \
        t. ! queue ! v4l2sink device=/dev/video11
' >"$LOG_DIR/camera_pipeline.log" 2>&1 &
PIPELINE_PID=$!

echo "Waiting for /dev/video10 and /dev/video11..."
for _ in $(seq 1 20); do
  [[ -e /dev/video10 && -e /dev/video11 ]] && break
  sleep 0.5
done
if [[ ! -e /dev/video10 || ! -e /dev/video11 ]]; then
  echo "ERROR: loopback devices never appeared. Check $LOG_DIR/camera_pipeline.log"
  exit 1
fi
sleep 2 # let frames actually start flowing before consumers attach

# --- 2. vision node ---
echo "[2/3] Starting erc_vision_node..."
setsid bash -c "
    cd '$ROS2_WS' &&
    source /opt/ros/jazzy/setup.bash &&
    source install/setup.bash &&
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VAL &&
    exec ros2 run erc_vision_node erc_vision_node
" >"$LOG_DIR/vision_node.log" 2>&1 &
NODE_PID=$!

# --- 3. web streamer ---
echo "[3/3] Starting vision_web_streamer..."
setsid bash -c "
    cd '$DRONE_WS' &&
    source /opt/ros/jazzy/setup.bash &&
    source .venv/bin/activate &&
    export ROS_DOMAIN_ID=$ROS_DOMAIN_ID_VAL &&
    exec python3 scripts/vision_web_streamer.py
" >"$LOG_DIR/web_streamer.log" 2>&1 &
STREAMER_PID=$!

echo ""
echo "All three running. PIDs: pipeline=$PIPELINE_PID node=$NODE_PID streamer=$STREAMER_PID"
echo "Logs: $LOG_DIR"
echo "  tail -f $LOG_DIR/*.log"
echo "Press Ctrl+C to stop everything cleanly."
echo ""

wait
