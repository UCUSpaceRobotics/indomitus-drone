#!/usr/bin/env bash
# One-command mission launcher for ERC 2026 Droning Sub-Task.
# Usage: ./scripts/start_mission.sh

set -euo pipefail

echo "🛩️  Indomitus Drone — Starting Mission..."

# Source ROS 2 environment
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Set ROS domain to avoid cross-talk with other teams
export ROS_DOMAIN_ID=42

# Launch the main mission script with the venv's Python.
# We must use the full path because sudo resolves python3 from root's PATH,
# which skips the venv and misses rclpy.
cd "$(dirname "$0")/.."
source .venv/bin/activate
exec sudo -E .venv/bin/python3 main.py "$@"