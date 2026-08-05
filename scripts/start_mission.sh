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

# Launch the main mission script
cd "$(dirname "$0")/.."
exec sudo -E python3 main.py "$@"