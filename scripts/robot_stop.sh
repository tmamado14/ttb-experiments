#!/usr/bin/env bash
# Stop the robot-side ROS 2 nodes (does NOT power off the robot).
#   Usage:  ./scripts/robot_stop.sh
set -euo pipefail
cd "$(dirname "$0")/.."
set -a; source .env; set +a

REMOTE_CMD='pkill -9 -f robot.launch.py; pkill -9 -f turtlebot3_ros;
pkill -9 -f image_publisher; pkill -9 -f robot_state_publisher;
pkill -9 -f "ros2 launch"; pkill -9 -f "ros2 run"; sleep 2;
pgrep -af "turtlebot3|image_publisher|robot_state" | grep -v pgrep || echo "ALL ROS NODES STOPPED"'

if command -v sshpass >/dev/null 2>&1; then
  sshpass -p "${ROBOT_PASSWORD}" ssh -o StrictHostKeyChecking=accept-new \
    "${ROBOT_USER}@${ROBOT_IP}" "${REMOTE_CMD}" || true
else
  ssh -o StrictHostKeyChecking=accept-new "${ROBOT_USER}@${ROBOT_IP}" "${REMOTE_CMD}" || true
fi
