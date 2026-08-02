#!/usr/bin/env bash
# Start the robot-side ROS 2 nodes (motor bringup + camera publisher) over SSH.
# Reads credentials/config from ../.env. Uses sshpass if available, otherwise
# SSH will prompt for the password interactively.
#   Usage:  ./scripts/robot_start.sh
set -euo pipefail
cd "$(dirname "$0")/.."
set -a; source .env; set +a

REMOTE_CMD='cat > ~/start_all.sh <<EOF
#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID='"${ROS_DOMAIN_ID}"'
export TURTLEBOT3_MODEL='"${TURTLEBOT3_MODEL}"'
export LDS_MODEL='"${LDS_MODEL}"'
pkill -f robot.launch.py 2>/dev/null; pkill -f image_publisher 2>/dev/null; sleep 2
nohup setsid ros2 launch turtlebot3_bringup robot.launch.py > ~/bringup.log 2>&1 < /dev/null &
sleep 4
nohup setsid ros2 run turtlebot3_image_motion image_publisher > ~/camera.log 2>&1 < /dev/null &
EOF
chmod +x ~/start_all.sh
nohup setsid ~/start_all.sh >/dev/null 2>&1 </dev/null & disown
sleep 1; echo "robot nodes launching (domain '"${ROS_DOMAIN_ID}"')"'

if command -v sshpass >/dev/null 2>&1; then
  sshpass -p "${ROBOT_PASSWORD}" ssh -o StrictHostKeyChecking=accept-new \
    "${ROBOT_USER}@${ROBOT_IP}" "${REMOTE_CMD}"
else
  echo "(sshpass not installed - you'll be asked for the robot password)"
  ssh -o StrictHostKeyChecking=accept-new "${ROBOT_USER}@${ROBOT_IP}" "${REMOTE_CMD}"
fi

echo "Waiting for nodes to come up..."; sleep 12
echo "Done. Check with:  ROS_DOMAIN_ID=${ROS_DOMAIN_ID} ros2 topic list"
