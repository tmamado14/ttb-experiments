#!/bin/bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=203
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=LDS-01
pkill -f robot.launch.py 2>/dev/null; pkill -f image_publisher 2>/dev/null; sleep 2
nohup setsid ros2 launch turtlebot3_bringup robot.launch.py > ~/bringup.log 2>&1 < /dev/null &
sleep 4
nohup setsid ros2 run turtlebot3_image_motion image_publisher > ~/camera.log 2>&1 < /dev/null &
