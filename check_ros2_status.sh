#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

echo "=== ROS 2 NODES ==="
ros2 node list

echo "=== ROS 2 ACTIONS ==="
ros2 action list

echo "=== ROS 2 TOPICS ==="
ros2 topic list
