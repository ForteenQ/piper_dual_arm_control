#!/usr/bin/env bash
set -e
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
exec ros2 launch piper start_single_piper.launch.py can_port:=can0 auto_enable:=false
