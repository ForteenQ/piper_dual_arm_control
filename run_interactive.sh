#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lab203/piper_students/lq/real_arm_control/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

cd /home/lab203/piper_students/lq/real_arm_control
python3 calibrate_easy_handeye2_interactive.py
