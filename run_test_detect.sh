#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
cd /home/lab203/piper_students/lq/real_arm_control/
python3 test_detect_checkerboard.py
