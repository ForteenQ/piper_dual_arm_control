#!/bin/bash
# 执行 PIPER ROS 2 轨迹动作客户端测试
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

cd /home/lab203/piper_students/lq/real_arm_control/
exec python3 test_trajectory_action_client.py
