#!/usr/bin/env bash
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export DISPLAY=:0
export XAUTHORITY=/run/user/1000/.mutter-Xwaylandauth.5F0GV3
export QT_LOGGING_RULES="qt.qpa.*=true"
export QT_X11_NO_MITSHM=1

rviz2 --help-all 2>&1 | head -n 30
