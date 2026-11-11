#!/bin/bash
set -e
source /opt/ros/humble/setup.bash
cd /home/lab203/piper_students/lq/real_arm_control

colcon build --packages-select easy_handeye2_msgs easy_handeye2
echo "BUILD_COMPLETE_SUCCESS"
