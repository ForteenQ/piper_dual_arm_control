#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lab203/piper_students/lq/real_arm_control/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export DISPLAY=${DISPLAY:-:0}

cd /home/lab203/piper_students/lq/real_arm_control

ros2 run easy_handeye2 rqt_calibrator.py --ros-args \
  -p name:=piper_d435_eye_to_hand \
  -p calibration_type:=eye_on_base \
  -p robot_base_frame:=base_link \
  -p robot_effector_frame:=link6 \
  -p tracking_base_frame:=camera_color_optical_frame \
  -p tracking_marker_frame:=checkerboard \
  -p freehand_robot_movement:=true
