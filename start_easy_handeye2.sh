#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lab203/piper_students/lq/real_arm_control/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

cd /home/lab203/piper_students/lq/real_arm_control

# 停止旧的实例
pkill -f checkerboard_tf_broadcaster.py 2>/dev/null || true
pkill -f "handeye_server" 2>/dev/null || true
sleep 1

# 1. 启动标定板 TF 广播节点
nohup python3 checkerboard_tf_broadcaster.py > /tmp/checkerboard_tf.log 2>&1 &
echo $! > /tmp/checkerboard_tf.pid

# 2. 启动 easy_handeye2 服务端核心
nohup ros2 run easy_handeye2 handeye_server --ros-args \
  -p name:=piper_d435_eye_to_hand \
  -p calibration_type:=eye_on_base \
  -p robot_base_frame:=base_link \
  -p robot_effector_frame:=link6 \
  -p tracking_base_frame:=camera_color_optical_frame \
  -p tracking_marker_frame:=checkerboard \
  -p freehand_robot_movement:=true > /tmp/easy_handeye2_server.log 2>&1 &
echo $! > /tmp/easy_handeye2_server.pid

sleep 2
echo "easy_handeye2 标定系统启动成功！"
echo "TF 广播 PID: $(cat /tmp/checkerboard_tf.pid)"
echo "标定服务 PID: $(cat /tmp/easy_handeye2_server.pid)"
