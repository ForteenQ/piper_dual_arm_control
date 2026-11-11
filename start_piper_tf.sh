#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

echo "============================================================"
echo "🚀 启动 PIPER 机械臂驱动与 TF 坐标广播 (Domain 42)..."
echo "============================================================"
ros2 launch piper start_single_piper_rviz.launch.py can_port:=can0 auto_enable:=true rviz_ctrl_flag:=false
