#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

cd /home/lab203/piper_students/lq/real_arm_control/

# 彻底释放 8080 端口与旧进程
fuser -k 8080/tcp 2>/dev/null || true
pkill -9 -f calibration_visualizer_server.py 2>/dev/null || true
sleep 1

nohup python3 calibration_visualizer_server.py > visualizer.log 2>&1 < /dev/null &
echo $! > visualizer.pid
sleep 2
echo "Visualizer started with PID: $(cat visualizer.pid)"
