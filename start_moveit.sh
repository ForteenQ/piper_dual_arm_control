#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

cd /home/lab203/piper_students/lq/real_arm_control/
if [ -f moveit.pid ]; then
    PID=$(cat moveit.pid)
    if kill -0 $PID 2>/dev/null; then
        echo "MoveIt already running with PID $PID"
        exit 0
    fi
fi

nohup ros2 launch real_arm_moveit.launch.py > moveit.log 2>&1 &
echo $! > moveit.pid
sleep 2
echo "MoveIt started with PID: $(cat moveit.pid)"
