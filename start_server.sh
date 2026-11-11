#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

cd /home/lab203/piper_students/lq/real_arm_control/

# 如果已在运行则不重复启动
if [ -f action_server.pid ]; then
    PID=$(cat action_server.pid)
    if kill -0 $PID 2>/dev/null; then
        echo "Action Server already running with PID $PID"
        exit 0
    fi
fi

nohup python3 piper_trajectory_action_server.py > action_server.log 2>&1 &
echo $! > action_server.pid
sleep 1
echo "Action Server successfully started with PID: $(cat action_server.pid)"
