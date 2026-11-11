#!/bin/bash
cd /home/lab203/piper_students/lq/real_arm_control/
if [ -f moveit.pid ]; then
    PID=$(cat moveit.pid)
    if kill -0 $PID 2>/dev/null; then
        echo "Stopping MoveIt PID $PID..."
        kill -INT $PID 2>/dev/null || true
        sleep 2
        kill -9 $PID 2>/dev/null || true
    fi
    rm -f moveit.pid
    echo "MoveIt stopped."
else
    echo "No moveit.pid found."
fi
