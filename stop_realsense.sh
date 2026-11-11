#!/bin/bash
cd /home/lab203/piper_students/lq/real_arm_control/
if [ -f realsense.pid ]; then
    PID=$(cat realsense.pid)
    if kill -0 $PID 2>/dev/null; then
        echo "Stopping RealSense PID $PID..."
        kill -INT $PID 2>/dev/null || true
        sleep 2
        kill -9 $PID 2>/dev/null || true
    fi
    rm -f realsense.pid
    echo "RealSense stopped."
fi
