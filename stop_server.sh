#!/bin/bash
cd /home/lab203/piper_students/lq/real_arm_control/
if [ -f action_server.pid ]; then
    PID=$(cat action_server.pid)
    if kill -0 $PID 2>/dev/null; then
        echo "Stopping Action Server PID $PID..."
        kill -INT $PID 2>/dev/null || true
        sleep 1
        kill -9 $PID 2>/dev/null || true
    fi
    rm -f action_server.pid
    echo "Action Server stopped."
else
    echo "No action_server.pid found."
fi
