#!/bin/bash
# ==============================================================================
# RealSense D435 ROS 2 驱动守护启动脚本 (DOMAIN_ID=42)
# 开启色彩流、深度流与对齐深度流 (align_depth.enable:=true)
# ==============================================================================
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

cd /home/lab203/piper_students/lq/real_arm_control/

echo "🧹 正在清理旧的 RealSense 节点..."
pkill -9 -f realsense2_camera_node 2>/dev/null || true
pkill -9 -f rs_launch.py 2>/dev/null || true
sleep 2

echo "🚀 启动 RealSense D435 驱动 (RGB 640x480, 对齐深度使能)..."
nohup ros2 launch realsense2_camera rs_launch.py \
    enable_color:=true \
    enable_depth:=true \
    align_depth.enable:=true \
    pointcloud.enable:=false > realsense.log 2>&1 < /dev/null &

PID=$!
echo "${PID}" > realsense.pid
sleep 3

if kill -0 "${PID}" 2>/dev/null; then
    echo "✅ RealSense 驱动已启动成功 (PID: ${PID}, DOMAIN_ID=42)"
else
    echo "❌ RealSense 启动失败，查看日志:"
    cat realsense.log
fi
