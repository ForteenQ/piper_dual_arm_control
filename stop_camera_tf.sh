#!/usr/bin/env bash
pkill -9 -f "static_transform_publisher.*camera_color_optical_frame" 2>/dev/null || true
rm -f /home/lab203/piper_students/lq/real_arm_control/camera_tf.pid
echo "🛑 静态 TF 广播已停止"
