#!/usr/bin/env bash
# 自动生成的 RealSense 相机到 PIPER 机械臂基座静态 TF 广播
# base_link -> camera_color_optical_frame
source /opt/ros/humble/setup.bash
echo "🚀 正在广播静态 TF: base_link -> camera_color_optical_frame"
ros2 run tf2_ros static_transform_publisher \
    --x 0.246391 --y 0.438633 --z -0.289591 \
    --qx 0.061937 --qy -0.064934 --qz 0.262943 --qw 0.960629 \
    --frame-id base_link \
    --child-frame-id camera_color_optical_frame
