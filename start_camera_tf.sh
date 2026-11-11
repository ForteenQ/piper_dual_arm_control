#!/usr/bin/env bash
# ==============================================================================
# RealSense 相机到 PIPER 机械臂基座静态 TF 广播守护启动脚本 (真值标定版)
# 标定 RMSE 误差: 21.37 mm
# 坐标映射: base_link -> camera_color_optical_frame
# ==============================================================================
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

WORK_DIR="/home/lab203/piper_students/lq/real_arm_control"
cd "${WORK_DIR}" || exit 1

pkill -9 -f "static_transform_publisher.*camera_color_optical_frame" 2>/dev/null || true
sleep 1

echo "🚀 [TF Broadcaster] 正在启动后台静态 TF 广播: base_link -> camera_color_optical_frame..."
nohup ros2 run tf2_ros static_transform_publisher \
    --x 0.209168 --y 0.304901 --z 0.579219 \
    --qx 0.711570 --qy 0.702479 --qz -0.006215 --qw 0.012368 \
    --frame-id base_link \
    --child-frame-id camera_color_optical_frame > camera_tf.log 2>&1 < /dev/null &

PID=$!
echo "${PID}" > camera_tf.pid
sleep 1
echo "✅ 静态 TF 广播已刷新并启动 (PID: ${PID})"
