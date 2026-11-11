#!/bin/bash
# ==============================================================================
# 203实验室 PIPER 双臂视觉协同系统 · 一键安全关闭脚本
# 用法: bash /home/lab203/piper_students/lq/real_arm_control/stop_dual_arm_system.sh
# 注意: 运行本脚本前，请务必先在网页点击【🏠 双臂同步安全回原点】，让机械臂安全折叠落座！
# ==============================================================================

WORK_DIR="/home/lab203/piper_students/lq/real_arm_control"
cd "$WORK_DIR"

echo "================================================================="
echo "🛑 正在安全停止 PIPER 双臂视觉协同全套后台服务..."
echo "================================================================="

# 1. 停止双臂与单臂 Web 控制台 (8080 / 8081)
echo "[1/3] 🌐 关闭 Web 控制台 (端口 8080 & 8081)..."
pkill -INT -f calibration_visualizer_server.py 2>/dev/null || true
sleep 1
pkill -9 -f calibration_visualizer_server.py 2>/dev/null || true
fuser -k 8080/tcp 2>/dev/null || true
fuser -k 8081/tcp 2>/dev/null || true
rm -f "$WORK_DIR/visualizer.pid" "$WORK_DIR/visualizer_single.pid"

# 2. 停止双臂 200Hz CAN 运控服务
echo "[2/3] 🦾 停止双臂 200Hz CAN 运控服务..."
pkill -INT -f piper_dual_arm_server.py 2>/dev/null || true
sleep 1
pkill -9 -f piper_dual_arm_server.py 2>/dev/null || true
rm -f "$WORK_DIR/dual_arm_server.pid"

# 3. 停止 RealSense 相机驱动
echo "[3/3] 📷 关闭 RealSense D435 驱动..."
bash "$WORK_DIR/stop_realsense.sh" 2>/dev/null || true
pkill -INT -f realsense2_camera_node 2>/dev/null || true
sleep 1
pkill -9 -f realsense2_camera_node 2>/dev/null || true

echo "================================================================="
echo "✅ 所有上位机视觉与控制后台进程已彻底停止！"
echo "⚡ 现在可以安全切断两台机械臂的 24V 电源并拔下 USB 拓展坞。"
echo "================================================================="
