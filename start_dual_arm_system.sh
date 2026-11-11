#!/bin/bash
# ==============================================================================
# 203实验室 PIPER 双臂视觉协同系统 · 全链路一键启动脚本
# 用法: bash /home/lab203/piper_students/lq/real_arm_control/start_dual_arm_system.sh
# 功能:
#   1. 一键拉起 RealSense D435 深度相机驱动
#   2. 一键拉起双臂统一 200Hz Action Server (piper_dual_arm_server.py)
#   3. 同时启动:
#      - 端口 8080: 完整双臂协同与标定总控台 (internal/dual)
#      - 端口 8081: 纯单臂抓取与标定演示台 (advisor/single)
# ==============================================================================

source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
export PYTHONUNBUFFERED=1

WORK_DIR="/home/lab203/piper_students/lq/real_arm_control"
cd "$WORK_DIR"

echo "================================================================="
echo "🚀 正在一键启动 PIPER 视觉协同全套系统..."
echo "================================================================="

# 1. 启动/重启 RealSense 相机
echo "[1/4] 📷 启动 RealSense D435 驱动..."
bash "$WORK_DIR/start_realsense.sh"

# 2. 启动双臂统一控制节点
echo "[2/4] 🦾 启动 200Hz 运控与 CAN 热管理服务..."
pkill -9 -f piper_trajectory_action_server.py 2>/dev/null || true
pkill -9 -f piper_dual_arm_server.py 2>/dev/null || true
sleep 1
nohup python3 "$WORK_DIR/piper_dual_arm_server.py" > "$WORK_DIR/dual_arm_server.log" 2>&1 < /dev/null &
echo $! > "$WORK_DIR/dual_arm_server.pid"
sleep 1

# 清理所有旧 Web 控制台进程
fuser -k 8080/tcp 2>/dev/null || true
fuser -k 8081/tcp 2>/dev/null || true
pkill -9 -f calibration_visualizer_server.py 2>/dev/null || true
sleep 1

# 3. 启动双臂研发总控台 (端口 8080)
echo "[3/4] 🌐 启动双臂协同与标定总控台 (端口 8080)..."
nohup python3 "$WORK_DIR/calibration_visualizer_server.py" --mode dual --port 8080 > "$WORK_DIR/visualizer.log" 2>&1 < /dev/null &
echo $! > "$WORK_DIR/visualizer.pid"

# 4. 启动单臂演示网页 (端口 8081)
echo "[4/4] 🎓 启动纯单臂视觉控制演示网页 (端口 8081)..."
nohup python3 "$WORK_DIR/calibration_visualizer_server.py" --mode single --port 8081 --arm arm_a > "$WORK_DIR/visualizer_single.log" 2>&1 < /dev/null &
echo $! > "$WORK_DIR/visualizer_single.pid"
sleep 2

echo "================================================================="
echo "✅ 全套系统启动完毕！"
echo "🎓 单臂导师演示版: http://192.168.31.70:8081"
echo "🔬 双臂科研协同版: http://192.168.31.70:8080"
echo "================================================================="
ip -br link show type can
