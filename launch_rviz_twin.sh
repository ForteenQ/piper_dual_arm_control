#!/usr/bin/env bash

# 1. 继承桌面会话变量
for var in $(systemctl --user show-environment); do
    export "$var"
done

# 2. 加载 ROS 2 环境与 PIPER 工作空间（在 export 之后，保证 PATH 包含 ros2）
source /opt/ros/humble/setup.bash
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash

export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1

echo "当前图形环境: DISPLAY=$DISPLAY, WAYLAND=$WAYLAND_DISPLAY, XAUTHORITY=$XAUTHORITY"

# 3. 启动 robot_state_publisher（将 /joint_states 映射至真机 200Hz 实测反馈 /joint_states_feedback）
if ! pgrep -f "robot_state_publisher.*joint_states_feedback" > /dev/null; then
    echo "启动 robot_state_publisher..."
    ROBOT_DESC=$(xacro /home/lab203/Desktop/piper_ros-humble/src/piper_description/urdf/piper_description.xacro)
    ros2 run robot_state_publisher robot_state_publisher \
        --ros-args \
        -p robot_description:="$ROBOT_DESC" \
        -r joint_states:=/joint_states_feedback \
        > /tmp/rsp_twin.log 2>&1 &
    echo $! > /tmp/rsp_twin.pid
    sleep 1
else
    echo "robot_state_publisher 已在运行中。"
fi

# 4. 启动 RViz2
RVIZ_CONFIG=/home/lab203/Desktop/piper_ros-humble/src/piper_description/rviz/piper_ctrl.rviz
echo "启动 RViz2 数字孪生窗口..."
export QT_QPA_PLATFORM=wayland
ros2 run rviz2 rviz2 -d "$RVIZ_CONFIG" > /tmp/rviz_twin.log 2>&1 &
echo $! > /tmp/rviz_twin.pid

sleep 3
if ps -p $(cat /tmp/rviz_twin.pid) > /dev/null; then
    echo "=== RViz2 数字孪生窗口成功在桌面显示！PID: $(cat /tmp/rviz_twin.pid) ==="
else
    echo "RViz2 退出日志："
    cat /tmp/rviz_twin.log
fi
