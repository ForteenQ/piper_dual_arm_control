# 203 实验室「PIPER 机械臂 × RealSense D435 双臂项目」常用指令速查手册

> **更新时间**：2026-11-09  
> **工控机主机**：`lab203@192.168.31.70` (Ubuntu 22.04 LTS, ROS 2 Humble)  
> **本机工作目录**：`d:\研\ROS2\`  
> **工控机工作目录**：`/home/lab203/piper_students/lq/real_arm_control/`  
> **通信域标准**：`ROS_DOMAIN_ID=42`

---

## 目录
1. [基础环境与通信域初始化](#一基础环境与通信域初始化)
2. [视觉感知与 Web 实时监控](#二视觉感知与-web-实时监控)
3. [Gazebo 物理仿真与 MoveIt（数字孪生）](#三gazebo-物理仿真与-moveit数字孪生)
4. [真实 PIPER 机械臂控制与安全运维](#四真实-piper-机械臂控制与安全运维)
5. [ROS 2 系统诊断与排查常用命令](#五ros-2-系统诊断与排查常用命令)

---

## 一、基础环境与通信域初始化

打开工控机终端或新建 SSH 连接后，**必须首先载入 ROS 2 环境与通信域设置**，避免跨域冲突或无法发现节点：

```bash
# 1. 载入 ROS 2 Humble 官方基础环境
source /opt/ros/humble/setup.bash

# 2. 载入 PIPER 机械臂 ROS 2 驱动工作空间
source /home/lab203/Desktop/piper_ros-humble/install/setup.bash

# 3. 锁定项目通信域 ID 为 42 (实验室统一标准，避免局域网设备串号)
export ROS_DOMAIN_ID=42

# 4. 限制通信仅在本机回环 (避免局域网组播广播风暴)
export ROS_LOCALHOST_ONLY=1

# 5. 进入实机控制主工作目录
cd /home/lab203/piper_students/lq/real_arm_control/
```

> **一键极简载入**：
> 已预置一键环境脚本，新终端运行如下单行指令即可等效完成上述 1~4 步：
> ```bash
> source /home/lab203/piper_students/lq/downloads/remote_lq/env_lq.bash 42
> ```

---

## 二、视觉感知与 Web 实时监控

### 1. 启动 RealSense D435 深度相机
```bash
cd /home/lab203/piper_students/lq/real_arm_control/
./start_realsense.sh
```
- **功能说明**：清理旧相机进程，后台拉起 RealSense 驱动，启用 RGB 彩色流（640×480 @ 30fps）和深度对齐流（`align_depth.enable:=true`），PID 记录于 `realsense.pid`。
- **停止指令**：`./stop_realsense.sh`
- **日志查看**：`tail -f realsense.log`

---

### 2. 启动 60 FPS 极速超低延迟 Web 网页监视器 🌟
```bash
cd /home/lab203/piper_students/lq/real_arm_control/
./run_calibration_visualizer.sh
```
- **功能说明**：攻克高网络丢包卡顿的 **60 FPS MJPEG 直通流 Web 服务器**。释放 8080 端口并后台守护运行。
- **浏览器访问地址**：
  在实验室局域网内任意手机或电脑浏览器输入：
  👉 `http://192.168.31.70:8080`
  可实时查看极低延迟的摄像头画面、角点标靶与空间坐标。
- **查看运行日志**：`tail -f visualizer.log`

---

### 3. 启动机械臂基座到相机的静态 TF 广播
```bash
cd /home/lab203/piper_students/lq/real_arm_control/
./start_camera_tf.sh
```
- **功能说明**：将实测标定外参广播到 TF 树（`base_link -> camera_color_optical_frame`），是视觉坐标转机械臂抓取坐标的数学基准。
- **停止指令**：`./stop_camera_tf.sh`
- **查看实时变换数值**：
  ```bash
  ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
  ```
  *(正常输出：平移 X=0.246m, Y=0.439m, Z=-0.290m；四元数 [0.062, -0.065, 0.263, 0.961])*

---

### 4. 单次测试桌面 AprilTag 靶标识别与坐标解算
```bash
# 传入贴纸实际物理边长 (毫米)，例如 25
python3 /home/lab203/piper_students/lq/real_arm_control/test_detect_tag.py 25
```
- **功能说明**：从相机截取一帧，解算 AprilTag 在相机系和机械臂基座系下的 `(X, Y, Z)` 绝对坐标（mm）与偏航角，判定可达性，并生成渲染图片 `tag_detected_preview.jpg`。

---

### 5. 持续目标追踪与 TF 广播节点
```bash
python3 /home/lab203/piper_students/lq/real_arm_control/apriltag_pose_estimator.py
```
- **功能说明**：持续高频检测 AprilTag，输出平滑滤波后的 `/detected_object_pose` 话题并在 RViz 中广播 `detected_object` 坐标系。

---

## 三、Gazebo 物理仿真与 MoveIt（数字孪生）

用于非实机状态下的运动学推演、避障与轨迹算法验证：

### 1. 启动 Gazebo 无头仿真后端 + MoveIt 核心
```bash
cd /home/lab203/piper_students/lq/downloads/piper_grasp_scene_lq/
./launch_backend.sh
```
- **功能说明**：启动 Gazebo 物理仿真引擎（加载抓取场景 `piper_grasp_lq.world`）与 MoveIt `move_group` 服务。采用无头模式（`gui:=false`），极大节约 CPU 和内存。

---

### 2. 打开 Gazebo 三维渲染客户端（可视化窗口）
```bash
cd /home/lab203/piper_students/lq/downloads/piper_grasp_scene_lq/
./launch_gzclient.sh
```
- **功能说明**：通过 SSH X11 转发打开 Gazebo 渲染窗口，直观查看机械臂模型、两指夹爪与抓取物体。

---

### 3. 打开 MoveIt RViz 交互式运动规划界面
```bash
cd /home/lab203/piper_students/lq/downloads/piper_grasp_scene_lq/
./launch_rviz.sh
```
- **功能说明**：打开 RViz 可视化界面，拖动机械臂末端交互球（Interactive Marker），手动指定位姿并点击 "Plan & Execute" 预览规划轨迹。

---

### 4. 一键清理所有仿真后台进程
```bash
pkill -9 gzserver; pkill -9 gzclient; pkill -9 move_group
```
- **功能说明**：强制杀死残留的 Gazebo 和 MoveIt 节点，防止后台多开导致系统卡顿。

---

## 四、真实 PIPER 机械臂控制与安全运维

### 1. 快速读取机械臂底层物理状态
```bash
python3 /home/lab203/piper_students/lq/real_arm_control/quick_read_state.py
```
- **功能说明**：通过 CAN 总线（1Mbps can0）读取机械臂固件寄存器，显示当前控制模式（`CAN_CONTROL` 还是 `TEACHING_MODE`）、使能状态与各关节实际物理角度。

---

### 2. 退出示教模式，重新切回 CAN 自动控制模式
```bash
python3 /home/lab203/piper_students/lq/real_arm_control/exit_teaching_mode.py
```
- **功能说明**：向 0x150 和 0x151 发送控制帧，退出拖动示教，切回自动 CAN 控制模式并上电闭环锁定。

---

### 3. 启动实机 200Hz 轨迹动作服务器（Action Server）
```bash
cd /home/lab203/piper_students/lq/real_arm_control/
./start_server.sh
```
- **功能说明**：后台拉起 `piper_trajectory_action_server.py`，对外暴露标准的 `/arm_controller/follow_joint_trajectory` 动作接口，进行 200Hz 五次多项式 S 曲线平滑插值驱动实机。
- **停止指令**：`./stop_server.sh`
- **日志查看**：`tail -f action_server.log`

---

### 4. 安全展开至工作就绪姿态（Ready Pose）
```bash
cd /home/lab203/piper_students/lq/real_arm_control/
./run_ready_pose_unfold.sh
```
- **功能说明**：以 8 秒慢速极平滑 S 曲线，将机械臂从零位折叠状态展开至高位待命姿态（大臂微扬，小臂前伸，避开相机视野）。

---

### 5. 夹爪独立开合与力矩安全测试
```bash
python3 /home/lab203/piper_students/lq/real_arm_control/safe_gripper_motion_isolated.py
```
- **功能说明**：在机械臂 6 轴绝对静止锁定的前提下，单独使能两指夹爪，测试开合（张开 50mm，闭合 20mm）与力矩反馈。

---

## 五、ROS 2 系统诊断与排查常用命令

```bash
# 1. 列出当前所有活动的 ROS 2 节点
ros2 node list

# 2. 列出当前所有话题
ros2 topic list

# 3. 实时查看相机彩色图像的实际发布频率 (Hz)
ros2 topic hz /camera/camera/color/image_raw

# 4. 实时查看机械臂当前 200Hz 真实关节反馈 (截断打印)
ros2 topic echo /joint_states --truncate

# 5. 查看系统 TF 坐标变换树 PDF 结构图
ros2 run tf2_tools view_frames

# 6. 查看 8080 端口占用情况
fuser 8080/tcp
```
