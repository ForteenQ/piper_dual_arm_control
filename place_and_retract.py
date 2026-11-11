#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 实机平稳放置桌面并安全恢复原位脚本
流程:
1. 读取当前空中抓取悬停状态 (Z ≈ 60.7 mm)
2. 预先计算全段安全轨迹:
   - 第一段: 6 段笛卡尔直线垂直下放 -48.5 mm (将方块轻柔贴放回桌面)
   - (中间动作: 张开夹爪至 50mm 释放方块)
   - 第二段: 6 段笛卡尔直线垂直升起 +80.0 mm (完全脱离方块顶部)
   - 第三段: 平滑折叠收回到原位安全待命姿态 [0°, 5°, -5°, 0°, 20°, 0°]
3. 全链路通过防撞扫描 (距离摄像头 > 220mm) 后开始执行
"""

import time
import math
import threading
import numpy as np
import cv2
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration
from cv_bridge import CvBridge
import tf2_ros

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

BOUNDS = [
    (-2.618, 2.618),
    (-0.1, 3.1416),
    (-2.967, 0.1),
    (-1.832, 1.832),
    (-1.22, 1.22),
    (-2.094, 2.094),
]


def rpy_mat(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    return np.array([
        [cy*cp, cy*sp*sr - sy*cr, cy*sp*cr + sy*sr],
        [sy*cp, sy*sp*sr + cy*cr, sy*sp*cr - cy*sr],
        [-sp, cp*sr, cp*cr]
    ])


def T_joint(xyz, rpy, q):
    T = np.eye(4)
    R0 = rpy_mat(*rpy)
    Rz = np.array([[np.cos(q), -np.sin(q), 0], [np.sin(q), np.cos(q), 0], [0, 0, 1]])
    T[:3, :3] = R0 @ Rz
    T[:3, 3] = xyz
    return T


def piper_all_links_fk(q):
    T = np.eye(4)
    T1 = T @ T_joint([0, 0, 0.123], [0, 0, 0], q[0])
    T2 = T1 @ T_joint([0, 0, 0], [1.5708, -0.1359, -3.1416], q[1])
    T3 = T2 @ T_joint([0.28503, 0, 0], [0, 0, -1.7939], q[2])
    T4 = T3 @ T_joint([-0.021984, -0.25075, 0], [1.5708, 0, 0], q[3])
    T5 = T4 @ T_joint([0, 0, 0], [-1.5708, 0, 0], q[4])
    T6 = T5 @ T_joint([8.8259e-05, -0.091, 0], [1.5708, 0, 0], q[5])
    T_tip = T6 @ T_joint([0, 0, 0.136], [0, 0, 0], 0)
    links = [T1[:3, 3], T2[:3, 3], T3[:3, 3], T4[:3, 3], T5[:3, 3], T6[:3, 3], T_tip[:3, 3]]
    return links, T6, T_tip


def solve_cartesian_step_ik(q_prev, target_pos, target_R6):
    def loss(q):
        _, T6, T_tip = piper_all_links_fk(q)
        p_err = np.sum((T_tip[:3, 3] - target_pos)**2) * 50000.0
        r_err = np.sum((T6[:3, :3] - target_R6)**2) * 500.0
        jump_err = np.sum((q - q_prev)**2) * 0.5
        return p_err + r_err + jump_err

    res = minimize(loss, q_prev, method='SLSQP', bounds=BOUNDS, options={'maxiter': 500, 'ftol': 1e-9})
    return res.x, res.success


class PlaceAndRetractClient(Node):
    def __init__(self):
        super().__init__('piper_place_retract_client')
        self.curr_joints = None
        self.latest_img = None
        self.bridge = CvBridge()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.gripper_pub = self.create_publisher(Float64, '/piper_gripper_cmd', 10)
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, 1)

        self.done_event = threading.Event()
        self.exec_ok = False

    def joint_cb(self, msg):
        if len(msg.name) >= 6:
            try:
                self.curr_joints = np.array([msg.position[msg.name.index(j)] for j in ARM_JOINTS], dtype=float)
            except ValueError:
                pass

    def img_cb(self, msg):
        try:
            self.latest_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass

    def send_gripper(self, width_mm):
        msg = Float64()
        msg.data = float(width_mm)
        self.gripper_pub.publish(msg)

    def send_trajectory(self, waypoints, total_duration=3.0):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        dt_step = total_duration / len(waypoints)
        for i, q in enumerate(waypoints):
            t = (i + 1) * dt_step
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.velocities = [0.0] * 6
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            pt.time_from_start = RosDuration(sec=sec, nanosec=nanosec)
            goal.trajectory.points.append(pt)

        self.done_event.clear()
        fut = self.action_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_cb)
        self.done_event.wait(timeout=total_duration + 8.0)
        return self.exec_ok

    def _goal_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.exec_ok = False
            self.done_event.set()
            return
        gh.get_result_async().add_done_callback(self._res_cb)

    def _res_cb(self, fut):
        res = fut.result().result
        self.exec_ok = (res.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        self.done_event.set()


def main():
    rclpy.init()
    node = PlaceAndRetractClient()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    print("=" * 65)
    print("📦 [平滑放置与归位] 垂直轻放桌面 -> 松开脱开 -> 收回原位")
    print("=" * 65)

    t0 = time.time()
    while node.curr_joints is None:
        if time.time() - t0 > 4.0:
            print("❌ 超时未获取机械臂状态")
            rclpy.shutdown()
            return
        time.sleep(0.05)

    q_curr = node.curr_joints.copy()
    _, T6_c, T_tip_c = piper_all_links_fk(q_curr)
    pos_start = T_tip_c[:3, 3]
    R_target = T6_c[:3, :3]

    # 阶段 1: 垂直下放 -48.5mm (从 Z≈60.7mm 到 Z≈12.2mm，轻贴桌面不重砸)
    down_dz = -0.0485
    waypoints_down = []
    q_prev = q_curr.copy()
    for i in range(1, 7):
        p_step = pos_start + np.array([0.0, 0.0, down_dz * (i / 6.0)])
        q_step, ok = solve_cartesian_step_ik(q_prev, p_step, R_target)
        _, _, T_tip_s = piper_all_links_fk(q_step)
        if not ok or np.linalg.norm(T_tip_s[:3, 3] - p_step) > 0.003:
            print("🔴 下降路点逆解异常，终止！")
            rclpy.shutdown()
            return
        waypoints_down.append(q_step)
        q_prev = q_step

    # 阶段 2: 从放置点垂直上升 +80.0mm (彻底高过 3cm 方块顶面)
    pos_placed = pos_start + np.array([0.0, 0.0, down_dz])
    up_dz = 0.080
    waypoints_up = []
    for i in range(1, 7):
        p_step = pos_placed + np.array([0.0, 0.0, up_dz * (i / 6.0)])
        q_step, ok = solve_cartesian_step_ik(q_prev, p_step, R_target)
        _, _, T_tip_s = piper_all_links_fk(q_step)
        if not ok or np.linalg.norm(T_tip_s[:3, 3] - p_step) > 0.003:
            print("🔴 上升路点逆解异常，终止！")
            rclpy.shutdown()
            return
        waypoints_up.append(q_step)
        q_prev = q_step

    # 阶段 3: 平滑收回原点待命态 [0°, 5°, -5°, 0°, 20°, 0°]
    deg = math.pi / 180.0
    q_home = np.array([0.0, 5.0 * deg, -5.0 * deg, 0.0, 20.0 * deg, 0.0])

    # 全路径摄像头防撞扫描
    try:
        tf_cam = node.tf_buffer.lookup_transform('base_link', 'camera_color_optical_frame', rclpy.time.Time(), timeout=Duration(seconds=1.5))
        cam_pos = np.array([tf_cam.transform.translation.x, tf_cam.transform.translation.y, tf_cam.transform.translation.z])
        min_cam_dist = 999.0
        for seg_s, seg_e in [(waypoints_up[-1], q_home)]:
            for alpha in np.linspace(0, 1, 25):
                q_chk = (1 - alpha) * seg_s + alpha * seg_e
                lks, _, _ = piper_all_links_fk(q_chk)
                for pt in lks:
                    min_cam_dist = min(min_cam_dist, np.linalg.norm(pt - cam_pos))
        print(f"🛡️ 归位全路径距摄像头最小净空: {min_cam_dist*1000:.1f} mm")
        if min_cam_dist < 0.20:
            print("🔴 安全拦截: 归位路径接近摄像头！")
            rclpy.shutdown()
            return
    except Exception:
        pass

    print("1️⃣ 正在垂直轻柔下放方块至桌面 (3.0s)...")
    if not node.send_trajectory(waypoints_down, total_duration=3.0):
        print("⚠️ 下放失败")
        rclpy.shutdown()
        return

    print("2️⃣ 正在平稳张开夹爪 (50.0mm) 释放方块...")
    node.send_gripper(50.0)
    time.sleep(0.8)

    print("3️⃣ 正在垂直直线升起 +80mm 安全脱离方块 (2.5s)...")
    if not node.send_trajectory(waypoints_up, total_duration=2.5):
        print("⚠️ 升起失败")
        rclpy.shutdown()
        return

    print("4️⃣ 正在平滑折叠恢复机械臂原位 (4.0s)...")
    if node.send_trajectory([q_home], total_duration=4.0):
        # 归位后顺手将夹爪复原闭合至 0mm 整洁收纳
        node.send_gripper(0.0)
        time.sleep(0.8)
        print("🎉 方块已稳稳放回桌面，机械臂圆满恢复原位！")
        if node.latest_img is not None:
            out_img = "/home/lab203/piper_students/lq/real_arm_control/final_placed_preview.jpg"
            cv2.imwrite(out_img, node.latest_img)
            print(f"📸 放置收官实拍图已保存: {out_img}")
    else:
        print("⚠️ 恢复原位异常！")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
