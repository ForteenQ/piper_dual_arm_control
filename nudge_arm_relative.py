#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 末端空间笛卡尔微调移动脚本 (保持姿态与高度不变，局部精细平滑微调)
用法: python3 nudge_arm_relative.py <dx_mm> <dy_mm> <dz_mm>
示例: python3 nudge_arm_relative.py 0 3 0  (沿基座 +Y 方向精准微调移动 3mm)
"""

import sys
import time
import math
import threading
import numpy as np
import cv2
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState, Image
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration
from cv_bridge import CvBridge

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

BOUNDS = [
    (-2.618, 2.618),
    (0.0, 3.1416),
    (-2.967, 0.0),
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


def solve_local_nudge_ik(q_curr, target_tip_pos, target_R6):
    """以 q_curr 为初值求解保持末端姿态不变的局部精确逆解"""
    def loss(q):
        _, T6, T_tip = piper_all_links_fk(q)
        pos_err = np.sum((T_tip[:3, 3] - target_tip_pos)**2) * 50000.0
        rot_err = np.sum((T6[:3, :3] - target_R6)**2) * 500.0
        # 最小化关节跳变
        joint_jump = np.sum((q - q_curr)**2) * 1.0
        return pos_err + rot_err + joint_jump

    res = minimize(loss, q_curr, method='SLSQP', bounds=BOUNDS, options={'maxiter': 400, 'ftol': 1e-9})
    return res.x, res.success


class NudgeClient(Node):
    def __init__(self):
        super().__init__('piper_nudge_client')
        self.curr_joints = None
        self.latest_img = None
        self.bridge = CvBridge()
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

    def send_trajectory(self, q_target, duration_sec=2.0):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [float(x) for x in q_target]
        pt.velocities = [0.0] * 6
        sec = int(duration_sec)
        nanosec = int((duration_sec - sec) * 1e9)
        pt.time_from_start = RosDuration(sec=sec, nanosec=nanosec)
        goal.trajectory.points.append(pt)

        self.done_event.clear()
        fut = self.action_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_cb)
        self.done_event.wait(timeout=duration_sec + 5.0)
        return self.exec_ok

    def _goal_cb(self, fut):
        gh = fut.result()
        if not gh.accepted:
            self.exec_ok = False
            self.done_event.set()
            return
        res_fut = gh.get_result_async()
        res_fut.add_done_callback(self._res_cb)

    def _res_cb(self, fut):
        res = fut.result().result
        self.exec_ok = (res.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        self.done_event.set()


def main():
    args = [a for a in sys.argv[1:] if a != '--dry-run']
    dry_run = '--dry-run' in sys.argv[1:]
    dx_mm = float(args[0]) if len(args) > 0 else 0.0
    dy_mm = float(args[1]) if len(args) > 1 else 0.0
    dz_mm = float(args[2]) if len(args) > 2 else 0.0

    # 安全限制单次微调最大步长 (防止误操作单步超过 50mm)
    if max(abs(dx_mm), abs(dy_mm), abs(dz_mm)) > 50.0:
        print("🔴 安全拒绝: 单次微调步长不可超过 50mm！")
        return

    rclpy.init()
    node = NudgeClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    t0 = time.time()
    while node.curr_joints is None:
        if time.time() - t0 > 4.0:
            print("❌ 未接收到 /joint_states")
            rclpy.shutdown()
            return
        time.sleep(0.05)

    q_curr = node.curr_joints.copy()
    _, T6_curr, T_tip_curr = piper_all_links_fk(q_curr)
    pos_curr = T_tip_curr[:3, 3]
    pos_target = pos_curr + np.array([dx_mm / 1000.0, dy_mm / 1000.0, dz_mm / 1000.0])

    print("=" * 60)
    print(f"🎯 [空间微调指令] 偏移量: dX={dx_mm:+.1f}mm, dY={dy_mm:+.1f}mm, dZ={dz_mm:+.1f}mm")
    print(f"   当前指尖坐标: X={pos_curr[0]*1000:.2f}mm, Y={pos_curr[1]*1000:.2f}mm, Z={pos_curr[2]*1000:.2f}mm")
    print(f"   目标指尖坐标: X={pos_target[0]*1000:.2f}mm, Y={pos_target[1]*1000:.2f}mm, Z={pos_target[2]*1000:.2f}mm")

    q_sol, ok = solve_local_nudge_ik(q_curr, pos_target, T6_curr[:3, :3])
    _, _, T_tip_sol = piper_all_links_fk(q_sol)
    err_mm = np.linalg.norm(T_tip_sol[:3, 3] - pos_target) * 1000
    max_deg_change = np.max(np.abs(q_sol - q_curr)) * 180.0 / math.pi

    print(f"   ✅ 逆解收敛={ok} | 精确度误差={err_mm:.3f}mm | 最大单轴微调={max_deg_change:.2f}°")
    if not ok or err_mm > 2.0 or max_deg_change > 20.0:
        print("🔴 安全中止: 局部逆解跳变过大或精度不足！")
        rclpy.shutdown()
        return

    if dry_run:
        print("🟡 [DRY-RUN 测试通过] 目标角(度):", [round(math.degrees(x), 2) for x in q_sol])
        rclpy.shutdown()
        return

    print("🚀 正在以 2.0 秒极平滑慢速微调移动...")
    if node.send_trajectory(q_sol, duration_sec=2.0):
        print("✅ 微调到位完成！")
        time.sleep(0.4)
        if node.latest_img is not None:
            cv2.imwrite("/home/lab203/piper_students/lq/real_arm_control/hover_verify_preview.jpg", node.latest_img)
    else:
        print("⚠️ 微调运动异常！")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
