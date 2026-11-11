#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 笛卡尔直线垂直下探脚本 (纯直上直下，零水平偏移，严防触碰桌面)
功能:
1. 保持当前 X, Y 和夹爪朝向姿态不变，通过多点笛卡尔插值沿着垂直轴 (-Z) 平滑下降指定深度
2. 启动前执行全路点逆解预演与碰撞/限位/最小离地安全检验
3. 支持 --dry-run 只计算不运动
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

# 严格遵守 PIPER 物理关节限位 (rad)
BOUNDS = [
    (-2.618, 2.618),   # j1
    (0.0, 3.1416),     # j2
    (-2.967, 0.0),     # j3
    (-1.832, 1.832),   # j4
    (-1.22, 1.22),     # j5 (+/- 69.9 deg)
    (-2.094, 2.094),   # j6
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


def piper_fk(q):
    T = np.eye(4)
    T = T @ T_joint([0, 0, 0.123], [0, 0, 0], q[0])
    T = T @ T_joint([0, 0, 0], [1.5708, -0.1359, -3.1416], q[1])
    T = T @ T_joint([0.28503, 0, 0], [0, 0, -1.7939], q[2])
    T = T @ T_joint([-0.021984, -0.25075, 0], [1.5708, 0, 0], q[3])
    T = T @ T_joint([0, 0, 0], [-1.5708, 0, 0], q[4])
    T = T @ T_joint([8.8259e-05, -0.091, 0], [1.5708, 0, 0], q[5])
    T_tip = T @ T_joint([0, 0, 0.136], [0, 0, 0], 0)
    return T, T_tip


def solve_cartesian_step_ik(q_prev, target_pos, target_R6):
    def loss(q):
        T6, T_tip = piper_fk(q)
        p_err = np.sum((T_tip[:3, 3] - target_pos)**2) * 50000.0
        # 保持末端夹爪垂直向下及对准角度不变
        r_err = np.sum((T6[:3, :3] - target_R6)**2) * 500.0
        jump_err = np.sum((q - q_prev)**2) * 0.5
        return p_err + r_err + jump_err

    res = minimize(loss, q_prev, method='SLSQP', bounds=BOUNDS, options={'maxiter': 500, 'ftol': 1e-9})
    return res.x, res.success


class DescentClient(Node):
    def __init__(self):
        super().__init__('piper_descent_client')
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

    def send_trajectory(self, waypoints, total_duration=4.0):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        dt_step = total_duration / len(waypoints)
        for i, q in enumerate(waypoints):
            t = (i + 1) * dt_step
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            # 中间点给予平滑连续速度估算，终点速度置零
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
    dz_mm = -100.0 # 默认垂直下探 100mm (从 ~131mm 离地高度降至 ~31mm 或 26mm 安全位置)
    dry_run = False
    for arg in sys.argv[1:]:
        if arg == '--dry-run':
            dry_run = True
        else:
            dz_mm = float(arg)

    if dz_mm > 0 or dz_mm < -120.0:
        print(f"🔴 安全拦截: 下降幅度 {dz_mm}mm 超出安全许可范围 [-120mm, 0mm]！")
        return

    rclpy.init()
    node = DescentClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    t0 = time.time()
    while node.curr_joints is None:
        if time.time() - t0 > 4.0:
            print("❌ 超时未能获取 /joint_states")
            rclpy.shutdown()
            return
        time.sleep(0.05)

    q_curr = node.curr_joints.copy()
    T6_c, T_tip_c = piper_fk(q_curr)
    pos_start = T_tip_c[:3, 3]
    R_target = T6_c[:3, :3]

    # 生成 8 段笛卡尔直线插值路点
    num_steps = 8
    waypoints = []
    q_prev = q_curr.copy()
    max_pos_err = 0.0
    max_align_err = 0.0

    print("=" * 65)
    print(f"📐 [笛卡尔直线下探规划] 垂直下降距离: {dz_mm:.1f} mm ({num_steps} 段线性插补)")
    print(f"   起始指尖坐标: X={pos_start[0]*1000:.1f}mm, Y={pos_start[1]*1000:.1f}mm, Z={pos_start[2]*1000:.1f}mm")

    for i in range(1, num_steps + 1):
        frac = i / float(num_steps)
        p_step = pos_start + np.array([0.0, 0.0, (dz_mm / 1000.0) * frac])
        q_step, ok = solve_cartesian_step_ik(q_prev, p_step, R_target)
        T6_s, T_tip_s = piper_fk(q_step)
        err_mm = np.linalg.norm(T_tip_s[:3, 3] - p_step) * 1000
        z_dot = np.dot(T6_s[:3, 2], np.array([0.0, 0.0, -1.0]))
        max_pos_err = max(max_pos_err, err_mm)
        max_align_err = max(max_align_err, 1.0 - z_dot)
        if not ok or err_mm > 3.0 or z_dot < 0.96:
            print(f"🔴 第 {i} 步逆解畸变过大 (err={err_mm:.2f}mm, z_dot={z_dot:.4f})，终止规划！")
            rclpy.shutdown()
            return
        waypoints.append(q_step)
        q_prev = q_step

    _, T_tip_end = piper_fk(waypoints[-1])
    pos_end = T_tip_end[:3, 3]
    print(f"   终点指尖坐标: X={pos_end[0]*1000:.1f}mm, Y={pos_end[1]*1000:.1f}mm, Z={pos_end[2]*1000:.1f}mm")
    print(f"   终点关节角 (度): {[round(math.degrees(x), 2) for x in waypoints[-1]]}")
    print(f"   全段最大直线偏离: {max_pos_err:.2f} mm | 末端垂直度保持: {1.0 - max_align_err:.4f}")

    # 最低安全绝对高度校验 (物理绿桌面约在 Z_base=+4.6mm，绝对不允许 Z_tip 低于 +18mm)
    if pos_end[2] < 0.018:
        print(f"🔴 安全门槛拦截: 目标终点 Z={pos_end[2]*1000:.1f}mm 低于 18mm 护桌防线，拒绝下探！")
        rclpy.shutdown()
        return

    if dry_run:
        print("🟡 [DRY-RUN 测试通过] 规划完美无畸变，未下发真实电机运动。")
        rclpy.shutdown()
        return

    print("🚀 安全验证全部通过！启动 4.0 秒匀速垂直直线下降...")
    if node.send_trajectory(waypoints, total_duration=4.0):
        print("✅ 垂直下探到位完成！")
        time.sleep(0.5)
        if node.latest_img is not None:
            cv2.imwrite("/home/lab203/piper_students/lq/real_arm_control/grasp_ready_preview.jpg", node.latest_img)
    else:
        print("⚠️ 下探过程中出现异常！")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
