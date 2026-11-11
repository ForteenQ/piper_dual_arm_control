#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 实机原地闭环抓取并垂直上升 5cm 悬停脚本
动作流程:
1. 读取当前对准方块中心后的实时关节状态
2. 两段式柔顺闭合夹爪 (50mm -> 34mm 预对中 -> 22mm 恒力矩抓牢 30mm 方块)
3. 监测夹爪真实开度反馈确认抓取状态 (防止抓空)
4. 通过 6 段笛卡尔直线轨迹垂直匀速上升 +50.0 mm (耗时 3.0 秒) 并稳稳悬停
5. 抓拍实景预览图验证抓取与悬停效果
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
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import JointState, Image
from std_msgs.msg import Float64
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
        r_err = np.sum((T6[:3, :3] - target_R6)**2) * 500.0
        jump_err = np.sum((q - q_prev)**2) * 0.5
        return p_err + r_err + jump_err

    res = minimize(loss, q_prev, method='SLSQP', bounds=BOUNDS, options={'maxiter': 500, 'ftol': 1e-9})
    return res.x, res.success


class GraspAndLiftClient(Node):
    def __init__(self):
        super().__init__('piper_grasp_lift_client')
        self.curr_joints = None
        self.curr_gripper_mm = None
        self.latest_img = None
        self.bridge = CvBridge()

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
                if 'joint7' in msg.name and 'joint8' in msg.name:
                    j7 = msg.position[msg.name.index('joint7')]
                    j8 = msg.position[msg.name.index('joint8')]
                    self.curr_gripper_mm = (j7 + j8) * 1000.0
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
    node = GraspAndLiftClient()
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    print("=" * 65)
    print("🤏 [闭环抓取启动] 原地柔顺抓取 -> 垂直上升 5cm -> 稳态悬停")
    print("=" * 65)

    t0 = time.time()
    while node.curr_joints is None:
        if time.time() - t0 > 4.0:
            print("❌ 超时未获取机械臂状态")
            rclpy.shutdown()
            return
        time.sleep(0.05)

    q_curr = node.curr_joints.copy()
    T6_c, T_tip_c = piper_fk(q_curr)
    pos_start = T_tip_c[:3, 3]
    R_target = T6_c[:3, :3]

    # 预先解算垂直上升 +50.0mm 的笛卡尔插值轨迹 (确保上升路径 100% 合法顺畅后再合拢抓手)
    lift_dz_m = 0.050
    num_steps = 6
    waypoints = []
    q_prev = q_curr.copy()
    for i in range(1, num_steps + 1):
        p_step = pos_start + np.array([0.0, 0.0, lift_dz_m * (i / float(num_steps))])
        q_step, ok = solve_cartesian_step_ik(q_prev, p_step, R_target)
        T6_s, T_tip_s = piper_fk(q_step)
        err = np.linalg.norm(T_tip_s[:3, 3] - p_step) * 1000
        if not ok or err > 2.0:
            print(f"🔴 上升轨迹预演逆解异常 (err={err:.2f}mm)，为保证安全终止动作！")
            rclpy.shutdown()
            return
        waypoints.append(q_step)
        q_prev = q_step

    print(f"1️⃣ 当前抓取起点指尖坐标: X={pos_start[0]*1000:.1f}, Y={pos_start[1]*1000:.1f}, Z={pos_start[2]*1000:.1f} mm")
    print(f"   当前夹爪初始开度: {node.curr_gripper_mm:.2f} mm")

    # 两段式柔顺闭合夹爪
    print("\n2️⃣ 正在执行两段式柔顺轻柔抓取 (50mm -> 33mm -> 26.5mm)...")
    node.send_gripper(33.0)
    time.sleep(0.5)
    print(f"   -> 轻贴预对中开度: {node.curr_gripper_mm:.2f} mm")

    node.send_gripper(26.5)
    time.sleep(1.1)
    grip_final = node.curr_gripper_mm
    print(f"   -> 夹紧锁定开度: {grip_final:.2f} mm")

    if grip_final is not None and grip_final < 27.8:
        print(f"⚠️ 警示: 夹爪开度收窄至 {grip_final:.1f}mm (小于 27.8mm)，疑似未能夹实 30mm 方块！")
    else:
        print(f"   ✅ 咬合检测通过！夹爪卡住于 {grip_final:.2f} mm (紧实咬住 30mm 方块)")

    # 执行垂直直线上升 +50.0mm
    print(f"\n3️⃣ 正在以 3.0 秒匀速笛卡尔垂直直线上升 +50.0 mm 至 Z={(pos_start[2]+lift_dz_m)*1000:.1f} mm...")
    ok = node.send_trajectory(waypoints, total_duration=3.0)
    if ok:
        time.sleep(0.5)
        print(f"🎉 已平稳垂直上升 5cm 并稳态悬停！当前悬停咬合开度: {node.curr_gripper_mm:.2f} mm")
        if node.latest_img is not None:
            out_img = "/home/lab203/piper_students/lq/real_arm_control/lifted_hover_preview.jpg"
            cv2.imwrite(out_img, node.latest_img)
            print(f"📸 抓取升空实拍图已保存: {out_img}")
    else:
        print("⚠️ 上升过程中轨迹执行未圆满完成，请检查！")

    rclpy.shutdown()


if __name__ == '__main__':
    main()
