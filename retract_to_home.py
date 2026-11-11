#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 机械臂安全退回原位脚本
策略:
1. 读取当前关节位置与末端位置
2. 先沿垂直方向 (+Z) 安全平稳抬高 100mm (确保不横扫桌上的方块)
3. 再分步平滑折叠收回到原点待命姿态 [0°, 5°, -5°, 0°, 20°, 0°]
"""

import time
import math
import threading
import numpy as np
from scipy.optimize import minimize

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration

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

def solve_lift_ik(q_curr, dz=0.10):
    T6_c, T_tip_c = piper_fk(q_curr)
    target_pos = T_tip_c[:3, 3] + np.array([0.0, 0.0, dz])
    target_R6 = T6_c[:3, :3]

    def loss(q):
        T6, T_tip = piper_fk(q)
        p_err = np.sum((T_tip[:3, 3] - target_pos)**2) * 30000.0
        r_err = np.sum((T6[:3, :3] - target_R6)**2) * 200.0
        j_err = np.sum((q - q_curr)**2) * 1.0
        return p_err + r_err + j_err

    res = minimize(loss, q_curr, method='SLSQP', bounds=BOUNDS, options={'maxiter': 300})
    return res.x

class RetractClient(Node):
    def __init__(self):
        super().__init__('piper_retract_client')
        self.curr_joints = None
        self.action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        self.done_event = threading.Event()
        self.exec_ok = False

    def joint_cb(self, msg):
        if len(msg.name) >= 6:
            try:
                self.curr_joints = np.array([msg.position[msg.name.index(j)] for j in ARM_JOINTS], dtype=float)
            except ValueError:
                pass

    def send_trajectory(self, waypoints_rad, durations_sec):
        if not self.action_client.wait_for_server(timeout_sec=5.0):
            print("❌ Action Server 未就绪！")
            return False
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        t_acc = 0.0
        for q, dt in zip(waypoints_rad, durations_sec):
            t_acc += dt
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.velocities = [0.0] * 6
            sec = int(t_acc)
            nanosec = int((t_acc - sec) * 1e9)
            pt.time_from_start = RosDuration(sec=sec, nanosec=nanosec)
            goal.trajectory.points.append(pt)

        self.done_event.clear()
        fut = self.action_client.send_goal_async(goal)
        fut.add_done_callback(self._goal_cb)
        self.done_event.wait(timeout=t_acc + 10.0)
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
    rclpy.init()
    node = RetractClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    t0 = time.time()
    while node.curr_joints is None:
        if time.time() - t0 > 4.0:
            print("❌ 超时未获取关节状态")
            rclpy.shutdown()
            return
        time.sleep(0.05)

    q_curr = node.curr_joints.copy()
    print("1️⃣ 计算垂直抬升 10cm 路点 (远离方块)...")
    q_lift = solve_lift_ik(q_curr, dz=0.10)

    # 原位安全待命点 (J1=0, J2=5°, J3=-5°, J4=0, J5=20°, J6=0)
    deg = math.pi / 180.0
    q_home = np.array([0.0, 5.0 * deg, -5.0 * deg, 0.0, 20.0 * deg, 0.0])

    print("2️⃣ 正在执行两段式平滑归位: 垂直抬升(2.5s) -> 收回原位(4.0s)...")
    ok = node.send_trajectory([q_lift, q_home], [2.5, 4.0])
    if ok:
        print("✅ 机械臂已安全抬起并平稳收回原位！")
    else:
        print("⚠️ 收回原位出现异常，请检查状态！")

    rclpy.shutdown()

if __name__ == '__main__':
    main()
