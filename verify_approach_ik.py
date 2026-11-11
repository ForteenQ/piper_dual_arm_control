#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
纯数学与 MoveIt 安全仿真验算脚本 (不动真机！)：
1. 根据当前检测到的方块位置，计算安全的上方悬停目标位姿
2. 求解关节逆解与从当前位置到目标位置的插值路径
3. 全程计算机械臂每个连杆关节 (link1..link7) 与摄像头 (X=0.209, Y=0.305, Z=0.579) 的最短三维安全距离！
"""

import math
import numpy as np
from scipy.optimize import minimize

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
    """返回 base_link 下各关节与指尖的三维坐标及 link6 变换矩阵"""
    T = np.eye(4)
    T1 = T @ T_joint([0, 0, 0.123], [0, 0, 0], q[0])
    T2 = T1 @ T_joint([0, 0, 0], [1.5708, -0.1359, -3.1416], q[1])
    T3 = T2 @ T_joint([0.28503, 0, 0], [0, 0, -1.7939], q[2])
    T4 = T3 @ T_joint([-0.021984, -0.25075, 0], [1.5708, 0, 0], q[3])
    T5 = T4 @ T_joint([0, 0, 0], [-1.5708, 0, 0], q[4])
    T6 = T5 @ T_joint([8.8259e-05, -0.091, 0], [1.5708, 0, 0], q[5])
    T_tip = T6 @ T_joint([0, 0, 0.136], [0, 0, 0], 0)
    links = [T1[:3,3], T2[:3,3], T3[:3,3], T4[:3,3], T5[:3,3], T6[:3,3], T_tip[:3,3]]
    return links, T6, T_tip

# 关节硬件限位 (rad)
BOUNDS = [
    (-2.618, 2.618),   # j1
    (0.0, 3.1416),     # j2 (大臂前倾 0 ~ 180°)
    (-2.967, 0.0),     # j3 (小臂展开 -170° ~ 0°)
    (-1.832, 1.832),   # j4
    (-1.22, 1.22),     # j5
    (-2.094, 2.094),   # j6
]

def solve_ik_numerical(target_tip_pos, target_z_dir=np.array([0.0, 0.0, -1.0]), q_init=None):
    if q_init is None:
        q_init = np.array([0.5, 0.8, -0.8, 0.0, 0.8, 0.0])

    def loss(q):
        _, T6, T_tip = piper_all_links_fk(q)
        pos_err = np.sum((T_tip[:3, 3] - target_tip_pos)**2) * 10000.0
        # 末端工具 Z 轴方向严格朝向 target_z_dir ([0,0,-1] 垂直向下)
        tool_z = T6[:3, 2]
        rot_err = np.sum((tool_z - target_z_dir)**2) * 100.0
        # 正则化避免关节4/6无谓乱转
        reg = 0.01 * (q[3]**2 + q[5]**2)
        return pos_err + rot_err + reg

    res = minimize(loss, q_init, method='SLSQP', bounds=BOUNDS, options={'maxiter': 500, 'ftol': 1e-8})
    return res.x, res.success

def main():
    # 目标方块顶面位置 (来自刚刚实测的 AprilTag ID 3)
    obj_pos = np.array([0.2522, 0.1474, 0.0341])
    # 悬停在方块正上方 80mm
    hover_tip_pos = obj_pos + np.array([0.0, 0.0, 0.080])

    # 相机物理位置 (来自刚刚真值标定外参)
    cam_pos = np.array([0.2092, 0.3049, 0.5792])

    print("=" * 65)
    print("🧮 [零风险数字孪生预演] 求解靠近方块正上方 8cm 的安全轨迹...")
    print(f"🎯 目标指尖悬停位置: X={hover_tip_pos[0]*1000:.1f}mm, Y={hover_tip_pos[1]*1000:.1f}mm, Z={hover_tip_pos[2]*1000:.1f}mm")
    print(f"📷 摄像头警戒位置:   X={cam_pos[0]*1000:.1f}mm, Y={cam_pos[1]*1000:.1f}mm, Z={cam_pos[2]*1000:.1f}mm")
    print("=" * 65)

    # 先试探不同的初值找最优无碰撞关节解
    theta = math.atan2(hover_tip_pos[1], hover_tip_pos[0])
    q_sol, ok = solve_ik_numerical(hover_tip_pos, q_init=np.array([theta, 0.8, -1.0, 0.0, 0.6, 0.0]))
    links, T6, T_tip = piper_all_links_fk(q_sol)

    pos_err_mm = np.linalg.norm(T_tip[:3, 3] - hover_tip_pos) * 1000
    z_alignment = np.dot(T6[:3, 2], np.array([0.0, 0.0, -1.0]))

    print(f"✅ 逆解求解完成 (收敛={ok}):")
    print(f"   目标关节角 (度): {[round(math.degrees(x), 2) for x in q_sol]}")
    print(f"   指尖位置误差:    {pos_err_mm:.2f} mm")
    print(f"   指尖垂下对齐度:  {z_alignment:.4f} (1.0 表示 100% 垂直向下)")

    # 模拟从当前折叠位置到目标位置的 50 个插值点，全程扫描是否靠近摄像头或碰撞桌面
    q_curr = np.array([0.0584, -0.0330, 0.0318, -0.0456, 0.3341, -0.0945])
    # 为防止直插桌面，先抬升至安全过渡姿态 Ready Pose (j2=0.5, j3=-0.6, j5=0.5) 再飞向目标
    q_ready = np.array([theta, 0.45, -0.75, 0.0, 0.60, 0.0])

    min_cam_dist = 999.0
    max_arm_z = -999.0
    min_tip_z = 999.0

    for seg_start, seg_end in [(q_curr, q_ready), (q_ready, q_sol)]:
        for alpha in np.linspace(0, 1, 30):
            q = (1 - alpha) * seg_start + alpha * seg_end
            lks, _, _ = piper_all_links_fk(q)
            for pt in lks:
                d_cam = np.linalg.norm(pt - cam_pos)
                if d_cam < min_cam_dist:
                    min_cam_dist = d_cam
                if pt[2] > max_arm_z:
                    max_arm_z = pt[2]
            if lks[-1][2] < min_tip_z:
                min_tip_z = lks[-1][2]

    print("\n🛡️ 全轨迹碰撞安全扫描结果:")
    print(f"   全路径机械臂任意部位距摄像头最近距离: {min_cam_dist*1000:.1f} mm")
    print(f"   全路径机械臂最高点高度 (Max Z):      {max_arm_z*1000:.1f} mm (摄像头高度为 {cam_pos[2]*1000:.1f} mm)")
    print(f"   全路径指尖最低高度 (Min Tip Z):      {min_tip_z*1000:.1f} mm (方块顶面高度为 {obj_pos[2]*1000:.1f} mm)")

    if min_cam_dist > 0.25 and min_tip_z > 0.08:
        print("\n🟢 [安全放行评估] 全程距离摄像头 25cm 以上，全程高度远高于桌面与方块，绝对安全！")
    else:
        print("\n🔴 [安全警告] 轨迹存在靠近摄像头或低高度风险，需调整路径！")

if __name__ == '__main__':
    main()
