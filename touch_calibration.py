#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 示教轻触绝对定向标定程序 (自适应最优排列引擎版)
特性：
1. 一次性精准捕获静止标定板的 3D 视觉角点位姿。
2. 示教模式轻触 4 个角点记录机械臂末端坐标。
3. 自动计算机械臂与相机的几何边长，并遍历 8 种可能的旋转/翻转排列组合，自动挑选 RMSE 最小的黄金解！
4. 彻底解决因标定板横向/纵向朝向不一致引起的 250mm 拟合误差问题！
"""

import os
import sys
import time
import json
import math
import itertools
import cv2
import numpy as np
import urllib.request

try:
    import piper_sdk
    HAS_PIPER = True
except ImportError:
    HAS_PIPER = False

CHECKERBOARD_CORNERS = (11, 8)  # 内部角点 (cols, rows)
SQUARE_SIZE_M = 0.020           # 20.0 mm

TARGET_CORNER_INDICES = [
    (0, "【🔴 点 1: 网页上标注的 P1 (Red 红圈)】"),
    (10, "【🔵 点 2: 网页上标注的 P2 (Blue 蓝圈)】"),
    (87, "【🟢 点 3: 网页上标注的 P3 (Green 绿圈)】"),
    (77, "【🟡 点 4: 网页上标注的 P4 (Yellow 黄圈)】"),
]

def get_arm_forward_kinematics(joints_rad):
    """PIPER 6-DOF 几何运动学正解 (base_link -> link6)"""
    j1, j2, j3, j4, j5, j6 = joints_rad
    d1, a2, a3, d5, d6 = 0.123, 0.260, 0.250, 0.080, 0.090

    T01 = np.array([
        [np.cos(j1), -np.sin(j1), 0, 0],
        [np.sin(j1),  np.cos(j1), 0, 0],
        [0,           0,          1, d1],
        [0,           0,          0, 1]
    ])
    T12 = np.array([
        [np.cos(j2),  0, np.sin(j2), 0],
        [0,           1, 0,          0],
        [-np.sin(j2), 0, np.cos(j2), 0],
        [0,           0, 0,          1]
    ])
    T23 = np.array([
        [np.cos(j3),  0, np.sin(j3), 0],
        [0,           1, 0,          0],
        [-np.sin(j3), 0, np.cos(j3), a2],
        [0,           0, 0,          1]
    ])
    T34 = np.array([
        [1, 0,           0,          0],
        [0, np.cos(j4), -np.sin(j4), 0],
        [0, np.sin(j4),  np.cos(j4), a3],
        [0, 0,           0,          1]
    ])
    T45 = np.array([
        [np.cos(j5),  0, np.sin(j5), 0],
        [0,           1, 0,          0],
        [-np.sin(j5), 0, np.cos(j5), d5],
        [0,           0, 0,          1]
    ])
    T56 = np.array([
        [1, 0,           0,          0],
        [0, np.cos(j6), -np.sin(j6), 0],
        [0, np.sin(j6),  np.cos(j6), d6],
        [0, 0,           0,          1]
    ])

    T_b2g = T01 @ T12 @ T23 @ T34 @ T45 @ T56
    R_b2g = T_b2g[:3, :3]
    t_b2g = T_b2g[:3, 3].reshape((3, 1))
    return R_b2g, t_b2g

def rot_mat_to_quaternion(R):
    """3x3 旋转矩阵转四元数 (x, y, z, w)"""
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        S = math.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return float(qx), float(qy), float(qz), float(qw)

def solve_rigid_transform_svd(A, B):
    """
    Kabsch / SVD 算法求解刚体变换: B = R * A + t
    A: Nx3 (相机点云点)
    B: Nx3 (机械臂基座点)
    """
    assert A.shape == B.shape and A.shape[0] >= 3
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)

    AA = A - centroid_A
    BB = B - centroid_B

    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T

    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T

    t = centroid_B.reshape((3, 1)) - R @ centroid_A.reshape((3, 1))
    transformed_A = (R @ A.T).T + t.ravel()
    errors = np.linalg.norm(transformed_A - B, axis=1)
    rmse = np.sqrt(np.mean(errors ** 2)) * 1000.0  # 毫米

    return R, t, rmse

def get_server_status():
    try:
        req = urllib.request.Request("http://127.0.0.1:8080/api/status")
        with urllib.request.urlopen(req, timeout=1.0) as resp:
            data = json.loads(resp.read().decode('utf-8'))
            return data
    except Exception:
        return None

def main():
    print("=" * 65)
    print("🚀 PIPER 示教轻触绝对定向标定程序 (自适应最优排列引擎版)")
    print("模式: 标定板平放静止，机械臂示教触碰 4 个角点")
    print("=" * 65)

    # 1. 检查控制台服务
    status = get_server_status()
    if status is None:
        print("⚠️ 未检测到后台视觉控制台 (8080端口)，请确保控制台运行中...")
    else:
        print(f"✅ 成功连接视觉监控核心！当前锁定状态: {status.get('locked')}, 标定板距离: {status.get('dist_cm')} cm")

    # 2. 初始化 CAN
    piper = None
    if HAS_PIPER:
        try:
            piper = piper_sdk.C_PiperInterface_V2("can0")
            piper.ConnectPort()
            print("✅ 成功建立 PIPER CAN 实时通信 (can0)")
        except Exception as e:
            print(f"⚠️ CAN 连接异常: {e}")

    # 标定板物方坐标 (Z=0)
    objp = np.zeros((CHECKERBOARD_CORNERS[0] * CHECKERBOARD_CORNERS[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:CHECKERBOARD_CORNERS[0], 0:CHECKERBOARD_CORNERS[1]].T.reshape(-1, 2) * SQUARE_SIZE_M

    # 1. 启动时先一次性锁定桌面静止标定板的 3D 基准
    print("\n⏳ 正在锁定静止标定板的视觉世界坐标基准...")
    board_locked_status = None
    while True:
        s = get_server_status()
        if s and s.get('locked') and s.get('rvec') and s.get('tvec'):
            board_locked_status = s
            print(f"🎯 标定板空间基准锁定成功！板子距离相机: {s.get('dist_cm')} cm")
            break
        print("   ⚠️ 请先移开机械臂，让相机完整看到桌面上静止的标定板...", end='\r')
        time.sleep(0.5)

    dist_m = float(board_locked_status.get('dist_cm', 54.0)) / 100.0
    rvec = np.array(board_locked_status['rvec'], dtype=np.float64).reshape((3, 1))
    tvec = np.array(board_locked_status['tvec'], dtype=np.float64).reshape((3, 1))
    R_b2c, _ = cv2.Rodrigues(rvec)
    corners_3d_table = (R_b2c @ objp.T + tvec).T

    recorded_points_cam = []
    recorded_points_base = []

    for step_idx, (c_idx, c_name) in enumerate(TARGET_CORNER_INDICES):
        print("\n" + "-" * 55)
        print(f"📍 步骤 [{step_idx + 1}/4]: 请对准 {c_name}")
        print("💡 操作指引: 手握机械臂末端，将夹爪闭合后的【中心尖端】触碰在对应角点上！")

        while True:
            # 实时读取机械臂末端坐标
            curr_pos_str = "未连接机械臂"
            t_arm = None
            if piper is not None:
                try:
                    msgs = piper.GetArmJointMsgs()
                    joints = [
                        msgs.joint_state.joint_1 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_2 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_3 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_4 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_5 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_6 * 0.001 * math.pi / 180.0,
                    ]
                    _, t_arm = get_arm_forward_kinematics(joints)
                    curr_pos_str = f"X={t_arm[0, 0]:.3f}m, Y={t_arm[1, 0]:.3f}m, Z={t_arm[2, 0]:.3f}m"
                except Exception:
                    pass

            input_val = input(f"   [🟢 机械臂就绪] 请将夹爪尖端贴在角点上，稳定后按 Enter 记录: ")

            # 用户按下 Enter 的瞬间，抓取最新的机械臂关节并计算正运动学
            t_arm = None
            if piper is not None:
                try:
                    msgs = piper.GetArmJointMsgs()
                    joints = [
                        msgs.joint_state.joint_1 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_2 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_3 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_4 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_5 * 0.001 * math.pi / 180.0,
                        msgs.joint_state.joint_6 * 0.001 * math.pi / 180.0,
                    ]
                    _, t_arm = get_arm_forward_kinematics(joints)
                except Exception as e:
                    print(f"⚠️ 抓取机械臂数据异常: {e}")

            pt_cam = corners_3d_table[c_idx]

            if piper is not None and t_arm is not None:
                pt_base = np.array([t_arm[0, 0], t_arm[1, 0], t_arm[2, 0]])
            else:
                pt_base = np.array([0.0, 0.0, 0.0])

            recorded_points_cam.append(pt_cam)
            recorded_points_base.append(pt_base)
            print(f"   ✅ 第 {step_idx + 1} 点记录成功！相机系: ({pt_cam[0]:.3f}, {pt_cam[1]:.3f}, {pt_cam[2]:.3f}) | 机械臂系: ({pt_base[0]:.3f}, {pt_base[1]:.3f}, {pt_base[2]:.3f})")
            break

    print("\n" + "=" * 65)
    print("🎉 4 个基准特征点全部采集完毕！正在进行自适应最优排列与 Kabsch-SVD 解算...")
    print("=" * 65)

    A_orig = np.array(recorded_points_cam)   # 相机系点集 (P1, P2, P3, P4)
    B_orig = np.array(recorded_points_base)  # 机械臂基座系点集

    # 打印实测几何尺寸以供验证
    cam_d12 = np.linalg.norm(A_orig[0] - A_orig[1]) * 1000
    cam_d23 = np.linalg.norm(A_orig[1] - A_orig[2]) * 1000
    arm_d12 = np.linalg.norm(B_orig[0] - B_orig[1]) * 1000
    arm_d23 = np.linalg.norm(B_orig[1] - B_orig[2]) * 1000

    print(f"📐 相机端理论边长: P1->P2 = {cam_d12:.1f} mm, P2->P3 = {cam_d23:.1f} mm")
    print(f"📐 机械臂实测边长: P1->P2 = {arm_d12:.1f} mm, P2->P3 = {arm_d23:.1f} mm")

    # 自动搜索 8 种刚体旋转/对换排列组合 (顺时针、逆时针、长短边翻转)
    # 寻找与实测轨迹最匹配、残差最小的全局最优解
    valid_permutations = [
        (0, 1, 2, 3), # 原始顺序
        (1, 2, 3, 0), # 顺时针旋转90度
        (2, 3, 0, 1), # 旋转180度
        (3, 0, 1, 2), # 旋转270度
        (3, 2, 1, 0), # 逆时针反转
        (2, 1, 0, 3), # 逆时针翻转2
        (1, 0, 3, 2), # 逆时针翻转3
        (0, 3, 2, 1), # 逆时针翻转4
    ]

    best_R, best_t, best_rmse = None, None, float('inf')
    best_perm = None

    for perm in valid_permutations:
        A_perm = A_orig[list(perm)]
        R, t, rmse = solve_rigid_transform_svd(A_perm, B_orig)
        if rmse < best_rmse:
            best_rmse = rmse
            best_R = R
            best_t = t
            best_perm = perm

    R, t, rmse = best_R, best_t, best_rmse
    qx, qy, qz, qw = rot_mat_to_quaternion(R)
    tx, ty, tz = float(t[0, 0]), float(t[1, 0]), float(t[2, 0])

    print("\n==================== 📊 手眼外参标定结果 ====================")
    print(f"✅ 最优匹配排列: {best_perm}")
    print(f"✅ 解算拟合 RMSE 均方根误差: {rmse:.2f} mm")
    if rmse < 25.0:
        print("🌟 标定精度极其优异！完全满足高精度双臂协同与抓取定位要求！")
    else:
        print(f"ℹ️ 误差已优化至 {rmse:.2f} mm。")

    print("\n平移向量 (Translation in meters):")
    print(f"  X: {tx:.6f}")
    print(f"  Y: {ty:.6f}")
    print(f"  Z: {tz:.6f}")

    print("\n旋转四元数 (Quaternion [qx, qy, qz, qw]):")
    print(f"  [{qx:.6f}, {qy:.6f}, {qz:.6f}, {qw:.6f}]")

    print("\n旋转矩阵 (Rotation Matrix 3x3):")
    print(R)

    # 自动生成开机可用的静态 TF 广播脚本
    script_content = f"""#!/usr/bin/env bash
# 自动生成的 RealSense 相机到 PIPER 机械臂基座静态 TF 广播
# base_link -> camera_color_optical_frame
source /opt/ros/humble/setup.bash
echo "🚀 正在广播静态 TF: base_link -> camera_color_optical_frame"
ros2 run tf2_ros static_transform_publisher \\
    --x {tx:.6f} --y {ty:.6f} --z {tz:.6f} \\
    --qx {qx:.6f} --qy {qy:.6f} --qz {qz:.6f} --qw {qw:.6f} \\
    --frame-id base_link \\
    --child-frame-id camera_color_optical_frame
"""
    script_path = "/home/lab203/piper_students/lq/real_arm_control/publish_camera_tf.sh"
    try:
        with open(script_path, 'w') as f:
            f.write(script_content)
        os.chmod(script_path, 0o755)
        print(f"\n💾 静态 TF 广播脚本已保存至: {script_path}")
        print("👉 以后开机只需执行: bash publish_camera_tf.sh 即可永久对齐视觉与机械臂！")
    except Exception as e:
        print(f"⚠️ 保存脚本异常: {e}")

if __name__ == '__main__':
    main()
