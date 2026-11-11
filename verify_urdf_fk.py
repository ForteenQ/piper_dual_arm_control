import time
import math
import numpy as np
from piper_sdk import C_PiperInterface_V2

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

def piper_urdf_fk(joints_rad, gripper_tip_offset_z=0.1358):
    q = joints_rad
    T = np.eye(4)
    T = T @ T_joint([0, 0, 0.123], [0, 0, 0], q[0])
    T = T @ T_joint([0, 0, 0], [1.5708, -0.1359, -3.1416], q[1])
    T = T @ T_joint([0.28503, 0, 0], [0, 0, -1.7939], q[2])
    T = T @ T_joint([-0.021984, -0.25075, 0], [1.5708, 0, 0], q[3])
    T = T @ T_joint([0, 0, 0], [-1.5708, 0, 0], q[4])
    T = T @ T_joint([8.8259e-05, -0.091, 0], [1.5708, 0, 0], q[5])
    T_link6 = T.copy()
    T_tip = T @ T_joint([0, 0, gripper_tip_offset_z], [0, 0, 0], 0)
    return T_link6, T_tip

if __name__ == '__main__':
    p = C_PiperInterface_V2("can0")
    p.ConnectPort()
    time.sleep(0.4)
    js = p.GetArmJointMsgs().joint_state
    ep = p.GetArmEndPoseMsgs().end_pose

    joints = [
        js.joint_1 * 0.001 * math.pi / 180.0,
        js.joint_2 * 0.001 * math.pi / 180.0,
        js.joint_3 * 0.001 * math.pi / 180.0,
        js.joint_4 * 0.001 * math.pi / 180.0,
        js.joint_5 * 0.001 * math.pi / 180.0,
        js.joint_6 * 0.001 * math.pi / 180.0,
    ]
    T_link6, T_tip = piper_urdf_fk(joints, gripper_tip_offset_z=0.150)

    print("1. 固件 GetArmEndPoseMsgs (mm):", [round(ep.X_axis*0.001, 2), round(ep.Y_axis*0.001, 2), round(ep.Z_axis*0.001, 2)])
    print("2. 真实 URDF link6 位置 (mm):  ", np.round(T_link6[:3, 3]*1000, 2).tolist())
    print("3. 真实 URDF 夹爪尖端 (+150mm):", np.round(T_tip[:3, 3]*1000, 2).tolist())
