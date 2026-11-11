#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
将当前机械臂末端夹爪 (Tool Frame / link6) 的自身局部坐标轴 (+X_tool, +Y_tool, -X_tool, -Y_tool)
直接三维投影到当前实拍悬停照片上，让用户直观确认工具坐标系正负方向！
"""
import time
import math
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, JointState
import tf2_ros
from rclpy.duration import Duration

from verify_urdf_fk import piper_urdf_fk

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

def quat_to_rot_mat(qx, qy, qz, qw):
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
    ])

def main():
    rclpy.init()
    node = rclpy.create_node('tool_axes_projector')
    cam_K, cam_D, curr_q = None, None, None

    def info_cb(msg):
        nonlocal cam_K, cam_D
        if cam_K is None:
            cam_K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            cam_D = np.array(msg.d, dtype=np.float64)

    def joint_cb(msg):
        nonlocal curr_q
        if len(msg.name) >= 6:
            try:
                curr_q = [msg.position[msg.name.index(j)] for j in ARM_JOINTS]
            except ValueError:
                pass

    node.create_subscription(CameraInfo, '/camera/camera/color/camera_info', info_cb, 10)
    node.create_subscription(JointState, '/joint_states', joint_cb, 10)
    tf_buffer = tf2_ros.Buffer()
    tf2_ros.TransformListener(tf_buffer, node)

    t0 = time.time()
    tf_c2b = None
    while time.time() - t0 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cam_K is not None and curr_q is not None:
            try:
                tf_c2b = tf_buffer.lookup_transform(
                    'camera_color_optical_frame', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.2)
                )
                break
            except Exception:
                pass

    if cam_K is None or curr_q is None or tf_c2b is None:
        print("Failed to get state!")
        return

    tr = tf_c2b.transform.translation
    qr = tf_c2b.transform.rotation
    R_b2c = quat_to_rot_mat(qr.x, qr.y, qr.z, qr.w)
    t_b2c = np.array([[tr.x], [tr.y], [tr.z]])
    rvec, _ = cv2.Rodrigues(R_b2c)

    T6, T_tip = piper_urdf_fk(curr_q, 0.136)
    p0 = T_tip[:3, 3] # 当前指尖中心在 base_link 下的真实 3D 坐标
    x_dir = T6[:3, 0] # 工具 +X 轴 (垂平指尖连线)
    y_dir = T6[:3, 1] # 工具 +Y 轴 (两指开合连线)

    L = 0.08 # 箭头长度 8cm
    pts_base = np.array([
        p0,
        p0 + L * x_dir,
        p0 - L * x_dir,
        p0 + L * y_dir,
        p0 - L * y_dir,
    ], dtype=np.float32)

    img_pts, _ = cv2.projectPoints(pts_base, rvec, t_b2c, cam_K, cam_D)
    img_pts = img_pts.reshape(-1, 2).astype(int)

    path = "/home/lab203/piper_students/lq/real_arm_control/hover_verify_preview.jpg"
    img = cv2.imread(path)
    if img is None:
        print("Image not found!")
        return

    o = tuple(img_pts[0])
    px_p, px_n = tuple(img_pts[1]), tuple(img_pts[2])
    py_p, py_n = tuple(img_pts[3]), tuple(img_pts[4])

    # 绘制工具坐标系四向箭头
    # +X_tool / -X_tool 用橙黄色; +Y_tool / -Y_tool (手指开合方向) 用亮青色/洋红色
    cv2.arrowedLine(img, o, px_p, (0, 200, 255), 3, tipLength=0.2)
    cv2.putText(img, "+X_tool", (px_p[0]-20, px_p[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 200, 255), 2)

    cv2.arrowedLine(img, o, px_n, (0, 120, 200), 2, tipLength=0.15)
    cv2.putText(img, "-X_tool", (px_n[0]-20, px_n[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 120, 200), 2)

    cv2.arrowedLine(img, o, py_p, (255, 255, 0), 3, tipLength=0.2)
    cv2.putText(img, "+Y_tool (Open)", (py_p[0]-60, py_p[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

    cv2.arrowedLine(img, o, py_n, (255, 0, 255), 3, tipLength=0.2)
    cv2.putText(img, "-Y_tool (Open)", (py_n[0]-20, py_n[1]+20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)

    cv2.circle(img, o, 6, (0, 0, 255), -1)

    out_path = "/home/lab203/piper_students/lq/real_arm_control/tool_axes_preview.jpg"
    cv2.imwrite(out_path, img)
    print("Saved:", out_path)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
