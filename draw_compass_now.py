#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
在当前机械臂下探到位图 (grasp_ready_preview.jpg) 上，直接投影绘制基座坐标系的四个方向箭头，
并标注真实桌面物理参照物，让用户一眼指出 XOY 平面内 1cm 的调整方向！
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
    node = rclpy.create_node('compass_projector')
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
        print("Failed!")
        return

    tr = tf_c2b.transform.translation
    qr = tf_c2b.transform.rotation
    R_b2c = quat_to_rot_mat(qr.x, qr.y, qr.z, qr.w)
    t_b2c = np.array([[tr.x], [tr.y], [tr.z]])
    rvec, _ = cv2.Rodrigues(R_b2c)

    _, T_tip = piper_urdf_fk(curr_q, 0.136)
    p0 = T_tip[:3, 3]

    L = 0.12 # 12cm 长大箭头，清晰露出在夹爪外围！
    pts_base = np.array([
        p0,
        p0 + np.array([ L, 0, 0]), # +X (前 -> 三脚架方向)
        p0 + np.array([-L, 0, 0]), # -X (后 -> 机械臂底座方向)
        p0 + np.array([0,  L, 0]), # +Y (左 -> 右侧空桌面方向)
        p0 + np.array([0, -L, 0]), # -Y (右 -> 左侧桌外沿方向)
    ], dtype=np.float32)

    img_pts, _ = cv2.projectPoints(pts_base, rvec, t_b2c, cam_K, cam_D)
    img_pts = img_pts.reshape(-1, 2).astype(int)

    img = cv2.imread("/home/lab203/piper_students/lq/real_arm_control/grasp_ready_preview.jpg")
    o = tuple(img_pts[0])
    p_xp, p_xn, p_yp, p_yn = tuple(img_pts[1]), tuple(img_pts[2]), tuple(img_pts[3]), tuple(img_pts[4])

    # 画四个大方向指南针
    cv2.arrowedLine(img, o, p_xp, (0, 0, 255), 3, tipLength=0.2)
    cv2.putText(img, "+X (Tripod)", (p_xp[0]-45, p_xp[1]+25), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)

    cv2.arrowedLine(img, o, p_xn, (255, 0, 255), 3, tipLength=0.2)
    cv2.putText(img, "-X (Arm Base)", (p_xn[0]-45, p_xn[1]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 0, 255), 2)

    cv2.arrowedLine(img, o, p_yp, (0, 255, 0), 3, tipLength=0.2)
    cv2.putText(img, "+Y (Open Table)", (p_yp[0]+10, p_yp[1]+5), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

    cv2.arrowedLine(img, o, p_yn, (255, 255, 0), 3, tipLength=0.2)
    cv2.putText(img, "-Y (Left Edge)", (p_yn[0]-140, p_yn[1]+5), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)

    cv2.circle(img, o, 6, (0, 255, 255), -1)

    out_path = "/home/lab203/piper_students/lq/real_arm_control/compass_preview.jpg"
    cv2.imwrite(out_path, img)
    print("Saved:", out_path)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
