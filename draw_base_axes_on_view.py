#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
利用真值手眼外参矩阵，将机械臂底座坐标系 base_link 的 +X (前方)、+Y (左方) 轴
直接三维投影绘制到桌面实景照片上，彻底消除物理方位歧义！
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import tf2_ros
from rclpy.duration import Duration


def quat_to_rot_mat(qx, qy, qz, qw):
    return np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw),     1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw),     2*(qy*qz + qx*qw),     1 - 2*(qx**2 + qy**2)]
    ])


def main():
    rclpy.init()
    node = rclpy.create_node('axes_projector')
    cam_K, cam_D = None, None

    def cb(msg):
        nonlocal cam_K, cam_D
        if cam_K is None:
            cam_K = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            cam_D = np.array(msg.d, dtype=np.float64)

    node.create_subscription(CameraInfo, '/camera/camera/color/camera_info', cb, 10)
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    import time
    t0 = time.time()
    tf_c2b = None
    while time.time() - t0 < 4.0:
        rclpy.spin_once(node, timeout_sec=0.1)
        if cam_K is not None:
            try:
                # 我们需要将 base_link 点转到 camera_color_optical_frame
                tf_c2b = tf_buffer.lookup_transform(
                    'camera_color_optical_frame', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=0.2)
                )
                break
            except Exception:
                pass

    if cam_K is None or tf_c2b is None:
        print("Failed to get camera_info or TF!")
        return

    tr = tf_c2b.transform.translation
    qr = tf_c2b.transform.rotation
    R_b2c = quat_to_rot_mat(qr.x, qr.y, qr.z, qr.w)
    t_b2c = np.array([[tr.x], [tr.y], [tr.z]])
    rvec, _ = cv2.Rodrigues(R_b2c)

    # 在桌面 Z = 0.035m (方块高度平面) 投影坐标网格与轴线:
    # 选取参考原点 P0 = (0.15, 0.0, 0.035), +X 终点 = (0.40, 0.0, 0.035), +Y 终点 = (0.15, 0.25, 0.035)
    # 同时也投影真实底座中心 (0, 0, 0.035) 与当前方块参考点 (0.252, 0.166, 0.035)
    pts_base = np.array([
        [0.00, 0.00, 0.035],   # 0: 机械臂底座中心
        [0.25, 0.00, 0.035],   # 1: +X 方向 (正前方 25cm)
        [0.00, 0.25, 0.035],   # 2: +Y 方向 (正左方 25cm)
        [0.252, 0.166, 0.035], # 3: 参考原点 (当前方块附近)
        [0.352, 0.166, 0.035], # 4: 从方块往 +X 前方 10cm
        [0.252, 0.266, 0.035], # 5: 从方块往 +Y 左方 10cm
    ], dtype=np.float32)

    img_pts, _ = cv2.projectPoints(pts_base, rvec, t_b2c, cam_K, cam_D)
    img_pts = img_pts.reshape(-1, 2).astype(int)

    for fname in ["tag_detected_preview.jpg", "hover_verify_preview.jpg"]:
        path = f"/home/lab203/piper_students/lq/real_arm_control/{fname}"
        img = cv2.imread(path)
        if img is None:
            continue

        p_base0 = tuple(img_pts[0])
        p_base_x = tuple(img_pts[1])
        p_base_y = tuple(img_pts[2])
        p_cube = tuple(img_pts[3])
        p_cube_x = tuple(img_pts[4])
        p_cube_y = tuple(img_pts[5])

        # 画从底座中心出发的大轴线
        cv2.arrowedLine(img, p_base0, p_base_x, (0, 0, 255), 3, tipLength=0.15)
        cv2.arrowedLine(img, p_base0, p_base_y, (0, 255, 0), 3, tipLength=0.15)
        cv2.circle(img, p_base0, 7, (255, 255, 0), -1)
        cv2.putText(img, "Base (0,0)", (p_base0[0] + 10, p_base0[1] + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 0), 2)
        cv2.putText(img, "+X (Front)", (p_base_x[0] - 40, p_base_x[1] + 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(img, "+Y (Left)", (p_base_y[0] + 10, p_base_y[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

        # 在方块附近再画小十字方向指引 (红=+X, 绿=+Y)
        cv2.arrowedLine(img, p_cube, p_cube_x, (0, 0, 255), 3, tipLength=0.25)
        cv2.arrowedLine(img, p_cube, p_cube_y, (0, 255, 0), 3, tipLength=0.25)
        cv2.putText(img, "+X", (p_cube_x[0] + 5, p_cube_x[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 0, 255), 2)
        cv2.putText(img, "+Y", (p_cube_y[0] + 5, p_cube_y[1] + 15), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        out_path = path.replace(".jpg", "_axes.jpg")
        cv2.imwrite(out_path, img)
        print("Saved:", out_path)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
