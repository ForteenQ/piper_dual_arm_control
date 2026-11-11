#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag 3D 位姿检测验证脚本 (单次截帧与可视化标记输出)
用于抓取前验证：
1. 相机能否清晰识别桌面上放置的 AprilTag
2. 输出物体在相机系与 base_link 基座系下的绝对 3D 坐标
3. 保存带轴线标记的可视化图片到 tag_detected_preview.jpg
"""
import os
import sys
import time
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration


class TagTestDetector(Node):
    def __init__(self, tag_size=0.050, tag_id=-1):
        super().__init__('tag_test_detector')
        self.tag_size = tag_size
        self.tag_id = tag_id
        self.bridge = CvBridge()

        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_cv_img = None

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        s = self.tag_size / 2.0
        self.obj_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0]
        ], dtype=np.float32)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, 10)

    def info_cb(self, msg):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def img_cb(self, msg):
        try:
            self.latest_cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception:
            pass


def main():
    tag_size = 0.050 # 默认 50mm
    tag_id = -1      # 默认检测视野内任意 ID
    if len(sys.argv) > 1:
        tag_size = float(sys.argv[1]) / 1000.0 # 输入 mm
    if len(sys.argv) > 2:
        tag_id = int(sys.argv[2])

    print("=" * 60)
    print(f"🔍 [AprilTag 验证测试] 正在捕获图像... Tag物理边长: {tag_size*1000:.1f}mm, 目标ID: {tag_id if tag_id >= 0 else '任意'}")
    print("=" * 60)

    rclpy.init()
    node = TagTestDetector(tag_size, tag_id)

    # 等待内参和图像到达
    timeout = 5.0
    start = time.time()
    while rclpy.ok() and (node.camera_matrix is None or node.latest_cv_img is None):
        rclpy.spin_once(node, timeout_sec=0.1)
        if time.time() - start > timeout:
            print("❌ 超时: 未能从 /camera/camera/color 接收到图像或内参！请检查 RealSense 驱动状态。")
            node.destroy_node()
            rclpy.shutdown()
            return

    # 等待 0.5s 让图像稳定
    time.sleep(0.5)
    rclpy.spin_once(node, timeout_sec=0.2)

    img = node.latest_cv_img.copy()
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        detector = cv2.aruco.ArucoDetector(node.aruco_dict, node.aruco_params)
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, node.aruco_dict, parameters=node.aruco_params)

    script_dir = os.path.dirname(os.path.abspath(__file__))
    out_file = os.path.join(script_dir, "tag_detected_preview.jpg")

    if ids is None or len(ids) == 0:
        print("⚠️ 未能在画面中检测到 AprilTag 36h11 标签！")
        print("💡 建议: 请确认标签是否在 RealSense 视野内、光照是否过暗、或有严重反光。")
        cv2.imwrite(out_file, img)
        print(f"📸 原始视野已保存至 {out_file}")
        node.destroy_node()
        rclpy.shutdown()
        return

    print(f"✅ 成功检测到 {len(ids)} 个 AprilTag: {ids.flatten().tolist()}")

    found = False
    for i, tid in enumerate(ids.flatten()):
        if tid == node.tag_id or node.tag_id < 0:
            found = True
            tag_corners = corners[i].reshape((4, 2))
            success, rvec, tvec = cv2.solvePnP(
                node.obj_points, tag_corners, node.camera_matrix, node.dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if success:
                tx, ty, tz = tvec.flatten()
                print(f"\n📌 [目标 Tag ID {tid} - 相机坐标系 Camera Frame]")
                print(f"   X (水平右): {tx*1000:+.1f} mm")
                print(f"   Y (垂直下): {ty*1000:+.1f} mm")
                print(f"   Z (纵深前): {tz*1000:+.1f} mm")

                # 绘制 3D 坐标轴与边界
                cv2.aruco.drawDetectedMarkers(img, corners)
                cv2.drawFrameAxes(img, node.camera_matrix, node.dist_coeffs, rvec, tvec, node.tag_size * 0.8)

                # 坐标变换到 base_link (stamp=0 表示使用最新的静态 TF 变换)
                pose_cam = PoseStamped()
                pose_cam.header.stamp = rclpy.time.Time().to_msg()
                pose_cam.header.frame_id = 'camera_color_optical_frame'
                pose_cam.pose.position.x = float(tx)
                pose_cam.pose.position.y = float(ty)
                pose_cam.pose.position.z = float(tz)
                pose_cam.pose.orientation.w = 1.0

                # 等待 TF
                time.sleep(0.3)
                try:
                    pose_base = node.tf_buffer.transform(pose_cam, 'base_link', timeout=Duration(seconds=1.0))
                    bx = pose_base.pose.position.x
                    by = pose_base.pose.position.y
                    bz = pose_base.pose.position.z
                    print(f"\n🎯 [目标 Tag ID {tid} - 机械臂基座坐标系 Base Frame]")
                    print(f"   X_base (前方): {bx*1000:+.1f} mm")
                    print(f"   Y_base (左方): {by*1000:+.1f} mm")
                    print(f"   Z_base (上方): {bz*1000:+.1f} mm")

                    r_horizontal = np.hypot(bx, by)
                    dist = np.sqrt(bx**2 + by**2 + bz**2)
                    print(f"   水平投影距离: {r_horizontal*1000:.1f} mm")
                    print(f"   空间直线距离: {dist*1000:.1f} mm")
                    if 0.15 <= r_horizontal <= 0.48 and -0.15 <= bz <= 0.35:
                        print("   ✨ 状态: 处于 PIPER 机械臂完美可达抓取工作空间 (Reachability: OK)")
                    else:
                        print("   ⚠️ 警告: 物体可能接近或超出机械臂工作空间极限，请移至机械臂正前方 25~40cm！")
                except Exception as e:
                    print(f"⚠️ 坐标变换失败 (TF base_link -> camera 尚未启动或超时): {e}")

                break

    cv2.imwrite(out_file, img)
    print(f"\n🖼️ 带 3D 轴线标注的检测效果图已保存至: {out_file}")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
