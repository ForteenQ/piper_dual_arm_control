#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 机械臂 × RealSense D435 桌面目标 3D 位姿估计节点 (AprilTag 36h11)
功能:
1. 订阅 RealSense 彩色图与相机内参
2. 基于 OpenCV Aruco (DICT_APRILTAG_36h11) 检测靶标角点并使用 solvePnP 解算相机系 6-DOF 位姿
3. 借助 TF2 监听 base_link -> camera_color_optical_frame 将物体位姿自动变换至基座坐标系
4. 对检测位姿进行滑动窗口滤波 (平滑消除抖动与跳变)
5. 高频广播 detected_object TF 并发布 /detected_object_pose 话题
6. 支持网页/图片查看当前检测渲染帧

作者: 203实验室 双臂抛接课题组
域ID: ROS_DOMAIN_ID=42
"""

import os
import sys
import time
import math
import collections
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs


def rotation_matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """3x3 旋转矩阵转换为四元数 [x, y, z, w]"""
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    q = np.array([qx, qy, qz, qw], dtype=float)
    return q / np.linalg.norm(q)


class AprilTagPoseEstimator(Node):
    def __init__(self):
        super().__init__('apriltag_pose_estimator')

        # 参数声明
        self.declare_parameter('tag_family', 'tag36h11')
        self.declare_parameter('tag_id', 0)                  # 靶标 ID (-1 表示追踪视野内检测到的第一个)
        self.declare_parameter('tag_size', 0.050)            # 靶标物理实际边长 (米)，默认 50mm (0.05m)
        self.declare_parameter('target_frame', 'base_link')
        self.declare_parameter('camera_frame', 'camera_color_optical_frame')

        self.tag_id = self.get_parameter('tag_id').value
        self.tag_size = float(self.get_parameter('tag_size').value)
        self.target_frame = self.get_parameter('target_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value

        self.get_logger().info(
            f"🎯 启动 AprilTag 3D 位姿估计节点 | 目标 Tag ID: {self.tag_id} | 边长: {self.tag_size*1000:.1f} mm | 目标基座帧: {self.target_frame}"
        )

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None

        # AprilTag 字典与参数 (兼容 OpenCV 4.5 与 4.7+)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        if hasattr(cv2.aruco, 'DetectorParameters_create'):
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        else:
            self.aruco_params = cv2.aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.aruco_detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params) if hasattr(cv2.aruco, 'ArucoDetector') else None

        # 3D 标定点定义 (物体局部坐标系中心位于 Tag 中心)
        s = self.tag_size / 2.0
        self.obj_points = np.array([
            [-s,  s, 0.0],
            [ s,  s, 0.0],
            [ s, -s, 0.0],
            [-s, -s, 0.0]
        ], dtype=np.float32)

        # TF2 监听与广播
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 结果发布者
        self.pose_pub = self.create_publisher(PoseStamped, '/detected_object_pose', 10)

        # 滑动窗口平滑滤波 (长度为 10)
        self.pos_history = collections.deque(maxlen=10)

        # 订阅相机流
        self.info_sub = self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        self.image_sub = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.image_callback,
            1
        )

        self.last_log_time = 0.0

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d, dtype=np.float64)
            self.get_logger().info(f"📸 成功锁定相机内参: fx={self.camera_matrix[0,0]:.1f}, fy={self.camera_matrix[1,1]:.1f}")

    def image_callback(self, msg: Image):
        if self.camera_matrix is None:
            return

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"图像转换失败: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        if self.aruco_detector is not None:
            corners, ids, rejected = self.aruco_detector.detectMarkers(gray)
        else:
            corners, ids, rejected = cv2.aruco.detectMarkers(
                gray, self.aruco_dict, parameters=self.aruco_params
            )

        if ids is None or len(ids) == 0:
            return

        matched_idx = -1
        for i, tid in enumerate(ids.flatten()):
            if self.tag_id < 0 or tid == self.tag_id:
                matched_idx = i
                break

        if matched_idx == -1:
            return

        # 提取角点并解算 PnP 位姿
        tag_corners = corners[matched_idx].reshape((4, 2))
        success, rvec, tvec = cv2.solvePnP(
            self.obj_points,
            tag_corners,
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if not success:
            return

        tx, ty, tz = tvec.flatten()
        R, _ = cv2.Rodrigues(rvec)
        quat = rotation_matrix_to_quaternion(R)

        # 构造相机坐标系下的 PoseStamped
        pose_cam = PoseStamped()
        pose_cam.header.stamp = msg.header.stamp
        pose_cam.header.frame_id = self.camera_frame
        pose_cam.pose.position.x = float(tx)
        pose_cam.pose.position.y = float(ty)
        pose_cam.pose.position.z = float(tz)
        pose_cam.pose.orientation.x = float(quat[0])
        pose_cam.pose.orientation.y = float(quat[1])
        pose_cam.pose.orientation.z = float(quat[2])
        pose_cam.pose.orientation.w = float(quat[3])

        # 通过 TF2 将坐标变换到基座 base_link
        try:
            # 查找并执行位姿坐标变换
            pose_base = self.tf_buffer.transform(
                pose_cam,
                self.target_frame,
                timeout=Duration(seconds=0.05)
            )
        except Exception as ex:
            # 等待 TF 树稳定
            return

        # 滑动窗口平滑滤波
        raw_pos = np.array([
            pose_base.pose.position.x,
            pose_base.pose.position.y,
            pose_base.pose.position.z
        ])
        self.pos_history.append(raw_pos)
        smooth_pos = np.median(np.array(self.pos_history), axis=0)

        pose_base.pose.position.x = float(smooth_pos[0])
        pose_base.pose.position.y = float(smooth_pos[1])
        pose_base.pose.position.z = float(smooth_pos[2])

        # 发布基座系下的目标位姿话题
        self.pose_pub.publish(pose_base)

        # 广播 detected_object 动态 TF 供 RViz 可视化
        tf_msg = TransformStamped()
        tf_msg.header.stamp = msg.header.stamp
        tf_msg.header.frame_id = self.target_frame
        tf_msg.child_frame_id = 'detected_object'
        tf_msg.transform.translation.x = pose_base.pose.position.x
        tf_msg.transform.translation.y = pose_base.pose.position.y
        tf_msg.transform.translation.z = pose_base.pose.position.z
        tf_msg.transform.rotation = pose_base.pose.orientation
        self.tf_broadcaster.sendTransform(tf_msg)

        # 周期性日志打印 (约 2Hz)
        now = time.time()
        if now - self.last_log_time > 0.5:
            self.last_log_time = now
            self.get_logger().info(
                f"🎯 [基座坐标系] X={smooth_pos[0]:.4f}m, Y={smooth_pos[1]:.4f}m, Z={smooth_pos[2]:.4f}m | "
                f"相机距: {tz:.3f}m"
            )


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagPoseEstimator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
