#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
GP290 陶瓷标定板实时 TF 广播节点 (专门适配 easy_handeye2)
标定板规格: 11x8 内角点, 20.0mm 边长
发布 TF: camera_color_optical_frame -> checkerboard
"""

import sys
import math
import numpy as np
import cv2

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros

CHECKERBOARD_CORNERS = (11, 8)
SQUARE_SIZE_M = 0.020  # 20.0 mm


def rvec_to_quaternion(rvec):
    angle = float(np.linalg.norm(rvec))
    if angle < 1e-7:
        return 0.0, 0.0, 0.0, 1.0
    axis = rvec.flatten() / angle
    half = angle * 0.5
    s = math.sin(half)
    return float(axis[0] * s), float(axis[1] * s), float(axis[2] * s), float(math.cos(half))


class CheckerboardTfBroadcaster(Node):
    def __init__(self):
        super().__init__('checkerboard_tf_broadcaster')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.camera_matrix = None
        self.dist_coeffs = None
        self.optical_frame_id = 'camera_color_optical_frame'

        # 3D 标定板物体物理坐标
        self.objp = np.zeros((CHECKERBOARD_CORNERS[0] * CHECKERBOARD_CORNERS[1], 3), np.float32)
        self.objp[:, :2] = np.mgrid[0:CHECKERBOARD_CORNERS[0], 0:CHECKERBOARD_CORNERS[1]].T.reshape(-1, 2) * SQUARE_SIZE_M

        self.is_processing = False
        self.last_log_time = 0.0

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_callback, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, qos)

        self.get_logger().info('Checkerboard TF 广播节点已就绪 (QoS Depth=1, 零排队)...')

    def info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            if msg.header.frame_id:
                self.optical_frame_id = msg.header.frame_id
            self.get_logger().info(f'相机内参已锁定: frame_id={self.optical_frame_id}, fx={self.camera_matrix[0,0]:.1f}')

    def image_callback(self, msg: Image):
        # 严格防积压：如果上一帧还在解算，直接跳过当前帧，保持 100% 绝对实时性
        if self.is_processing or self.camera_matrix is None:
            return

        self.is_processing = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            ret, corners = cv2.findChessboardCorners(
                gray, CHECKERBOARD_CORNERS,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
            )

            if ret:
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.01)
                corners2 = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
                pnp_ret, rvec, tvec = cv2.solvePnP(self.objp, corners2, self.camera_matrix, self.dist_coeffs)

                if pnp_ret:
                    qx, qy, qz, qw = rvec_to_quaternion(rvec)

                    t = TransformStamped()
                    t.header.stamp = msg.header.stamp
                    t.header.frame_id = self.optical_frame_id
                    t.child_frame_id = 'checkerboard'

                    t.transform.translation.x = float(tvec[0, 0])
                    t.transform.translation.y = float(tvec[1, 0])
                    t.transform.translation.z = float(tvec[2, 0])
                    t.transform.rotation.x = qx
                    t.transform.rotation.y = qy
                    t.transform.rotation.z = qz
                    t.transform.rotation.w = qw

                    self.tf_broadcaster.sendTransform(t)

                    now = self.get_clock().now().nanoseconds * 1e-9
                    if now - self.last_log_time >= 2.0:
                        dist = float(np.linalg.norm(tvec))
                        self.get_logger().info(f'🎯 标定板持续稳定锁定中: 距离相机 {dist*100.0:.1f} cm -> 发布 TF 到 /tf')
                        self.last_log_time = now

        except Exception as e:
            self.get_logger().error(f'图像解算异常: {e}')
        finally:
            self.is_processing = False


def main():
    rclpy.init()
    node = CheckerboardTfBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
