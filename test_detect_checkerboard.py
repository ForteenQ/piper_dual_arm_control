#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂 - 手眼标定视觉检测测试 (GP290-20-12*9 标定板)
规格: 内部角点 11x8, 单个方格 20.0 mm
"""

import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

# 标定板参数
CHECKERBOARD_CORNERS = (11, 8)
SQUARE_SIZE_METERS = 0.020  # 20.0 mm

class CheckerboardDetector(Node):
    def __init__(self):
        super().__init__('checkerboard_detector')
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_image = None

        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._info_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self._img_cb, 10)
        self.get_logger().info('正在监听 RealSense 相机图像与内参话题...')

    def _info_cb(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape((3, 3))
            self.dist_coeffs = np.array(msg.d)
            self.get_logger().info(f'获取到相机内参矩阵 K:\n{self.camera_matrix}')

    def _img_cb(self, msg: Image):
        try:
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'图像解码失败: {e}')

    def test_detection(self):
        # 等待数据
        start_t = time.time()
        while (self.camera_matrix is None or self.latest_image is None) and (time.time() - start_t < 6.0):
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.latest_image is None:
            self.get_logger().error('❌ 未能获取到相机图像，请检查 RealSense 驱动是否运行')
            return False

        img = self.latest_image.copy()
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape

        # 保存当前视野画面
        cv2.imwrite('current_camera_view.jpg', img)
        self.get_logger().info(f'已抓拍当前画面 (分辨率: {w}x{h})，正在检测 11x8 棋盘格...')

        # 尝试两种角点尺寸 (11, 8) 与 (8, 11)
        patterns = [(11, 8), (8, 11)]
        ret = False
        corners = None
        found_pattern = None

        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

        for pat in patterns:
            # 使用高鲁棒性角点检测算法
            ret, corners = cv2.findChessboardCorners(
                gray, pat, 
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
            )
            if ret:
                found_pattern = pat
                break

        if not ret:
            # 尝试高级 Sector-Based 角点检测器 (OpenCV 4+)
            try:
                ret, corners = cv2.findChessboardCornersSB(gray, (11, 8), cv2.CALIB_CB_NORMALIZE_IMAGE)
                if ret:
                    found_pattern = (11, 8)
            except Exception:
                pass

        if not ret:
            self.get_logger().warn('⚠️ 当前画面未能检测到 11x8 棋盘格角点！')
            self.get_logger().info('可能原因: 标定板未放入视野中、被夹爪严重遮挡、或者光线过暗/反光。当前画面已保存为 current_camera_view.jpg')
            return False

        # 亚像素精细化
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

        # 构造三维物理坐标
        objp = np.zeros((found_pattern[0] * found_pattern[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:found_pattern[0], 0:found_pattern[1]].T.reshape(-1, 2) * SQUARE_SIZE_METERS

        # solvePnP 求解相机到标定板的三维位姿
        success, rvec, tvec = cv2.solvePnP(objp, corners2, self.camera_matrix, self.dist_coeffs)
        if success:
            dist_m = float(np.linalg.norm(tvec))
            self.get_logger().info('====================================================')
            self.get_logger().info(f'✅ 成功精确识别标定板！(匹配规格: {found_pattern[0]}x{found_pattern[1]})')
            self.get_logger().info(f'标定板距相机中心物理距离: {dist_m:.3f} 米 ({dist_m*100:.1f} cm)')
            self.get_logger().info(f'空间平移 tvec [X, Y, Z]: {[round(float(x), 3) for x in tvec.flatten()]} 米')
            self.get_logger().info('====================================================')

            # 绘制角点并保存
            cv2.drawChessboardCorners(img, found_pattern, corners2, ret)
            # 绘制坐标系坐标轴 (长度 0.06m)
            cv2.drawFrameAxes(img, self.camera_matrix, self.dist_coeffs, rvec, tvec, 0.06)
            cv2.imwrite('detected_checkerboard.jpg', img)
            self.get_logger().info('识别结果图已保存为 detected_checkerboard.jpg')
            return True
        else:
            self.get_logger().error('solvePnP 姿态解算失败')
            return False

def main():
    rclpy.init()
    detector = CheckerboardDetector()
    success = detector.test_detection()
    detector.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
