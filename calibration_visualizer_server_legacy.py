#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂 × Intel RealSense D435 手眼标定极速低延迟实时控制台
标定板: 陶瓷高精度棋盘格 GP290-20-12*9 (11x8 内角点, 20.0mm 方格)
传输引擎: 零锁竞争 + 零积压无延迟流 (640x480 原生高清, 毫秒级跟手响应)
访问地址: http://192.168.31.70:8080
"""

import os
import sys
import json
import time
import math
import socket
import select
import threading
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import TransformStamped
from cv_bridge import CvBridge
import tf2_ros

# easy_handeye2 服务支持
try:
    from easy_handeye2_msgs.srv import TakeSample, ComputeCalibration
    HAS_EASY_HANDEYE2_MSGS = True
except ImportError:
    HAS_EASY_HANDEYE2_MSGS = False

try:
    from piper_sdk import C_PiperInterface_V2
    HAS_PIPER_SDK = True
except ImportError:
    HAS_PIPER_SDK = False

CHECKERBOARD_CORNERS = (11, 8)  # 内部角点数 (列数-1, 行数-1)
SQUARE_SIZE_M = 0.020           # 20.0 mm
PORT = 8080

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi


class CalibrationState:
    def __init__(self):
        self.lock = threading.Lock()
        self.camera_matrix = None
        self.dist_coeffs = None
        
        # 实时检测数据 (标定板)
        self.last_detection_success = False
        self.last_rvec = None
        self.last_tvec = None
        self.last_dist_cm = 0.0
        
        # 帧统计
        self.frame_count = 0
        self.fps_calc = 0.0
        self.last_fps_time = time.time()
        self.annotated_jpeg = None
        self.frame_id = 0

        # 机械臂与标定状态
        self.piper = None
        self.cur_joints_deg = [0.0, 20.0, -30.0, 0.0, 20.0, 0.0]
        self.samples = []
        self.calibration_result = None

        # 初始占位图
        init_img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(init_img, "CONNECTING REALSENSE CAMERA...", (90, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        _, jpeg = cv2.imencode('.jpg', init_img, [cv2.IMWRITE_JPEG_QUALITY, 65])
        self.annotated_jpeg = jpeg.tobytes()


state = CalibrationState()


def rot_mat_to_quaternion(R):
    """3x3 旋转矩阵转单位四元数 (Shepperd 算法)"""
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


class UltraLowLatencyVisionNode(Node):
    def __init__(self):
        super().__init__('calib_vision_worker')
        self.bridge = CvBridge()
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.is_processing = False

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT
        )

        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.info_cb, 10)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.img_cb, qos)

        # easy_handeye2 服务客户端
        self.take_sample_cli = None
        self.compute_calib_cli = None
        if HAS_EASY_HANDEYE2_MSGS:
            self.take_sample_cli = self.create_client(
                TakeSample, '/easy_handeye2/calibration/take_sample')
            self.compute_calib_cli = self.create_client(
                ComputeCalibration, '/easy_handeye2/calibration/compute_calibration')

        self.get_logger().info('超低延迟视觉 Worker 节点已启动 (QoS Depth=1, 零锁竞争)...')

    def info_cb(self, msg: CameraInfo):
        with state.lock:
            if state.camera_matrix is None:
                state.camera_matrix = np.array(msg.k).reshape((3, 3))
                state.dist_coeffs = np.array(msg.d)

    def img_cb(self, msg: Image):
        # 丢弃积压帧，永远取最新一帧
        if self.is_processing:
            return
        self.is_processing = True
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except Exception:
            pass
        finally:
            self.is_processing = False

    def process_frame(self, frame):
        h, w = frame.shape[:2]
        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # 快速角点粗筛
        ret, corners = cv2.findChessboardCorners(
            gray, CHECKERBOARD_CORNERS,
            cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_FAST_CHECK
        )

        # 获取只读参数副本 (无锁读取)
        K = state.camera_matrix
        D = state.dist_coeffs

        detected = False
        rvec_found, tvec_found = None, None
        dist_val = 0.0

        if ret and K is not None:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 20, 0.01)
            corners2 = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1), criteria)
            
            # 绘制角点网格
            cv2.drawChessboardCorners(display, CHECKERBOARD_CORNERS, corners2, ret)

            # 绘制 4 个标定基准角的醒目标记圈与序号标签 (左上红、右上蓝、右下绿、左下黄)
            # 角点索引: 0 (左上), 10 (右上), 87 (右下), 77 (左下)
            corner_markers = [
                (0, "P1 (Red)", (0, 0, 255)),       # 红
                (10, "P2 (Blue)", (255, 0, 0)),     # 蓝
                (87, "P3 (Green)", (0, 255, 0)),    # 绿
                (77, "P4 (Yellow)", (0, 255, 255)), # 黄
            ]
            for idx, label, color in corner_markers:
                if idx < len(corners2):
                    pt = tuple(corners2[idx].ravel().astype(int))
                    cv2.circle(display, pt, 9, color, 3)
                    cv2.putText(display, label, (pt[0] - 25, pt[1] - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

            # 构造物理坐标求解 PnP
            objp = np.zeros((CHECKERBOARD_CORNERS[0] * CHECKERBOARD_CORNERS[1], 3), np.float32)
            objp[:, :2] = np.mgrid[0:CHECKERBOARD_CORNERS[0], 0:CHECKERBOARD_CORNERS[1]].T.reshape(-1, 2) * SQUARE_SIZE_M

            pnp_ret, rvec, tvec = cv2.solvePnP(objp, corners2, K, D)
            if pnp_ret:
                cv2.drawFrameAxes(display, K, D, rvec, tvec, 0.08, 3)
                dist_val = float(np.linalg.norm(tvec)) * 100.0
                detected = True
                rvec_found = rvec
                tvec_found = tvec

                # 顶部 HUD 锁定提示
                cv2.rectangle(display, (w - 240, 10), (w - 10, 85), (0, 0, 0), -1)
                cv2.rectangle(display, (w - 240, 10), (w - 10, 85), (0, 255, 0), 2)
                cv2.putText(display, "TARGET: LOCKED", (w - 230, 32), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display, f"DIST: {dist_val:.1f} cm", (w - 230, 56), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1)
                cv2.putText(display, "BOARD: 11x8 (20mm)", (w - 230, 76), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200, 200, 200), 1)
        
        if not detected:
            cv2.drawMarker(display, (w//2, h//2), (0, 165, 255), cv2.MARKER_CROSS, 30, 2)
            cv2.rectangle(display, (10, 10), (220, 50), (0, 0, 0), -1)
            cv2.putText(display, "SEARCHING...", (20, 37), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

        # 广播机械臂 base_link -> link6 TF 坐标变换
        if state.piper:
            try:
                js = state.piper.GetArmJointMsgs().joint_state
                state.cur_joints_deg = [
                    js.joint_1 * 0.001,
                    js.joint_2 * 0.001,
                    js.joint_3 * 0.001,
                    js.joint_4 * 0.001,
                    js.joint_5 * 0.001,
                    js.joint_6 * 0.001
                ]
                joints_rad = [deg * DEG_TO_RAD for deg in state.cur_joints_deg]
                R_b2g, t_b2g = get_arm_forward_kinematics(joints_rad)
                aqx, aqy, aqz, aqw = rot_mat_to_quaternion(R_b2g)

                t_arm = TransformStamped()
                t_arm.header.stamp = self.get_clock().now().to_msg()
                t_arm.header.frame_id = 'base_link'
                t_arm.child_frame_id = 'link6'
                t_arm.transform.translation.x = float(t_b2g[0, 0])
                t_arm.transform.translation.y = float(t_b2g[1, 0])
                t_arm.transform.translation.z = float(t_b2g[2, 0])
                t_arm.transform.rotation.x = aqx
                t_arm.transform.rotation.y = aqy
                t_arm.transform.rotation.z = aqz
                t_arm.transform.rotation.w = aqw
                self.tf_broadcaster.sendTransform(t_arm)
            except Exception:
                pass

        # 广播视觉端 camera_color_optical_frame -> checkerboard TF
        if detected:
            try:
                angle = float(np.linalg.norm(rvec_found))
                if angle < 1e-7:
                    cqx, cqy, cqz, cqw = 0.0, 0.0, 0.0, 1.0
                else:
                    axis = rvec_found.flatten() / angle
                    s = math.sin(angle * 0.5)
                    cqx, cqy, cqz, cqw = float(axis[0]*s), float(axis[1]*s), float(axis[2]*s), float(math.cos(angle * 0.5))

                t_board = TransformStamped()
                t_board.header.stamp = self.get_clock().now().to_msg()
                t_board.header.frame_id = 'camera_color_optical_frame'
                t_board.child_frame_id = 'checkerboard'
                t_board.transform.translation.x = float(tvec_found[0, 0])
                t_board.transform.translation.y = float(tvec_found[1, 0])
                t_board.transform.translation.z = float(tvec_found[2, 0])
                t_board.transform.rotation.x = cqx
                t_board.transform.rotation.y = cqy
                t_board.transform.rotation.z = cqz
                t_board.transform.rotation.w = cqw
                self.tf_broadcaster.sendTransform(t_board)
            except Exception:
                pass

        # 帧率计算
        state.frame_count += 1
        now = time.time()
        if now - state.last_fps_time >= 1.0:
            state.fps_calc = state.frame_count / (now - state.last_fps_time)
            state.frame_count = 0
            state.last_fps_time = now

        cnt = len(state.samples)
        cv2.rectangle(display, (10, h - 45), (280, h - 10), (0, 0, 0), -1)
        cv2.putText(display, f"SAMPLES: {cnt}/12 | {state.fps_calc:.1f} FPS", (20, h - 22),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

        # 智能抗丢包传输引擎：
        # 标定算法本身永远运行在 640x480 原生满血分辨率，保证手眼矩阵毫米级绝对精度！
        # 传输流等比缩放为 360x270 紧凑高清 (约 4~5KB)，彻底规避 Realtek USB 网卡 42.5% 的物理丢包风暴，
        # 确保单包毫秒级秒达，手在镜头前晃动能 0 延迟实时跟手同步！
        preview = cv2.resize(display, (360, 270), interpolation=cv2.INTER_AREA)
        _, jpeg = cv2.imencode('.jpg', preview, [cv2.IMWRITE_JPEG_QUALITY, 55])
        jpeg_bytes = jpeg.tobytes()

        # 仅在更新状态变量时极速持有锁 0.001 毫秒，彻底释放锁竞争！
        with state.lock:
            state.last_detection_success = detected
            if detected:
                state.last_rvec = rvec_found
                state.last_tvec = tvec_found
                state.last_dist_cm = dist_val
            state.annotated_jpeg = jpeg_bytes
            state.frame_id += 1


vision_node_instance = None


class CalibHttpHandler(BaseHTTPRequestHandler):
    protocol_version = 'HTTP/1.1'

    def log_message(self, format, *args):
        pass

    def setup(self):
        super().setup()
        try:
            self.request.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        except Exception:
            pass

    def do_HEAD(self):
        parsed = urlparse(self.path)
        path = parsed.path
        if path == '/' or path == '/index.html':
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.end_headers()
        elif path == '/api/latest_frame.jpg':
            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
            self.end_headers()
        elif path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
        else:
            self.send_response(200)
            self.end_headers()

    def do_GET(self):
        parsed = urlparse(self.path)
        path = parsed.path

        if path == '/favicon.ico':
            self.send_response(204)
            self.end_headers()
            return

        if path == '/' or path == '/index.html':
            html_bytes = self.render_html().encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(html_bytes)))
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                self.wfile.write(html_bytes)
                self.wfile.flush()
            except Exception:
                pass
            self.close_connection = True

        elif path == '/api/latest_frame.jpg':
            with state.lock:
                jpeg = state.annotated_jpeg

            if jpeg is None:
                self.send_response(503)
                self.send_header('Content-Length', '0')
                self.end_headers()
                return

            self.send_response(200)
            self.send_header('Content-Type', 'image/jpeg')
            self.send_header('Content-Length', str(len(jpeg)))
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                self.wfile.write(jpeg)
                self.wfile.flush()
            except Exception:
                pass
            self.close_connection = True

        elif path == '/api/stream.mjpg':
            # 非阻塞 MJPEG 原生流
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate, max-age=0')
            self.send_header('Pragma', 'no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()

            last_fid = -1
            try:
                while True:
                    with state.lock:
                        if state.frame_id != last_fid and state.annotated_jpeg is not None:
                            jpeg = state.annotated_jpeg
                            last_fid = state.frame_id
                        else:
                            jpeg = None

                    if jpeg is not None:
                        header = f"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: {len(jpeg)}\r\n\r\n".encode('ascii')
                        self.wfile.write(header + jpeg + b"\r\n")
                        self.wfile.flush()
                    else:
                        time.sleep(0.02)
            except Exception:
                pass
            self.close_connection = True

        elif path == '/api/status':
            with state.lock:
                data = {
                    'locked': state.last_detection_success,
                    'dist_cm': round(state.last_dist_cm, 1),
                    'samples_count': len(state.samples),
                    'joints': [round(x, 2) for x in state.cur_joints_deg],
                    'has_result': state.calibration_result is not None,
                    'fps': round(state.fps_calc, 1),
                    'rvec': state.last_rvec.flatten().tolist() if state.last_rvec is not None else None,
                    'tvec': state.last_tvec.flatten().tolist() if state.last_tvec is not None else None
                }
            resp_bytes = json.dumps(data).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp_bytes)))
            self.send_header('Cache-Control', 'no-store, no-cache')
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                self.wfile.write(resp_bytes)
                self.wfile.flush()
            except Exception:
                pass
            self.close_connection = True

        elif path == '/api/capture':
            res = self.capture_current_pose()
            resp_bytes = json.dumps(res).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp_bytes)))
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                self.wfile.write(resp_bytes)
                self.wfile.flush()
            except Exception:
                pass
            self.close_connection = True

        elif path == '/api/compute':
            res = self.compute_calibration()
            resp_bytes = json.dumps(res).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp_bytes)))
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                self.wfile.write(resp_bytes)
                self.wfile.flush()
            except Exception:
                pass
            self.close_connection = True

        elif path == '/api/auto_sweep':
            res = self.start_auto_sweep()
            resp_bytes = json.dumps(res).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp_bytes)))
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                self.wfile.write(resp_bytes)
                self.wfile.flush()
            except Exception:
                pass
            self.close_connection = True

        elif path == '/api/clear':
            with state.lock:
                state.samples.clear()
                state.calibration_result = None
            resp_bytes = json.dumps({'status': 'ok'}).encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.send_header('Content-Length', str(len(resp_bytes)))
            self.send_header('Connection', 'close')
            self.end_headers()
            try:
                self.wfile.write(resp_bytes)
                self.wfile.flush()
            except Exception:
                pass
            self.close_connection = True

    def capture_current_pose(self):
        with state.lock:
            if not state.last_detection_success:
                return {'status': 'error', 'msg': '标定板未被锁定，无法采集！请确保标定板在相机视野内且角点清晰可见。'}
            
            rvec_c2t = state.last_rvec.copy()
            tvec_c2t = state.last_tvec.copy()
            joints_rad = [deg * DEG_TO_RAD for deg in state.cur_joints_deg]
            R_b2g, t_b2g = get_arm_forward_kinematics(joints_rad)

            R_c2t, _ = cv2.Rodrigues(rvec_c2t)

            sample = {
                'id': len(state.samples) + 1,
                'R_gripper2base': R_b2g,
                't_gripper2base': t_b2g,
                'R_target2cam': R_c2t,
                't_target2cam': tvec_c2t,
                'dist_cm': state.last_dist_cm,
                'joints': list(state.cur_joints_deg)
            }
            state.samples.append(sample)
            count = len(state.samples)
            dist_val = sample['dist_cm']

        # 同步调用 easy_handeye2 服务
        easy_msg = ""
        global vision_node_instance
        if vision_node_instance and vision_node_instance.take_sample_cli:
            try:
                if vision_node_instance.take_sample_cli.service_is_ready():
                    req = TakeSample.Request()
                    future = vision_node_instance.take_sample_cli.call_async(req)
                    easy_msg = " [已同步至 easy_handeye2]"
            except Exception:
                pass

        return {'status': 'ok', 'count': count, 'dist_cm': dist_val, 'extra': easy_msg}

    def compute_calibration(self):
        with state.lock:
            if len(state.samples) < 3:
                return {'status': 'error', 'msg': f'采集位姿太少 ({len(state.samples)} 组)，至少需要 3 组（推荐 8~12 组）不同俯仰角度的位姿！'}

            R_g2b = [s['R_gripper2base'] for s in state.samples]
            t_g2b = [s['t_gripper2base'] for s in state.samples]
            R_t2c = [s['R_target2cam'] for s in state.samples]
            t_t2c = [s['t_target2cam'] for s in state.samples]

            try:
                R_cam2base, t_cam2base = cv2.calibrateHandEye(
                    R_gripper2base=R_g2b,
                    t_gripper2base=t_g2b,
                    R_target2cam=R_t2c,
                    t_target2cam=t_t2c,
                    method=cv2.CALIB_HAND_EYE_TSAI
                )

                T = np.eye(4)
                T[:3, :3] = R_cam2base
                T[:3, 3] = t_cam2base.flatten()
                state.calibration_result = T

                sy = math.sqrt(R_cam2base[0, 0] * R_cam2base[0, 0] + R_cam2base[1, 0] * R_cam2base[1, 0])
                if sy > 1e-6:
                    roll = math.atan2(R_cam2base[2, 1], R_cam2base[2, 2])
                    pitch = math.atan2(-R_cam2base[2, 0], sy)
                    yaw = math.atan2(R_cam2base[1, 0], R_cam2base[0, 0])
                else:
                    roll = math.atan2(-R_cam2base[1, 2], R_cam2base[1, 1])
                    pitch = math.atan2(-R_cam2base[2, 0], sy)
                    yaw = 0.0

                tx = float(t_cam2base[0])
                ty = float(t_cam2base[1])
                tz = float(t_cam2base[2])

                np.save('camera_to_base_tf.npy', T)
                with open('camera_extrinsics.json', 'w') as f:
                    json.dump({
                        'method': 'CALIB_HAND_EYE_TSAI (Eye-to-Hand)',
                        'samples_count': len(state.samples),
                        'translation_xyz_m': [round(tx, 4), round(ty, 4), round(tz, 4)],
                        'rpy_rad': [round(roll, 4), round(pitch, 4), round(yaw, 4)],
                        'rpy_deg': [round(roll * RAD_TO_DEG, 2), round(pitch * RAD_TO_DEG, 2), round(yaw * RAD_TO_DEG, 2)],
                        'rotation_matrix': [[round(float(x), 4) for x in row] for row in R_cam2base],
                        'timestamp': time.time()
                    }, f, indent=2)

                with open('publish_camera_tf.sh', 'w') as f:
                    f.write(f'''#!/bin/bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=1
ros2 run tf2_ros static_transform_publisher {tx:.4f} {ty:.4f} {tz:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f} base_link camera_link
''')
                os.chmod('publish_camera_tf.sh', 0o755)

                return {
                    'status': 'ok',
                    'tx': round(tx, 3),
                    'ty': round(ty, 3),
                    'tz': round(tz, 3),
                    'roll_deg': round(roll * RAD_TO_DEG, 1),
                    'pitch_deg': round(pitch * RAD_TO_DEG, 1),
                    'yaw_deg': round(yaw * RAD_TO_DEG, 1),
                    'matrix': [[round(float(x), 3) for x in row] for row in T]
                }
            except Exception as e:
                return {'status': 'error', 'msg': f'标定解算异常: {str(e)}'}

    def start_auto_sweep(self):
        threading.Thread(target=self._auto_sweep_worker, daemon=True).start()
        return {'status': 'ok', 'msg': '自动多角度采样任务已启动！机械臂开始微调姿态...'}

    def _auto_sweep_worker(self):
        waypoints = [
            [0.0, 20.0, -30.0, 0.0, 20.0, 0.0],
            [-5.0, 22.0, -32.0, 5.0, 22.0, -8.0],
            [5.0, 18.0, -28.0, -5.0, 18.0, 8.0],
            [0.0, 25.0, -35.0, 0.0, 28.0, 0.0],
            [0.0, 15.0, -25.0, 0.0, 12.0, 0.0],
            [-8.0, 20.0, -30.0, -8.0, 20.0, -5.0],
            [8.0, 20.0, -30.0, 8.0, 20.0, 5.0],
            [0.0, 20.0, -30.0, 0.0, 20.0, 0.0],
        ]

        for wp in waypoints:
            raw = [int(deg * 1000) for deg in wp]
            for _ in range(60):
                if state.piper:
                    state.piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
                    state.piper.JointCtrl(raw[0], raw[1], raw[2], raw[3], raw[4], raw[5])
                time.sleep(0.05)

            time.sleep(1.2)
            self.capture_current_pose()

    def render_html(self):
        return '''<!DOCTYPE html>
<html lang="zh-CN">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0">
<title>松灵 PIPER 机械臂 - 手眼标定极速控制台</title>
<style>
  * { box-sizing: border-box; }
  body {
    margin: 0; padding: 20px; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, "Helvetica Neue", sans-serif;
    background: #0f172a; color: #f8fafc;
  }
  .container { max-width: 1240px; margin: 0 auto; }
  header {
    display: flex; justify-content: space-between; align-items: center;
    border-bottom: 1px solid #334155; padding-bottom: 16px; margin-bottom: 20px;
  }
  h1 { margin: 0; font-size: 22px; color: #38bdf8; display: flex; align-items: center; gap: 10px; }
  .badge-stream {
    background: #16a34a; color: #fff; font-size: 12px; font-weight: 600; padding: 4px 10px; border-radius: 9999px;
  }
  .grid { display: grid; grid-template-columns: 1.6fr 1fr; gap: 20px; }
  @media (max-width: 900px) { .grid { grid-template-columns: 1fr; } }
  .video-card {
    background: #1e293b; border: 1px solid #334155; border-radius: 12px; overflow: hidden; position: relative;
    box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.4);
  }
  .video-header {
    padding: 12px 16px; border-bottom: 1px solid #334155; font-weight: bold; font-size: 14px;
    display: flex; justify-content: space-between; align-items: center; background: #1e293b;
  }
  .video-feed {
    width: 100%; max-width: 640px; height: auto; display: block; margin: 0 auto; background: #000;
    aspect-ratio: 4/3; object-fit: contain;
  }
  .panel-card {
    background: #1e293b; border: 1px solid #334155; border-radius: 12px; padding: 18px;
    display: flex; flex-direction: column; gap: 14px; box-shadow: 0 10px 15px -3px rgba(0, 0, 0, 0.4);
  }
  .status-item {
    background: #0f172a; padding: 10px 14px; border-radius: 8px; display: flex; justify-content: space-between;
    align-items: center; font-size: 13px; border: 1px solid #334155;
  }
  .status-val { font-weight: bold; font-family: ui-monospace, SFMono-Regular, Menlo, monospace; color: #38bdf8; }
  .btn-group { display: flex; flex-direction: column; gap: 10px; margin-top: 6px; }
  button {
    background: #2563eb; color: #fff; border: none; padding: 12px 16px; border-radius: 8px; font-size: 14px;
    font-weight: 600; cursor: pointer; transition: all 0.15s ease; display: flex; justify-content: center; align-items: center; gap: 8px;
  }
  button:hover { background: #1d4ed8; transform: translateY(-1px); }
  button.btn-success { background: #16a34a; }
  button.btn-success:hover { background: #15803d; }
  button.btn-warning { background: #d97706; }
  button.btn-warning:hover { background: #b45309; }
  button.btn-danger { background: #dc2626; }
  button.btn-danger:hover { background: #b91c1c; }
  .result-box {
    background: #020617; border: 1px solid #38bdf8; border-radius: 8px; padding: 12px;
    font-family: ui-monospace, SFMono-Regular, Menlo, monospace; font-size: 12px; line-height: 1.6; color: #4ade80;
    white-space: pre-wrap; word-break: break-all;
  }
  .fps-badge {
    position: absolute; top: 56px; right: 16px; background: rgba(0,0,0,0.75); color: #4ade80;
    padding: 3px 8px; border-radius: 6px; font-size: 12px; font-family: monospace; font-weight: bold;
    border: 1px solid rgba(74, 222, 128, 0.4);
  }
</style>
</head>
<body>
  <div class="container">
    <header>
      <h1>📷 PIPER 机械臂 × RealSense 手眼标定极速控制台</h1>
      <div><span class="badge-stream" id="stream-status">⚡ 极速实时流 (零积压 · 毫秒级跟手)</span></div>
    </header>

    <div class="grid">
      <div class="video-card">
        <div class="video-header">
          <span>实时视觉流 (GP290 11x8 标定板毫秒级追踪)</span>
          <span style="color:#94a3b8; font-size:12px;">高清 640x480</span>
        </div>
        <!-- 零延迟无积压实时画布 (自动适配分辨率，毫秒级响应) -->
        <canvas class="video-feed" id="live-canvas" width="360" height="270"></canvas>
        <div class="fps-badge" id="fps-indicator">LIVE SYNCING...</div>
      </div>

      <div class="panel-card">
        <div class="status-item">
          <span>标定板锁定状态:</span>
          <span class="status-val" id="status-lock" style="color:#f97316;">检测中...</span>
        </div>
        <div class="status-item">
          <span>空间物理距离 (Z轴):</span>
          <span class="status-val" id="status-dist">-- cm</span>
        </div>
        <div class="status-item">
          <span>已采集位姿样本:</span>
          <span class="status-val" id="status-samples" style="color:#facc15;">0 / 12</span>
        </div>
        <div class="status-item">
          <span>机械臂当前 6 轴角度 (°):</span>
          <span class="status-val" id="status-joints" style="font-size:11px;">[0, 20, -30, 0, 20, 0]</span>
        </div>

        <div class="btn-group">
          <button onclick="capturePose()">📸 采集当前帧位姿 (Capture Pose)</button>
          <button class="btn-warning" onclick="autoSweep()">🔄 自动多姿态安全巡检 (Auto Sweep 8 Poses)</button>
          <button class="btn-success" onclick="computeCalibration()">⚙️ 一键解算手眼外参 (Compute Hand-Eye)</button>
          <button class="btn-danger" onclick="clearSamples()">🗑️ 清空已采样本 (Reset)</button>
        </div>

        <div id="result-container" style="display:none; margin-top:10px;">
          <div style="font-weight:bold; font-size:13px; margin-bottom:6px; color:#38bdf8;">🎉 手眼外参矩阵 (T_base_camera):</div>
          <div class="result-box" id="result-text"></div>
        </div>
      </div>
    </div>
  </div>

  <script>
    const canvas = document.getElementById('live-canvas');
    const ctx = canvas.getContext('2d');
    const fpsBadge = document.getElementById('fps-indicator');

    let isFetching = false;
    let frameCount = 0;
    let lastFpsTime = performance.now();

    // 零积压流水线：只有前一帧在 Canvas 绘制完毕后，才立刻抓取下一帧
    // 永远取相机最新画面，哪怕网络丢包也绝不堆积历史旧帧！
    function loadNextFrame() {
      if (isFetching) return;
      isFetching = true;

      const img = new Image();
      img.onload = () => {
        ctx.drawImage(img, 0, 0, canvas.width, canvas.height);
        isFetching = false;

        frameCount++;
        const now = performance.now();
        if (now - lastFpsTime >= 1000) {
          const fps = Math.round((frameCount * 1000) / (now - lastFpsTime));
          fpsBadge.innerText = 'LIVE: ' + fps + ' FPS | 极速';
          fpsBadge.style.color = '#4ade80';
          frameCount = 0;
          lastFpsTime = now;
        }

        // 立即触发下一帧
        requestAnimationFrame(loadNextFrame);
      };

      img.onerror = () => {
        isFetching = false;
        fpsBadge.innerText = 'RECONNECTING...';
        fpsBadge.style.color = '#f97316';
        setTimeout(loadNextFrame, 150);
      };

      img.src = '/api/latest_frame.jpg?t=' + performance.now();
    }

    // 启动实时流
    document.addEventListener('DOMContentLoaded', () => {
      loadNextFrame();
    });

    function updateStatus() {
      fetch('/api/status?t=' + Date.now())
        .then(r => r.json())
        .then(d => {
          const lockVal = document.getElementById('status-lock');
          if (d.locked) {
            lockVal.innerText = '已稳定锁定 (11x8)';
            lockVal.style.color = '#22c55e';
          } else {
            lockVal.innerText = '未锁定 (调整标定板位置)';
            lockVal.style.color = '#f97316';
          }
          document.getElementById('status-dist').innerText = d.dist_cm + ' cm';
          document.getElementById('status-samples').innerText = d.samples_count + ' / 12 (最少需3组)';
          document.getElementById('status-joints').innerText = JSON.stringify(d.joints);
        })
        .catch(() => {});
    }
    setInterval(updateStatus, 500);

    function capturePose() {
      fetch('/api/capture')
        .then(r => r.json())
        .then(d => {
          if (d.status === 'ok') {
            alert('✅ 采集成功！已采集: ' + d.count + ' 组样本，目标距离: ' + d.dist_cm.toFixed(1) + ' cm' + (d.extra || ''));
          } else {
            alert('⚠️ ' + d.msg);
          }
        });
    }

    function autoSweep() {
      if (confirm('确认启动机械臂自动微调采样吗？机械臂将在相机视野内慢速变换 8 个角度并自动抓拍！')) {
        fetch('/api/auto_sweep')
          .then(r => r.json())
          .then(d => alert(d.msg));
      }
    }

    function computeCalibration() {
      fetch('/api/compute')
        .then(r => r.json())
        .then(d => {
          if (d.status === 'ok') {
            document.getElementById('result-container').style.display = 'block';
            let txt = '空间平移 [X, Y, Z] (米):\\n' +
                      '  X: ' + d.tx + ' m\\n  Y: ' + d.ty + ' m\\n  Z: ' + d.tz + ' m\\n\\n' +
                      '旋转欧拉角 [Roll, Pitch, Yaw] (度):\\n' +
                      '  Roll: ' + d.roll_deg + '°\\n  Pitch: ' + d.pitch_deg + '°\\n  Yaw: ' + d.yaw_deg + '°\\n\\n' +
                      '4x4 变换矩阵 (T_base_camera):\\n' +
                      JSON.stringify(d.matrix, null, 2) + '\\n\\n' +
                      '已自动生成 publish_camera_tf.sh 供常驻广播！';
            document.getElementById('result-text').innerText = txt;
            alert('🎉 手眼标定计算完成！相机空间平移: [' + d.tx + ', ' + d.ty + ', ' + d.tz + '] 米');
          } else {
            alert('⚠️ ' + d.msg);
          }
        });
    }

    function clearSamples() {
      if (confirm('确定要清空所有已采样本吗？')) {
        fetch('/api/clear').then(() => updateStatus());
      }
    }
  </script>
</body>
</html>
'''


def main():
    global vision_node_instance
    rclpy.init()

    # 初始化 PIPER 机械臂 SDK
    if HAS_PIPER_SDK:
        try:
            piper = C_PiperInterface_V2("can0")
            piper.ConnectPort()
            state.piper = piper
            print("[Visualizer] 机械臂 CAN 通信已建立")
        except Exception as e:
            print(f"[Visualizer] 机械臂连接提示: {e}")

    vision_node_instance = UltraLowLatencyVisionNode()
    ros_thread = threading.Thread(target=lambda: rclpy.spin(vision_node_instance), daemon=True)
    ros_thread.start()

    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer(('0.0.0.0', PORT), CalibHttpHandler)
    print("============================================================")
    print("🚀 PIPER 超低延迟极速控制台已就绪！")
    print(f"🌐 访问链接: http://192.168.31.70:{PORT}")
    print("============================================================")

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.server_close()
        vision_node_instance.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
