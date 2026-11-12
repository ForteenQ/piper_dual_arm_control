#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂 × Intel RealSense D435 全自动视觉抓放、在线手眼标定与 3D 质点轨迹总控台 (v4.0)
核心特性:
1. 支持 --mode dual (:8080 三栏式双臂协同页) 与 --mode single (:8081 纯单臂导师演示页)
2. 实时 3D 桌面与方块质点轨迹跟踪 (支持视觉 + 夹爪遮挡期间运动学接续追踪)
3. 预留臂端手眼一体摄像头接口 (/hand_camera/color/image_raw -> /api/hand_cam_frame.jpg)
4. 修复低肘部过渡姿态规划与安全净空，确保 A/B 臂在 50cm 低架位均可安全绕避相机抓取
5. 增强 CAN 逻辑芯片(0x4AF)与 24V 动力母线(0x2A1)状态分离诊断，支持网页一键上电使能
"""

import os
import sys
import json
import time
import math
import argparse
import subprocess
import threading
import collections
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse

import cv2
import numpy as np
from scipy.interpolate import CubicSpline, PchipInterpolator

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from sensor_msgs.msg import Image, CameraInfo, JointState
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64, String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from builtin_interfaces.msg import Duration as RosDuration
from cv_bridge import CvBridge
import tf2_ros

from piper_kinematics_engine import (
    ARM_JOINTS,
    piper_all_links_fk,
    solve_hover_ik,
    solve_cartesian_segment,
    check_trajectory_safety,
    choose_safe_ready_pose,
    choose_best_cube_x_dir,
    rot_to_quat,
    get_home_joints_rad,
)
from dual_arm_calib_core import (
    WORK_DIR,
    CONFIG_PATH,
    TAG_SIZE_M,
    load_dual_config,
    save_dual_config,
    solve_rigid_transform_svd,
    MultiScaleAprilTagDetector,
    draw_arm_ar_overlay,
)
from web_ui_components import render_single_arm_html, render_dual_arm_html

_parser = argparse.ArgumentParser(add_help=False)
_parser.add_argument('--mode', choices=['dual', 'single'], default='dual')
_parser.add_argument('--port', type=int, default=8080)
_parser.add_argument('--arm', choices=['arm_a', 'arm_b'], default='arm_a')
_args, _ = _parser.parse_known_args()

MODE = _args.mode
PORT = _args.port
PRIMARY_ARM = _args.arm


class DualArmSystemState:
    """全局线程安全系统状态"""

    def __init__(self):
        self.lock = threading.Lock()
        self.config = load_dual_config()
        self.config_mtime = os.path.getmtime(CONFIG_PATH) if os.path.exists(CONFIG_PATH) else 0.0

        # 主相机与预留手眼一体相机状态
        self.camera_matrix = None
        self.dist_coeffs = None
        self.latest_depth = None
        self.last_img_time = 0.0
        self.annotated_jpeg = self._make_placeholder("REALSENSE OFFLINE - Click [Start Camera]")
        self.frame_id = 0

        self.hand_cam_topic = "/hand_camera/color/image_raw"
        self.hand_cam_time = 0.0
        self.hand_cam_jpeg = self._make_placeholder(
            "EYE-IN-HAND RESERVED", "Topic: /hand_camera/color/image_raw (Awaiting Connection)"
        )

        # 当前视觉检测到的目标 (相机系真值)
        self.tag_detected = False
        self.tag_id = -1
        self.tag_pos_cam = None
        self.tag_yaw_cam = None
        self.tag_cam_history = collections.deque(maxlen=6)
        self.table_depth_mm = 515.0

        # 双方块 (Cube A 左侧 / Cube B 右侧) 感知与状态机追踪
        self.cubes = {
            "A": {
                "detected": False,
                "id": -1,
                "p_cam": None,
                "p_base": None,
                "yaw_cam": None,
                "edge_dir": None,
                "history": collections.deque(maxlen=6),
                "init_p_base": None,
                "init_p_cam": None,
                "init_edge_dir": None,
                "uv_rot": None,
                "last_seen": 0.0
            },
            "B": {
                "detected": False,
                "id": -1,
                "p_cam": None,
                "p_base": None,
                "yaw_cam": None,
                "edge_dir": None,
                "history": collections.deque(maxlen=6),
                "init_p_base": None,
                "init_p_cam": None,
                "init_edge_dir": None,
                "uv_rot": None,
                "last_seen": 0.0
            }
        }
        self.anchors_locked = False
        self.swap_workflow_busy = False
        self.swap_workflow_step = 0  # 0=待命, 1=A送底座完成, 2=B叠顶层完成, 3=A取顶完成, 4=B取底完成
        self.swap_workflow_status = "待命 (请将方块A置于左侧，方块B置于右侧)"
        self.abort_workflow_flag = False

        # 3D 双质点与单臂兼容轨迹跟踪缓冲
        self.traj_points = collections.deque(maxlen=600)
        self.traj_curr = None
        self.traj_a_points = collections.deque(maxlen=400)
        self.traj_a_curr = None
        self.traj_b_points = collections.deque(maxlen=400)
        self.traj_b_curr = None
        self.carrying_arm = None
        self.carrying_cube = None

        # 双臂实时物理状态
        self.arm_runtime = {
            "arm_a": self._init_arm_runtime(),
            "arm_b": self._init_arm_runtime(),
        }
        self.raw_can_interfaces = []
        self.last_dual_status_time = 0.0

        # 鼠标选定的目标放置点
        self.clicked_uv = None
        self.clicked_cam_pos = None
        self.clicked_eval = {}

        # 标定工作区状态 (按臂独立缓冲)
        self.calib_target_arm = PRIMARY_ARM if MODE == 'single' else "arm_a"
        self.calib_touch_mode = "tip_touch"
        self.calib_pending_cam = {"arm_a": None, "arm_b": None}
        self.calib_points = {"arm_a": [], "arm_b": []}

        # 任务执行状态
        self.selected_arm_mode = PRIMARY_ARM if MODE == 'single' else "auto"
        self.task_busy = False
        self.task_status = "空闲待命 (随时可执行任务)"
        self.task_logs = collections.deque(maxlen=25)

    def get_arm_speed_factor(self, arm_id: str) -> float:
        """全局通用速度系数 (由前端滑块 20%~150% 动态调节)"""
        with self.lock:
            pct = float(self.config["arms"].get(arm_id, {}).get("speed_pct", 100))
        return max(0.2, min(1.8, pct / 100.0))

    @staticmethod
    def _init_arm_runtime():
        return {
            "joints": None,
            "joints_deg": [0.0] * 6,
            "gripper_mm": 0.0,
            "can_port": "--",
            "usb_serial": "--",
            "can_up": False,
            "connected": False,
            "logic_alive": False,
            "motors_alive": False,
            "ctrl_mode": -1,
            "arm_status": -1,
            "busy": False,
        }

    @staticmethod
    def _make_placeholder(title: str, sub: str = "Web Console Ready - Start Camera Anytime") -> bytes:
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.rectangle(img, (20, 20), (620, 460), (45, 65, 90), 2)
        cv2.putText(img, title, (70, 228), cv2.FONT_HERSHEY_SIMPLEX, 0.72, (0, 220, 255), 2)
        cv2.putText(img, sub, (60, 272), cv2.FONT_HERSHEY_SIMPLEX, 0.50, (140, 180, 210), 1)
        _, jpeg = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 75])
        return jpeg.tobytes()

    def add_log(self, msg: str):
        if MODE == 'single':
            for tok in (' [arm_a]', ' [arm_b]', ' [ARM_A]', ' [ARM_B]'):
                msg = msg.replace(tok, '')
        ts = time.strftime("%H:%M:%S")
        line = f"[{ts}] {msg}"
        with self.lock:
            self.task_logs.append(line)
            self.task_status = msg
        print(line, flush=True)

    def get_arm_T_base_cam(self, arm_id: str):
        arm_cfg = self.config["arms"].get(arm_id, {})
        mat = arm_cfg.get("T_base_cam")
        return np.array(mat, dtype=float) if mat is not None else None

    def transform_cam_to_arm(self, arm_id: str, p_cam: np.ndarray, yaw_cam: np.ndarray = None):
        T = self.get_arm_T_base_cam(arm_id)
        if T is None or p_cam is None:
            return None, None
        pb = (T @ np.array([p_cam[0], p_cam[1], p_cam[2], 1.0], dtype=float))[:3]
        yaw_b = np.array([1.0, 0.0, 0.0])
        if yaw_cam is not None:
            vb = T[:3, :3] @ np.asarray(yaw_cam, dtype=float)
            vb[2] = 0.0
            n = np.linalg.norm(vb)
            if n > 1e-4:
                yaw_b = vb / n
        return pb, yaw_b

    def get_world_ref_arm(self) -> str:
        """确定 3D 质点轨迹的统一桌面世界基准坐标系"""
        if MODE == 'single':
            return PRIMARY_ARM
        if self.get_arm_T_base_cam("arm_a") is not None:
            return "arm_a"
        if self.get_arm_T_base_cam("arm_b") is not None:
            return "arm_b"
        return "arm_a"

    def record_cube_3d_point(self, p_world_m: np.ndarray, table_z_m: float, is_tag_top: bool = True, cube_key: str = None):
        """将方块中心转为相对桌面的 3D 毫米坐标并加入轨迹 (支持单臂主轨迹及 A/B 双方块独立轨迹)"""
        z_center_m = float(p_world_m[2]) - float(table_z_m) - (0.015 if is_tag_top else 0.0)
        pt_mm = [
            round(float(p_world_m[0]) * 1000.0, 1),
            round(float(p_world_m[1]) * 1000.0, 1),
            round(max(0.0, z_center_m * 1000.0), 1),
        ]
        with self.lock:
            if cube_key is None or cube_key == "A":
                self.traj_curr = pt_mm
                if not self.traj_points:
                    self.traj_points.append(pt_mm)
                else:
                    last = self.traj_points[-1]
                    dist = math.sqrt((pt_mm[0] - last[0])**2 + (pt_mm[1] - last[1])**2 + (pt_mm[2] - last[2])**2)
                    if 2.0 <= dist <= 260.0:
                        self.traj_points.append(pt_mm)

            if cube_key == "A":
                self.traj_a_curr = pt_mm
                if not self.traj_a_points:
                    self.traj_a_points.append(pt_mm)
                else:
                    last = self.traj_a_points[-1]
                    dist = math.sqrt((pt_mm[0] - last[0])**2 + (pt_mm[1] - last[1])**2 + (pt_mm[2] - last[2])**2)
                    if 2.0 <= dist <= 260.0:
                        self.traj_a_points.append(pt_mm)
            elif cube_key == "B":
                self.traj_b_curr = pt_mm
                if not self.traj_b_points:
                    self.traj_b_points.append(pt_mm)
                else:
                    last = self.traj_b_points[-1]
                    dist = math.sqrt((pt_mm[0] - last[0])**2 + (pt_mm[1] - last[1])**2 + (pt_mm[2] - last[2])**2)
                    if 2.0 <= dist <= 260.0:
                        self.traj_b_points.append(pt_mm)


state = DualArmSystemState()
worker_node = None


class DualArmVisualWorkerNode(Node):
    """负责订阅相机流、手眼预留流、双臂关节与动态广播 TF"""

    def __init__(self):
        super().__init__(f'piper_visual_console_{MODE}_{PORT}')
        self.bridge = CvBridge()
        self.is_processing = False
        self.detector = MultiScaleAprilTagDetector(TAG_SIZE_M)
        self.cb_group = ReentrantCallbackGroup()

        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.create_timer(2.0, self._publish_calibrated_tfs, callback_group=self.cb_group)
        self.create_timer(1.0, self._check_camera_watchdog, callback_group=self.cb_group)
        self.create_timer(1.0, self._sync_config_if_modified, callback_group=self.cb_group)

        self.dual_cmd_pub = self.create_publisher(String, '/dual_arm/command', 10)
        self.arm_io = {}
        for aid in ("arm_a", "arm_b"):
            self.arm_io[aid] = {
                "gripper_pub": self.create_publisher(Float64, f'/{aid}/piper_gripper_cmd', 10),
                "effort_pub": self.create_publisher(Float64, f'/{aid}/piper_gripper_effort', 10),
                "action_client": ActionClient(
                    self, FollowJointTrajectory, f'/{aid}/arm_controller/follow_joint_trajectory',
                    callback_group=self.cb_group
                ),
                "done_event": threading.Event(),
                "exec_ok": False,
            }
            self.create_subscription(
                JointState, f'/{aid}/joint_states',
                lambda m, a=aid: self._joint_cb(a, m),
                10, callback_group=self.cb_group
            )

        self.create_subscription(String, '/dual_arm/status_json', self._dual_status_cb, 10, callback_group=self.cb_group)

        qos_best = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self._info_cb, 10, callback_group=self.cb_group)
        self.create_subscription(Image, '/camera/camera/color/image_raw', self._img_cb, qos_best, callback_group=self.cb_group)
        self.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', self._depth_cb, qos_best, callback_group=self.cb_group)

        # 预留臂端手眼一体摄像头话题订阅
        self.qos_best = qos_best
        self.hand_cam_sub = self.create_subscription(
            Image, state.hand_cam_topic, self._hand_img_cb, qos_best, callback_group=self.cb_group
        )

        self.last_detect_time = 0.0
        self.get_logger().info(f'🚀 视觉控制控制台 ROS 2 Worker 上线 (mode={MODE}, port={PORT})')

    def update_hand_cam_topic(self, topic_name: str):
        try:
            self.destroy_subscription(self.hand_cam_sub)
        except Exception:
            pass
        state.hand_cam_topic = topic_name
        state.hand_cam_time = 0.0
        state.hand_cam_jpeg = state._make_placeholder("EYE-IN-HAND RESERVED", f"Topic: {topic_name}")
        self.hand_cam_sub = self.create_subscription(
            Image, topic_name, self._hand_img_cb, self.qos_best, callback_group=self.cb_group
        )

    def send_dual_command(self, cmd_dict: dict):
        msg = String()
        msg.data = json.dumps(cmd_dict, ensure_ascii=False)
        self.dual_cmd_pub.publish(msg)

    def _sync_config_if_modified(self):
        if not os.path.exists(CONFIG_PATH):
            return
        try:
            mt = os.path.getmtime(CONFIG_PATH)
            with state.lock:
                if mt > state.config_mtime:
                    state.config = load_dual_config()
                    state.config_mtime = mt
        except Exception:
            pass

    def _publish_calibrated_tfs(self):
        transforms = []
        now = self.get_clock().now().to_msg()
        with state.lock:
            cfg = state.config
        for aid in ("arm_a", "arm_b"):
            mat = cfg["arms"][aid].get("T_base_cam")
            if mat is None:
                continue
            try:
                T_cam_base = np.linalg.inv(np.array(mat, dtype=float))
                qx, qy, qz, qw = rot_to_quat(T_cam_base[:3, :3])
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = 'camera_color_optical_frame'
                t.child_frame_id = f'{aid}_base_link'
                t.transform.translation.x = float(T_cam_base[0, 3])
                t.transform.translation.y = float(T_cam_base[1, 3])
                t.transform.translation.z = float(T_cam_base[2, 3])
                t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
                transforms.append(t)
            except Exception:
                pass
        if transforms:
            self.static_broadcaster.sendTransform(transforms)

    def _check_camera_watchdog(self):
        now = time.time()
        with state.lock:
            main_off = (now - state.last_img_time > 2.5)
            hand_off = (state.hand_cam_time > 0 and now - state.hand_cam_time > 3.0)
        if main_off:
            placeholder = state._make_placeholder("REALSENSE OFFLINE - Click [Start Camera]")
            with state.lock:
                state.annotated_jpeg = placeholder
                state.tag_detected = False
        if hand_off:
            ph = state._make_placeholder("EYE-IN-HAND OFFLINE", f"Topic: {state.hand_cam_topic}")
            with state.lock:
                state.hand_cam_jpeg = ph
                state.hand_cam_time = 0.0

    def _dual_status_cb(self, msg: String):
        try:
            d = json.loads(msg.data)
        except Exception:
            return
        with state.lock:
            state.last_dual_status_time = time.time()
            state.raw_can_interfaces = d.get("raw_can_interfaces", [])
            for aid in ("arm_a", "arm_b"):
                if aid in d:
                    rt = state.arm_runtime[aid]
                    src = d[aid]
                    for k in ("can_port", "usb_serial", "can_up", "connected", "logic_alive",
                              "motors_alive", "ctrl_mode", "arm_status", "busy", "gripper_mm", "joints_deg"):
                        if k in src:
                            rt[k] = src[k]
                    if src.get("motors_alive") and src.get("joints_deg"):
                        rt["joints"] = np.radians(np.array(src["joints_deg"], dtype=float))

    def _joint_cb(self, arm_id: str, msg: JointState):
        if len(msg.name) < 6:
            return
        try:
            q = np.array([msg.position[msg.name.index(j)] for j in ARM_JOINTS], dtype=float)
            g_mm = 0.0
            if 'joint7' in msg.name and 'joint8' in msg.name:
                g_mm = (msg.position[msg.name.index('joint7')] + msg.position[msg.name.index('joint8')]) * 1000.0
            with state.lock:
                rt = state.arm_runtime[arm_id]
                rt["joints"] = q
                rt["joints_deg"] = [round(math.degrees(x), 2) for x in q]
                rt["gripper_mm"] = round(g_mm, 1)
        except ValueError:
            pass

    def _info_cb(self, msg: CameraInfo):
        with state.lock:
            state.camera_matrix = np.array(msg.k, dtype=np.float64).reshape((3, 3))
            state.dist_coeffs = np.array(msg.d, dtype=np.float64)

    def _depth_cb(self, msg: Image):
        try:
            d = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            state.latest_depth = d.copy()
        except Exception:
            pass

    def _hand_img_cb(self, msg: Image):
        try:
            f = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv2.putText(f, "CAM-HAND (Eye-in-Hand)", (12, 26), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (56, 189, 248), 2)
            _, jpeg = cv2.imencode('.jpg', f, [cv2.IMWRITE_JPEG_QUALITY, 75])
            with state.lock:
                state.hand_cam_jpeg = jpeg.tobytes()
                state.hand_cam_time = time.time()
        except Exception:
            pass

    def _img_cb(self, msg: Image):
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

    def _update_3d_trajectory_from_state(self):
        """在遮挡抓取搬运期间，用运动学指尖接续追踪被搬运方块的 3D 质点"""
        with state.lock:
            carry = state.carrying_arm
            c_cube = state.carrying_cube
            cfg = state.config
            if carry is None:
                return
            q = state.arm_runtime[carry]["joints"]
            if q is None:
                return

        _, _, T_tip = piper_all_links_fk(q, tip_offset_z=0.136)
        p_tip = T_tip[:3, 3]
        ref_arm = state.get_world_ref_arm()
        tz = float(cfg["arms"][ref_arm].get("table_z_base", -0.005))

        if carry == ref_arm:
            p_world = p_tip
        elif ref_arm == "arm_a" and carry == "arm_b" and cfg.get("T_baseA_baseB") is not None:
            T_ab = np.array(cfg["T_baseA_baseB"], dtype=float)
            p_world = (T_ab @ np.array([p_tip[0], p_tip[1], p_tip[2], 1.0]))[:3]
        else:
            p_world = p_tip
        state.record_cube_3d_point(p_world, tz, is_tag_top=False, cube_key=c_cube)

    def process_frame(self, frame):
        # 响应用户需求：将画面翻转 180 度。在翻转后的图像上统一绘制标注与文字，确保文字方向正立、易读
        display = cv2.rotate(frame, cv2.ROTATE_180)
        with state.lock:
            state.last_img_time = time.time()
            K = state.camera_matrix
            D = state.dist_coeffs
            depth = state.latest_depth
            cfg = state.config
            c_uv = state.clicked_uv
            c_cam = state.clicked_cam_pos
            c_eval = state.clicked_eval
            cal_arm = PRIMARY_ARM if MODE == 'single' else state.calib_target_arm
            cal_pts = list(state.calib_points.get(cal_arm, []))

        if K is not None:
            ar_list = [(PRIMARY_ARM, "BASE", (255, 200, 0), (0, 230, 120))] if MODE == 'single' else [
                ("arm_a", "ARM A", (255, 200, 0), (0, 230, 120)),
                ("arm_b", "ARM B", (255, 100, 255), (255, 200, 50)),
            ]
            for aid, label, c_main, c_arc in ar_list:
                T_bc = state.get_arm_T_base_cam(aid)
                if T_bc is not None:
                    tz = float(cfg["arms"][aid].get("table_z_base", -0.005))
                    draw_arm_ar_overlay(display, label, T_bc, K, D, tz, c_main, c_arc)

            now_t = time.time()
            if MODE == 'single':
                # 单臂演示台：始终保持单标靶模式，若场内有多个方块则自动择优锁定距本臂最近的 1 块
                cands = self.detector.detect_all(frame, K, D, depth)
                det = None
                if len(cands) == 1:
                    det = cands[0]
                elif len(cands) > 1:
                    best_r = 1e9
                    for cand in cands:
                        ps_c, _ = state.transform_cam_to_arm(PRIMARY_ARM, cand["p_cam"])
                        r_c = float(math.hypot(ps_c[0], ps_c[1])) if ps_c is not None else float(np.linalg.norm(cand["p_cam"]))
                        if r_c < best_r:
                            best_r = r_c
                            det = cand
                if det is not None:
                    self.last_detect_time = now_t
                    raw_corners = det["corners"].reshape((4, 2))
                    tc_rot = np.zeros_like(raw_corners, dtype=np.float32)
                    tc_rot[:, 0] = 639.0 - raw_corners[:, 0]
                    tc_rot[:, 1] = 479.0 - raw_corners[:, 1]
                    cv2.polylines(display, [tc_rot.astype(np.int32)], True, (0, 255, 0), 2)
                    c_center = tuple(tc_rot.mean(axis=0).astype(int))
                    cv2.circle(display, c_center, 4, (0, 0, 255), -1)

                    p_cam = det["p_cam"]
                    with state.lock:
                        state.tag_cam_history.append(p_cam)
                        smooth_cam = np.median(np.array(state.tag_cam_history), axis=0)
                        state.tag_detected = True
                        state.tag_id = det["id"]
                        state.tag_pos_cam = smooth_cam
                        state.tag_yaw_cam = det["yaw_vec_cam"]

                    ref_arm = state.get_world_ref_arm()
                    p_ref, _ = state.transform_cam_to_arm(ref_arm, smooth_cam)
                    if p_ref is not None:
                        tz = float(cfg["arms"][ref_arm].get("table_z_base", -0.005))
                        state.record_cube_3d_point(p_ref, tz, is_tag_top=True, cube_key=None)
                    else:
                        state.record_cube_3d_point(
                            np.array([smooth_cam[0], smooth_cam[1], 0.515 - smooth_cam[2]]), 0.0, is_tag_top=True, cube_key=None
                        )

                    cx_px, cy_px = 639 - int(det["uv"][0]), 479 - int(det["uv"][1])
                    ps, _ = state.transform_cam_to_arm(PRIMARY_ARM, smooth_cam)
                    txt = (f"ID:{det['id']} ({ps[0]*1000:.0f},{ps[1]*1000:.0f},{ps[2]*1000:.0f})"
                           if ps is not None else
                           f"ID:{det['id']} CAM:({smooth_cam[0]*1000:.0f},{smooth_cam[1]*1000:.0f},{smooth_cam[2]*1000:.0f})")
                    cv2.putText(display, txt, (max(10, cx_px - 95), max(25, cy_px - 20)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
                else:
                    if state.carrying_arm is not None:
                        self._update_3d_trajectory_from_state()
                    elif now_t - self.last_detect_time > 0.6:
                        with state.lock:
                            state.traj_curr = None
            else:
                # 双臂模式：多标靶视觉检测 (左侧方块A，右侧方块B)
                all_dets = self.detector.detect_all(frame, K, D, depth)

                if len(all_dets) > 0:
                    self.last_detect_time = now_t
                    for d in all_dets:
                        raw_corners = d["corners"].reshape((4, 2))
                        tc_rot = np.zeros_like(raw_corners, dtype=np.float32)
                        tc_rot[:, 0] = 639.0 - raw_corners[:, 0]
                        tc_rot[:, 1] = 479.0 - raw_corners[:, 1]
                        d["tc_rot"] = tc_rot
                        d["u_rot"] = float(tc_rot[:, 0].mean())
                        d["v_rot"] = float(tc_rot[:, 1].mean())

                    all_dets.sort(key=lambda x: x["u_rot"])

                    det_a, det_b = None, None
                    if len(all_dets) >= 2:
                        det_a = all_dets[0]  # 最左侧为方块 A
                        det_b = all_dets[-1] # 最右侧为方块 B
                    else:
                        d_single = all_dets[0]
                        with state.lock:
                            c_cube = state.carrying_cube
                            init_a = state.cubes["A"]["init_p_cam"]
                            init_b = state.cubes["B"]["init_p_cam"]
                        if c_cube == "A":
                            det_b = d_single
                        elif c_cube == "B":
                            det_a = d_single
                        elif init_a is not None and init_b is not None:
                            dist_a = np.linalg.norm(d_single["p_cam"] - init_a)
                            dist_b = np.linalg.norm(d_single["p_cam"] - init_b)
                            if dist_a < dist_b:
                                det_a = d_single
                            else:
                                det_b = d_single
                        else:
                            if d_single["u_rot"] < 320.0:
                                det_a = d_single
                            else:
                                det_b = d_single

                    ref_arm = state.get_world_ref_arm()
                    ref_tz = float(cfg["arms"][ref_arm].get("table_z_base", -0.005))

                    for cube_key, det_obj, color_box, label_name in [
                        ("A", det_a, (255, 230, 0), "CUBE A (Left)"),
                        ("B", det_b, (255, 100, 255), "CUBE B (Right)"),
                    ]:
                        if det_obj is None:
                            with state.lock:
                                if now_t - state.cubes[cube_key]["last_seen"] > 0.6:
                                    state.cubes[cube_key]["detected"] = False
                                    if state.carrying_cube != cube_key:
                                        if cube_key == "A": state.traj_a_curr = None
                                        else: state.traj_b_curr = None
                            continue

                        tc_rot = det_obj["tc_rot"]
                        cv2.polylines(display, [tc_rot.astype(np.int32)], True, color_box, 2)
                        c_center = (int(round(det_obj["u_rot"])), int(round(det_obj["v_rot"])))
                        cv2.circle(display, c_center, 4, (0, 0, 255), -1)

                        p_cam = det_obj["p_cam"]
                        target_arm = "arm_a" if cube_key == "A" else "arm_b"

                        with state.lock:
                            c_dict = state.cubes[cube_key]
                            c_dict["history"].append(p_cam)
                            smooth_cam = np.median(np.array(c_dict["history"]), axis=0)
                            c_dict["detected"] = True
                            c_dict["id"] = det_obj["id"]
                            c_dict["p_cam"] = smooth_cam
                            c_dict["yaw_cam"] = det_obj["yaw_vec_cam"]
                            c_dict["uv_rot"] = [det_obj["u_rot"], det_obj["v_rot"]]
                            c_dict["last_seen"] = now_t

                            p_base, edge_dir = state.transform_cam_to_arm(target_arm, smooth_cam, det_obj["yaw_vec_cam"])
                            c_dict["p_base"] = p_base
                            c_dict["edge_dir"] = edge_dir

                            if not state.anchors_locked and not state.swap_workflow_busy and state.swap_workflow_step == 0:
                                c_dict["init_p_cam"] = smooth_cam.copy()
                                if p_base is not None:
                                    c_dict["init_p_base"] = p_base.copy()
                                if edge_dir is not None:
                                    c_dict["init_edge_dir"] = edge_dir.copy()

                            state.tag_detected = True
                            state.tag_id = det_obj["id"]
                            state.tag_pos_cam = smooth_cam
                            state.tag_yaw_cam = det_obj["yaw_vec_cam"]

                        if state.carrying_cube != cube_key:
                            p_ref, _ = state.transform_cam_to_arm(ref_arm, smooth_cam)
                            if p_ref is not None:
                                state.record_cube_3d_point(p_ref, ref_tz, is_tag_top=True, cube_key=cube_key)
                            else:
                                state.record_cube_3d_point(
                                    np.array([smooth_cam[0], smooth_cam[1], 0.515 - smooth_cam[2]]), 0.0, is_tag_top=True, cube_key=cube_key
                                )

                        cx_px, cy_px = int(det_obj["u_rot"]), int(det_obj["v_rot"])
                        if p_base is not None:
                            txt_coord = f"{cube_key}:({p_base[0]*1000:.0f},{p_base[1]*1000:.0f},{p_base[2]*1000:.0f})mm"
                        else:
                            txt_coord = f"CAM:({smooth_cam[0]*1000:.0f},{smooth_cam[1]*1000:.0f},{smooth_cam[2]*1000:.0f})mm"

                        cv2.putText(display, f"[{label_name}]", (max(10, cx_px - 85), max(22, cy_px - 26)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.48, color_box, 2)
                        cv2.putText(display, txt_coord, (max(10, cx_px - 85), max(40, cy_px - 8)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.44, (255, 255, 255), 1)

                    if state.carrying_arm is not None:
                        self._update_3d_trajectory_from_state()
                else:
                    with state.lock:
                        for ck in ("A", "B"):
                            if now_t - state.cubes[ck]["last_seen"] > 0.6:
                                state.cubes[ck]["detected"] = False
                                if state.carrying_cube != ck:
                                    if ck == "A": state.traj_a_curr = None
                                    else: state.traj_b_curr = None
                    if state.carrying_arm is not None:
                        self._update_3d_trajectory_from_state()

            # 绘制标定采样点 C1..CN (坐标经 180度翻转匹配显示画面)
            show_calib_arms = [PRIMARY_ARM] if MODE == 'single' else ["arm_a", "arm_b"]
            for aid_c in show_calib_arms:
                pts_c = state.calib_points.get(aid_c, [])
                col_c = (0, 140, 255) if aid_c == "arm_a" else (255, 100, 255)
                for idx, cp in enumerate(pts_c):
                    pc = np.array(cp["p_cam"], dtype=float)
                    if pc[2] > 0.05:
                        uv2, _ = cv2.projectPoints(pc.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, D)
                        pu, pv = 639 - int(round(uv2[0, 0, 0])), 479 - int(round(uv2[0, 0, 1]))
                        cv2.circle(display, (pu, pv), 6, col_c, 2)
                        lbl = f"C{idx+1}" if MODE == 'single' else f"{aid_c[-1].upper()}{idx+1}"
                        cv2.putText(display, lbl, (pu + 7, pv - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.45, col_c, 1)

        if time.time() - self.last_detect_time > 0.7:
            with state.lock:
                state.tag_detected = False

        if c_uv is not None:
            raw_cu, raw_cv = c_uv
            u, v = 639 - raw_cu, 479 - raw_cv
            if MODE == 'single':
                valid_any = c_eval.get(PRIMARY_ARM, {}).get("valid", False)
                t_lbl = "TARGET" if valid_any else "OUT OF RANGE"
            else:
                valid_any = any(ev.get("valid", False) for ev in c_eval.values())
                reach_str = "/".join([a[-1].upper() for a, ev in c_eval.items() if ev.get("valid")]) or "OUT"
                t_lbl = f"TARGET [{reach_str}]"
            color = (255, 255, 0) if valid_any else (0, 0, 255)
            cv2.circle(display, (u, v), 13, color, 2)
            cv2.line(display, (u - 18, v), (u + 18, v), color, 2)
            cv2.line(display, (u, v - 18), (u, v + 18), color, 2)
            if c_cam is not None:
                cv2.putText(display, t_lbl, (min(460, u + 16), max(25, v - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, color, 2)

        # 绘制双方块公共交汇放置点 (金色十字标记与圆环)
        if MODE == 'dual' and K is not None:
            try:
                T_bc_a = state.get_arm_T_base_cam("arm_a")
                if T_bc_a is not None:
                    T_cam_a = np.linalg.inv(T_bc_a)
                    tz_a = float(cfg["arms"]["arm_a"].get("table_z_base", -0.005))
                    p_mid_cam = (T_cam_a @ np.array([0.180, -0.315, tz_a + 0.004, 1.0]))[:3]
                    uv_mid, _ = cv2.projectPoints(p_mid_cam.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, D)
                    um_rot = 639 - int(round(uv_mid[0, 0, 0]))
                    vm_rot = 479 - int(round(uv_mid[0, 0, 1]))
                    if 0 <= um_rot < 640 and 0 <= vm_rot < 480:
                        cv2.drawMarker(display, (um_rot, vm_rot), (0, 215, 255), cv2.MARKER_CROSS, 16, 2)
                        cv2.circle(display, (um_rot, vm_rot), 8, (0, 215, 255), 1)
                        cv2.putText(display, "[MID-ZONE]", (max(10, um_rot - 38), max(15, vm_rot - 12)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 215, 255), 1)
            except Exception:
                pass

        _, jpeg = cv2.imencode('.jpg', display, [cv2.IMWRITE_JPEG_QUALITY, 75])
        with state.lock:
            state.annotated_jpeg = jpeg.tobytes()
            state.frame_id += 1

    def get_depth_3d_cam(self, u: int, v: int):
        K = state.camera_matrix
        if K is None:
            return None
        u = max(0, min(639, int(u)))
        v = max(0, min(479, int(v)))
        d_mm = None
        if state.latest_depth is not None:
            patch = state.latest_depth[max(0, v - 3):min(480, v + 4), max(0, u - 3):min(640, u + 4)]
            valid = patch[(patch > 150) & (patch < 1500)]
            if len(valid) > 0:
                d_mm = float(np.median(valid))
        if d_mm is None:
            d_mm = state.table_depth_mm
        z_c = d_mm / 1000.0
        x_c = (u - K[0, 2]) * z_c / K[0, 0]
        y_c = (v - K[1, 2]) * z_c / K[1, 1]
        return np.array([x_c, y_c, z_c], dtype=float)

    def evaluate_arm_point(self, arm_id: str, p_cam: np.ndarray, is_place=True):
        T_bc = state.get_arm_T_base_cam(arm_id)
        if T_bc is None:
            return {"valid": False, "pos_b": None, "r_mm": 0.0, "msg": "未标定"}

        pos_b, _ = state.transform_cam_to_arm(arm_id, p_cam)
        r_horiz = math.hypot(pos_b[0], pos_b[1])
        cam_pos_b = T_bc[:3, 3]

        if r_horiz < 0.14 or r_horiz > 0.45:
            return {
                "valid": False, "pos_b": pos_b.tolist(), "r_mm": round(r_horiz * 1000.0, 1),
                "msg": f"水平距基座 r={r_horiz*1000:.0f}mm 超出范围 [140,450]"
            }
        if pos_b[2] < -0.08 or pos_b[2] > 0.14:
            return {
                "valid": False, "pos_b": pos_b.tolist(), "r_mm": round(r_horiz * 1000.0, 1),
                "msg": f"高度异常 Z={pos_b[2]*1000:.0f}mm"
            }

        q_test, ok, err_mm, _ = solve_hover_ik(pos_b + np.array([0.0, 0.0, 0.095]))
        if not ok:
            return {
                "valid": False, "pos_b": pos_b.tolist(), "r_mm": round(r_horiz * 1000.0, 1),
                "msg": f"IK无解 ({err_mm:.1f}mm)"
            }
        theta_t = math.atan2(pos_b[1], pos_b[0])
        q_home = get_home_joints_rad()
        q_ready = choose_safe_ready_pose(theta_t, q_home, q_test, cam_pos_b)
        safe, min_c, _, smsg = check_trajectory_safety([(q_home, q_ready), (q_ready, q_test)], cam_pos_b)
        if not safe:
            return {"valid": False, "pos_b": pos_b.tolist(), "r_mm": round(r_horiz * 1000.0, 1), "msg": smsg}

        return {
            "valid": True, "pos_b": pos_b.tolist(), "r_mm": round(r_horiz * 1000.0, 1),
            "msg": f"可达 r={r_horiz*1000:.0f}mm 净空={min_c*1000:.0f}mm"
        }

    def send_gripper(self, arm_id: str, width_mm: float, effort: int = None):
        io = self.arm_io[arm_id]
        if effort is not None:
            emsg = Float64()
            emsg.data = float(effort)
            io["effort_pub"].publish(emsg)
            time.sleep(0.03)
        msg = Float64()
        msg.data = float(width_mm)
        io["gripper_pub"].publish(msg)

    def send_trajectory(self, arm_id: str, waypoints, total_duration: float):
        io = self.arm_io[arm_id]
        client = io["action_client"]
        if not client.wait_for_server(timeout_sec=3.0):
            return False

        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        dt_step = total_duration / len(waypoints)

        with state.lock:
            q_curr = state.arm_runtime[arm_id]["joints"]

        vels = [np.zeros(6) for _ in waypoints]
        accs = [np.zeros(6) for _ in waypoints]
        if len(waypoints) >= 2 and q_curr is not None:
            pts = np.vstack([q_curr, np.array(waypoints, dtype=float)])
            ts = np.linspace(0.0, total_duration, len(pts))
            if len(waypoints) >= 3:
                cs = CubicSpline(ts, pts, bc_type='clamped', axis=0)
                v_all, a_all = cs(ts, 1), cs(ts, 2)
                vels = [v_all[k] for k in range(1, len(pts))]
                accs = [a_all[k] for k in range(1, len(pts))]
            else:
                pchip = PchipInterpolator(ts, pts, axis=0)
                v_all = pchip(ts, 1)
                vels = [v_all[k] for k in range(1, len(pts))]
                vels[-1] = np.zeros(6)

        for i, q in enumerate(waypoints):
            t = (i + 1) * dt_step
            pt = JointTrajectoryPoint()
            pt.positions = [float(x) for x in q]
            pt.velocities = [float(v) for v in vels[i]]
            pt.accelerations = [float(a) for a in accs[i]]
            sec = int(t)
            pt.time_from_start = RosDuration(sec=sec, nanosec=int((t - sec) * 1e9))
            goal.trajectory.points.append(pt)

        io["done_event"].clear()
        fut = client.send_goal_async(goal)
        fut.add_done_callback(lambda f, a=arm_id: self._goal_cb(a, f))
        io["done_event"].wait(timeout=total_duration + 10.0)
        return io["exec_ok"]

    def _goal_cb(self, arm_id: str, fut):
        io = self.arm_io[arm_id]
        gh = fut.result()
        if not gh.accepted:
            io["exec_ok"] = False
            io["done_event"].set()
            return
        gh.get_result_async().add_done_callback(lambda f, a=arm_id: self._res_cb(a, f))

    def _res_cb(self, arm_id: str, fut):
        io = self.arm_io[arm_id]
        res = fut.result().result
        io["exec_ok"] = (res.error_code == FollowJointTrajectory.Result.SUCCESSFUL)
        io["done_event"].set()


# ==============================================================================
# 双臂运动任务调度与接力协同工作流
# ==============================================================================
def execute_single_arm_pick_place(arm_id: str, mode: str, target_cam_pos=None):
    node = worker_node
    with state.lock:
        rt = state.arm_runtime[arm_id]
        if not rt["motors_alive"] or rt["joints"] is None:
            state.add_log(f"❌ [{arm_id}] 24V 动力未上电或 CAN 未连接！")
            return False
        if not state.tag_detected or state.tag_pos_cam is None:
            state.add_log("❌ 视野中未检测到 AprilTag 方块！")
            return False
        tag_cam = state.tag_pos_cam.copy()
        yaw_cam = state.tag_yaw_cam.copy() if state.tag_yaw_cam is not None else None
        q_curr = rt["joints"].copy()
        arm_cfg = dict(state.config["arms"][arm_id])

    T_bc = state.get_arm_T_base_cam(arm_id)
    if T_bc is None:
        state.add_log(f"❌ [{arm_id}] 尚未完成手眼标定，请先在本栏完成 4~6 点标定！")
        return False

    obj_pos, edge_dir = state.transform_cam_to_arm(arm_id, tag_cam, yaw_cam)
    cam_pos = T_bc[:3, 3]
    r_obj = math.hypot(obj_pos[0], obj_pos[1])
    if r_obj < 0.14 or r_obj > 0.45:
        state.add_log(f"🔴 [{arm_id}] 安全拦截: 方块距基座 {r_obj*1000:.0f}mm 超出臂展 [140,450]")
        return False

    x_off = float(arm_cfg.get("grasp_x_offset_mm", 0.0)) / 1000.0
    y_off = float(arm_cfg.get("grasp_y_offset_mm", 0.0)) / 1000.0
    z_off = float(arm_cfg.get("grasp_z_offset_mm", -25.5)) / 1000.0
    effort = int(arm_cfg.get("grasp_effort", 350))
    grip_w = float(arm_cfg.get("grasp_target_mm", 26.5))
    table_z = float(arm_cfg.get("table_z_base", -0.005))
    spd_pct = float(arm_cfg.get("speed_pct", 100))
    spd_f = max(0.2, min(1.8, spd_pct / 100.0))

    gx, gy = obj_pos[0] + x_off, obj_pos[1] + y_off
    z_desk = float(obj_pos[2]) - 0.025
    z_grasp = max(z_desk + 0.003, obj_pos[2] + z_off)
    tip_above_bottom = z_grasp - z_desk
    z_pre = obj_pos[2] + 0.095
    z_lift = z_grasp + 0.075

    p_pre = np.array([gx, gy, z_pre])
    p_grasp = np.array([gx, gy, z_grasp])
    p_lift = np.array([gx, gy, z_lift])

    state.add_log(f"🦾 [{arm_id.upper()}] 锁定抓取点: ({gx*1000:.1f}, {gy*1000:.1f}, {z_grasp*1000:.1f})mm")

    theta_obj = math.atan2(gy, gx)
    best_pick_dir = choose_best_cube_x_dir(theta_obj, edge_dir)
    q_pre, ok_pre, err_pre, _ = solve_hover_ik(p_pre, target_x_dir=best_pick_dir)
    if not ok_pre:
        state.add_log(f"🔴 [{arm_id}] 预抓取逆解失败 ({err_pre:.1f}mm)")
        return False
    q_ready = choose_safe_ready_pose(theta_obj, q_curr, q_pre, cam_pos)

    _, T6_pre, _ = piper_all_links_fk(q_pre)
    R_grasp = T6_pre[:3, :3]

    wp_down, msg_d = solve_cartesian_segment(q_pre, p_pre, p_grasp, R_grasp, num_steps=10)
    if wp_down is None:
        state.add_log(f"🔴 [{arm_id}] 下探规划失败: {msg_d}")
        return False
    wp_lift, msg_l = solve_cartesian_segment(wp_down[-1], p_grasp, p_lift, R_grasp, num_steps=8)
    if wp_lift is None:
        state.add_log(f"🔴 [{arm_id}] 抬升规划失败: {msg_l}")
        return False

    safety_pairs = [(q_curr, q_ready), (q_ready, q_pre)]
    wp_pl_down = wp_pl_up = q_pl_above = None

    if mode == 'place' and target_cam_pos is not None:
        tb_pos, _ = state.transform_cam_to_arm(arm_id, target_cam_pos)
        if tb_pos is None:
            return False
        z_pl_down = z_desk + tip_above_bottom + 0.0003
        z_pl_above = z_pl_down + 0.090
        p_pl_ab = np.array([tb_pos[0], tb_pos[1], z_pl_above])
        p_pl_dn = np.array([tb_pos[0], tb_pos[1], z_pl_down])

        theta_pl = math.atan2(tb_pos[1], tb_pos[0])
        best_place_dir = choose_best_cube_x_dir(theta_pl, np.array([1.0, 0.0, 0.0]))
        q_pl_above, ok_pa, err_pa, _ = solve_hover_ik(p_pl_ab, target_x_dir=best_place_dir)
        if not ok_pa:
            state.add_log(f"🔴 [{arm_id}] 目标放置逆解失败 ({err_pa:.1f}mm)")
            return False
        _, T6_pa, _ = piper_all_links_fk(q_pl_above)
        R_pa = T6_pa[:3, :3]
        wp_pl_down, msg_pd = solve_cartesian_segment(q_pl_above, p_pl_ab, p_pl_dn, R_pa, num_steps=10)
        wp_pl_up, msg_pu = (solve_cartesian_segment(wp_pl_down[-1], p_pl_dn, p_pl_ab, R_pa, num_steps=8)
                            if wp_pl_down is not None else (None, msg_pd))
        if wp_pl_down is None or wp_pl_up is None:
            state.add_log(f"🔴 [{arm_id}] 放置段规划失败")
            return False
        safety_pairs.extend([(wp_lift[-1], q_pl_above), (q_pl_above, get_home_joints_rad())])
    else:
        safety_pairs.append((wp_lift[-1], get_home_joints_rad()))

    safe, min_c, _, smsg = check_trajectory_safety(safety_pairs, cam_pos)
    if not safe:
        state.add_log(f"🔴 [{arm_id}] 碰撞拦截: {smsg}")
        return False

    # === 实机执行序列 (全程实时响应全局通用速度滑块) ===
    try:
        # 1. 抓取前预备：立即张开夹爪至 55mm (充分敞开)
        spd_f = state.get_arm_speed_factor(arm_id)
        state.add_log(f"👐 [{arm_id.upper()}] 预抓取准备: 展开夹爪至 55mm (全局速率:{int(spd_f*100)}%)...")
        node.send_gripper(arm_id, 55.0, effort)

        if not node.send_trajectory(arm_id, [q_ready, q_pre], 4.2 / spd_f):
            state.add_log(f"❌ [{arm_id}] 飞抵悬停失败")
            return False

        # 核心防撞安全阻塞：在下探前严格校验夹爪是否已充分张开 (>= 42mm)
        t_wait_open = time.time()
        while time.time() - t_wait_open < 1.5:
            with state.lock:
                g_cur = state.arm_runtime[arm_id].get("gripper_mm", 0.0)
            if g_cur >= 42.0:
                break
            node.send_gripper(arm_id, 55.0, effort)
            time.sleep(0.12)

        spd_f = state.get_arm_speed_factor(arm_id)
        if not node.send_trajectory(arm_id, wp_down, 3.0 / spd_f):
            state.add_log(f"❌ [{arm_id}] 垂直下探失败")
            return False

        state.add_log(f"🤏 [{arm_id.upper()}] 闭合夹持方块 (目标开度:{grip_w}mm, 力矩:{effort}mN·m)...")
        node.send_gripper(arm_id, grip_w, effort)
        time.sleep(0.9)
        with state.lock:
            state.carrying_arm = arm_id

        spd_f = state.get_arm_speed_factor(arm_id)
        if not node.send_trajectory(arm_id, wp_lift, 2.2 / spd_f):
            return False
        time.sleep(0.3)

        if mode == 'place' and wp_pl_down is not None:
            spd_f = state.get_arm_speed_factor(arm_id)
            state.add_log(f"✈️ [{arm_id}] 搬运前往目标点...")
            if not node.send_trajectory(arm_id, [q_pl_above], 3.8 / spd_f):
                return False
            spd_f = state.get_arm_speed_factor(arm_id)
            if not node.send_trajectory(arm_id, wp_pl_down, max(2.2, 3.0 / spd_f)):
                return False
            time.sleep(0.15)
            with state.lock:
                state.carrying_arm = None
            node.send_gripper(arm_id, 55.0, effort)
            time.sleep(0.6)
            spd_f = state.get_arm_speed_factor(arm_id)
            node.send_trajectory(arm_id, wp_pl_up, 2.0 / spd_f)
            q_ret_from = wp_pl_up[-1]
        else:
            spd_f = state.get_arm_speed_factor(arm_id)
            state.add_log(f"⏬ [{arm_id}] 原地放回...")
            wp_ret_dn, _ = solve_cartesian_segment(wp_lift[-1], p_lift, p_grasp, R_grasp, num_steps=10)
            if wp_ret_dn:
                node.send_trajectory(arm_id, wp_ret_dn, 2.6 / spd_f)
            with state.lock:
                state.carrying_arm = None
            node.send_gripper(arm_id, 55.0, effort)
            time.sleep(0.6)
            spd_f = state.get_arm_speed_factor(arm_id)
            wp_ret_up, _ = solve_cartesian_segment(
                wp_ret_dn[-1] if wp_ret_dn else wp_lift[-1], p_grasp, p_lift, R_grasp, num_steps=6
            )
            if wp_ret_up:
                node.send_trajectory(arm_id, wp_ret_up, 1.8 / spd_f)
            q_ret_from = wp_ret_up[-1] if wp_ret_up else wp_lift[-1]

        spd_f = state.get_arm_speed_factor(arm_id)
        state.add_log(f"🏠 [{arm_id}] 安全退回待命折叠原点...")
        q_home = get_home_joints_rad()
        theta_cur = float(q_ret_from[0])
        q_exit = choose_safe_ready_pose(theta_cur, q_ret_from, q_home, cam_pos)
        node.send_trajectory(arm_id, [q_exit, q_home], 3.8 / spd_f)
        node.send_gripper(arm_id, 0.0, effort)
        return True
    finally:
        with state.lock:
            state.carrying_arm = None


def find_shared_handoff_cam_point():
    node = worker_node
    for u in range(200, 460, 28):
        for v in range(160, 380, 28):
            p_cam = node.get_depth_3d_cam(u, v)
            if p_cam is None:
                continue
            ea = node.evaluate_arm_point("arm_a", p_cam, is_place=True)
            eb = node.evaluate_arm_point("arm_b", p_cam, is_place=True)
            if ea["valid"] and eb["valid"]:
                return p_cam
    return None


def pick_arm_for_task(mode: str, target_cam_pos=None, arm_mode_override: str = None) -> tuple:
    node = worker_node
    with state.lock:
        sel = arm_mode_override if arm_mode_override else state.selected_arm_mode
        if MODE == 'single':
            sel = PRIMARY_ARM
        tag_cam = state.tag_pos_cam.copy() if state.tag_pos_cam is not None else None
        detected = state.tag_detected

    if not detected or tag_cam is None:
        return ("err", "视野内未识别到 AprilTag 方块")

    ev_src = {aid: node.evaluate_arm_point(aid, tag_cam, is_place=False) for aid in ("arm_a", "arm_b")}

    if mode == "return":
        candidates = ("arm_a", "arm_b") if sel in ("auto", "relay") else (sel,)
        best = None
        best_r = 9999.0
        for aid in candidates:
            if ev_src[aid]["valid"] and state.arm_runtime[aid]["motors_alive"]:
                if ev_src[aid]["r_mm"] < best_r:
                    best_r = ev_src[aid]["r_mm"]
                    best = aid
        if best:
            return ("single", best)
        return ("err", f"所选机械臂 ({sel}) 未上电/未标定或够不着方块 (A:{ev_src['arm_a']['msg']} | B:{ev_src['arm_b']['msg']})")

    ev_dst = {aid: node.evaluate_arm_point(aid, target_cam_pos, is_place=True) for aid in ("arm_a", "arm_b")}

    if sel == "relay":
        for src, dst in [("arm_a", "arm_b"), ("arm_b", "arm_a")]:
            if ev_src[src]["valid"] and ev_dst[dst]["valid"] and state.arm_runtime[src]["motors_alive"] and state.arm_runtime[dst]["motors_alive"]:
                return ("relay", src, dst)
        return ("err", "双臂接力模式要求两臂均已标定上电，且分别可达起点与终点")

    if sel in ("arm_a", "arm_b"):
        if ev_src[sel]["valid"] and ev_dst[sel]["valid"]:
            return ("single", sel)
        return ("err", f"{sel} 无法同时触及起点与终点 (起点:{ev_src[sel]['msg']}, 终点:{ev_dst[sel]['msg']})")

    for aid in ("arm_a", "arm_b"):
        if ev_src[aid]["valid"] and ev_dst[aid]["valid"] and state.arm_runtime[aid]["motors_alive"]:
            return ("single", aid)
    for src, dst in [("arm_a", "arm_b"), ("arm_b", "arm_a")]:
        if ev_src[src]["valid"] and ev_dst[dst]["valid"] and state.arm_runtime[src]["motors_alive"] and state.arm_runtime[dst]["motors_alive"]:
            return ("relay", src, dst)
    return ("err", f"没有任何臂可达起点或终点 (A:{ev_src['arm_a']['msg']}, B:{ev_src['arm_b']['msg']})")


def run_task_worker(mode='return', target_cam_pos=None, arm_mode_override=None):
    with state.lock:
        if state.task_busy:
            return
        state.task_busy = True
        state.task_logs.clear()

    try:
        plan = pick_arm_for_task(mode, target_cam_pos, arm_mode_override)
        if plan[0] == "err":
            state.add_log(f"❌ 任务无法启动: {plan[1]}")
            return

        if plan[0] == "single":
            arm_id = plan[1]
            msg = "🚀 正在执行全自动视觉抓放任务..." if MODE == 'single' else f"🚀 已调度 [{arm_id.upper()}] 执行视觉抓放任务..."
            state.add_log(msg)
            ok = execute_single_arm_pick_place(arm_id, mode, target_cam_pos)
            state.add_log("✅ 任务圆满完成！" if ok else "❌ 任务中断")

        elif plan[0] == "relay":
            src_arm, dst_arm = plan[1], plan[2]
            state.add_log(f"🤝 启动双臂接力协同模式: [{src_arm.upper()}] 抓取递交 -> [{dst_arm.upper()}] 接力搬运！")
            handoff_cam = find_shared_handoff_cam_point()
            if handoff_cam is None:
                state.add_log("❌ 未找到双臂公共重叠桌面交接点！")
                return
            state.add_log(f"📍 Step 1/2: [{src_arm.upper()}] 先抓起方块放置于双臂交接区...")
            if not execute_single_arm_pick_place(src_arm, "place", handoff_cam):
                return
            state.add_log("⏳ 视觉重新锁定交接区方块位置 (闭环交接自校)...")
            time.sleep(0.8)
            state.add_log(f"📍 Step 2/2: [{dst_arm.upper()}] 从交接区抓起方块搬运至最终目标...")
            ok = execute_single_arm_pick_place(dst_arm, "place", target_cam_pos)
            state.add_log("🎉 双臂协同接力搬运圆满成功！" if ok else "❌ 第二阶段接力失败")
    except Exception as e:
        state.add_log(f"🔴 执行异常: {e}")
    finally:
        with state.lock:
            state.task_busy = False


def get_safe_yield_pose(arm_id: str) -> np.ndarray:
    """获取各机械臂【夹爪朝着摄像头】的高位安全避让待命姿态 (各在己侧收起，无干涉且下次动作极快响应)"""
    if arm_id == "arm_a":
        # A 臂在左侧，高位耸立，夹爪略微下倾正对前方摄像头视野
        return np.array([0.15, 0.45, -0.75, 0.0, 0.60, 0.0], dtype=float)
    else:
        # B 臂在右侧，高位耸立，夹爪略微下倾正对前方摄像头视野
        return np.array([-0.15, 0.45, -0.75, 0.0, 0.60, 0.0], dtype=float)


def ensure_arm_yielded(arm_id: str, allow_home: bool = False) -> bool:
    """确保指定机械臂已位于外侧安全避让区（或允许处于安全的折叠原点 Home）"""
    with state.lock:
        rt = state.arm_runtime[arm_id]
        if not rt["motors_alive"] or rt["joints"] is None:
            return False
        q_curr = rt["joints"].copy()
    q_yield = get_safe_yield_pose(arm_id)
    # 若已处于外侧安全避让位附近则无需动作
    if float(np.max(np.abs(q_curr[:3] - q_yield[:3]))) < 0.25:
        return True
    # 若允许停在 Home 且当前已处于己侧安全折叠原点附近，同样无需再拉去避让位
    if allow_home:
        q_home = get_home_joints_rad()
        if float(np.max(np.abs(q_curr - q_home))) < 0.08:
            state.add_log(f"🏠 [{arm_id.upper()}] 已安全折叠于原点待命，无需移至避让区")
            return True
    spd_f = state.get_arm_speed_factor(arm_id)
    state.add_log(f"🛡️ [{arm_id.upper()}] 让位至【夹爪朝向摄像头】待命区 (随时快速出击，距对侧>50cm)...")
    return bool(worker_node.send_trajectory(arm_id, [q_yield], 2.4 / spd_f))


def _execute_retract_home(arm_id="all", skip_if_at_home=False):
    """内部回安全折叠原点执行函数（不抢占 task_busy，供独立任务及工作流收尾共用）"""
    targets = ("arm_a", "arm_b") if arm_id == "all" else (arm_id,)
    q_home = get_home_joints_rad()
    for aid in targets:
        with state.lock:
            rt = state.arm_runtime[aid]
            alive = rt["motors_alive"]
            q_curr = rt["joints"].copy() if rt["joints"] is not None else None
            effort = int(state.config["arms"][aid].get("grasp_effort", 350))
        if not alive or q_curr is None:
            state.add_log(f"⚠️ [{aid.upper()}] 离线或未上电，跳过回原点")
            continue
        if skip_if_at_home and float(np.max(np.abs(q_curr - q_home))) < 0.045:
            state.add_log(f"🏠 [{aid.upper()}] 已处于安全折叠原点，无需重复复位")
            worker_node.send_gripper(aid, 0.0, effort)
            continue
        spd_f = state.get_arm_speed_factor(aid)
        T_bc = state.get_arm_T_base_cam(aid)
        cam_pos = T_bc[:3, 3] if T_bc is not None else None
        state.add_log(f"🏠 [{aid.upper()}] 正在退回安全折叠原点 (速率:{int(spd_f*100)}%)...")
        q_mid = choose_safe_ready_pose(float(q_curr[0]), q_curr, q_home, cam_pos)
        ok_ret = worker_node.send_trajectory(aid, [q_mid, q_home], 3.8 / spd_f)
        if not ok_ret:
            state.add_log(f"⚠️ [{aid.upper()}] 回原点轨迹执行异常，请检查机械臂状态")
        worker_node.send_gripper(aid, 0.0, effort)
    state.add_log("✅ 安全折叠原点复位完成！")


def run_retract_home_worker(arm_id="all"):
    with state.lock:
        if state.task_busy:
            return
        state.task_busy = True
    try:
        _execute_retract_home(arm_id=arm_id, skip_if_at_home=False)
    finally:
        with state.lock:
            state.task_busy = False


# ==============================================================================
# 双方块堆叠与空间互换高精度执行引擎
# ==============================================================================
def execute_subtask_pick_place(arm_id: str, p_pick_base: np.ndarray, p_place_base: np.ndarray,
                               pick_edge_dir=None, place_edge_dir=None,
                               cube_top_z_ref: float = None, cube_key: str = "A",
                               is_stack_place: bool = False, is_stack_pick: bool = False,
                               retreat_home: bool = False, pick_top_z_obs: float = None,
                               async_yield_other: bool = False, allow_other_home: bool = False) -> bool:
    """
    通用高精度笛卡尔搬运原子动作
    - 动作前自动校验对侧臂已停靠至外侧触碰不到的安全待命位（支持并发启动避让）
    - 执行完毕后默认退回外侧安全避让位 (yield pose)；若 retreat_home=True 则直接回归安全折叠原点
    - 全过程动态读取 speed_pct，实现全局通用速度控制
    """
    node = worker_node
    other_arm = "arm_b" if arm_id == "arm_a" else "arm_a"
    yield_res = {"ok": True}
    yield_thread = None
    if async_yield_other:
        # 双臂同步启动：后台并发让对侧臂进入避让态，主线程立即开始当前臂的抓取动作
        def _bg_yield():
            try:
                yield_res["ok"] = ensure_arm_yielded(other_arm, allow_home=allow_other_home)
            except Exception as _ye:
                state.add_log(f"⚠️ [{other_arm.upper()}] 并发避让异常: {_ye}")
                yield_res["ok"] = False

        yield_thread = threading.Thread(target=_bg_yield, daemon=True)
        yield_thread.start()
    else:
        ensure_arm_yielded(other_arm, allow_home=allow_other_home)

    with state.lock:
        if state.abort_workflow_flag:
            state.add_log(f"🛑 [{arm_id}] 用户已请求中止当前动作")
            return False
        rt = state.arm_runtime[arm_id]
        if not rt["motors_alive"] or rt["joints"] is None:
            state.add_log(f"❌ [{arm_id}] 24V 动力未上电或 CAN 未连接！")
            return False
        arm_cfg = dict(state.config["arms"][arm_id])
        q_curr = rt["joints"].copy()

    T_bc = state.get_arm_T_base_cam(arm_id)
    if T_bc is None:
        state.add_log(f"❌ [{arm_id}] 尚未完成手眼标定！")
        return False
    cam_pos = T_bc[:3, 3]

    table_z = float(arm_cfg.get("table_z_base", -0.005))
    x_off = float(arm_cfg.get("grasp_x_offset_mm", 0.0)) / 1000.0
    y_off = float(arm_cfg.get("grasp_y_offset_mm", 0.0)) / 1000.0
    z_off = float(arm_cfg.get("grasp_z_offset_mm", -25.5)) / 1000.0
    effort = int(arm_cfg.get("grasp_effort", 350))
    grip_w = float(arm_cfg.get("grasp_target_mm", 26.5))

    # 1. 精确反推局部真实桌面基准 Z (解决下放悬空 1-2cm 问题)
    # 已知标准方块高度为 25mm (0.025m)
    z_desk = table_z
    if cube_top_z_ref is not None:
        z_desk = float(cube_top_z_ref) - 0.025

    # 2. 抓取点高度与抓持底高差计算
    z_pick_floor = max(z_desk + 0.003, (z_desk + 0.025) + z_off)
    base_pick_z = z_pick_floor + (0.025 if is_stack_pick else 0.0)
    if is_stack_pick:
        state.add_log(f"📐 [{arm_id.upper()}] 顶层抓取模式 (Z高度 +25mm -> {base_pick_z*1000:.1f}mm)")

    # 计算抓紧时指尖距离方块底面的真实偏移量 (>=3mm)；顶层取块时结合视觉实测底面修正
    tip_above_bottom = z_pick_floor - z_desk
    if is_stack_pick and pick_top_z_obs is not None:
        if (z_desk + 0.036) <= float(pick_top_z_obs) <= (z_desk + 0.062):
            cube_bottom_obs = float(pick_top_z_obs) - 0.025
            tip_above_bottom = max(0.001, min(0.010, base_pick_z - cube_bottom_obs))
            state.add_log(f"📐 [{arm_id.upper()}] 结合顶层视觉实测校正抓持底高差: {tip_above_bottom*1000:.1f}mm")

    p_grasp = np.array([p_pick_base[0] + x_off, p_pick_base[1] + y_off, base_pick_z])
    p_pre = np.array([p_grasp[0], p_grasp[1], p_grasp[2] + 0.090])
    p_lift = np.array([p_grasp[0], p_grasp[1], p_grasp[2] + 0.080])

    # 3. 放置点高度计算：
    # 彻底杜绝悬空与挤压：地面放置必须贴准真实桌面基准 z_desk；堆叠放置则以公共底座顶面为基准
    if is_stack_place:
        # 堆叠模式：底座是公共交汇点处的下方方块A，其顶面高度为交汇点桌面基准 + 0.025m
        mid_ground_z = float(p_place_base[2]) if (p_place_base is not None and len(p_place_base) > 2) else z_desk
        z_pl_base = mid_ground_z + 0.025 + tip_above_bottom + 0.0045
        state.add_log(f"📐 [{arm_id.upper()}] 顶层堆叠放置 (防挤压轻放裕量+4.5mm -> Z={z_pl_base*1000:.1f}mm)")
    else:
        # 地面放置模式：严格以真实物理桌面 z_desk 为承载面，极微隙贴地平稳就位 (+0.3mm)，消除空中掉落感
        z_pl_base = z_desk + tip_above_bottom + 0.0003
        state.add_log(f"📐 [{arm_id.upper()}] 贴地平稳放置 (贴桌面微隙+0.3mm -> Z={z_pl_base*1000:.1f}mm, 桌面Z={z_desk*1000:.1f}mm)")

    p_pl_dn = np.array([p_place_base[0], p_place_base[1], z_pl_base])
    p_pl_ab = np.array([p_place_base[0], p_place_base[1], z_pl_base + 0.090])

    state.add_log(
        f"🎯 [{arm_id.upper()}] 规划路径: ({p_grasp[0]*1000:.0f},{p_grasp[1]*1000:.0f},z={base_pick_z*1000:.1f}) -> "
        f"({p_pl_dn[0]*1000:.0f},{p_pl_dn[1]*1000:.0f},z={z_pl_base*1000:.1f})mm (桌面基准z={z_desk*1000:.1f}mm)"
    )

    # 4. 逆解与 6 轴腕部平面精准对齐 (彻底防止夹对角线)
    theta_pick = math.atan2(p_pre[1], p_pre[0])
    best_pick_dir = choose_best_cube_x_dir(theta_pick, pick_edge_dir)
    q_pre, ok_pre, err_pre, _ = solve_hover_ik(p_pre, target_x_dir=best_pick_dir)
    if not ok_pre:
        state.add_log(f"🔴 [{arm_id}] 预抓取位姿逆解失败 ({err_pre:.1f}mm)")
        return False
    q_ready = choose_safe_ready_pose(theta_pick, q_curr, q_pre, cam_pos)

    _, T6_pre, _ = piper_all_links_fk(q_pre)
    R_grasp = T6_pre[:3, :3]

    wp_down, msg_d = solve_cartesian_segment(q_pre, p_pre, p_grasp, R_grasp, num_steps=10)
    if wp_down is None:
        state.add_log(f"🔴 [{arm_id}] 下探直线规划失败: {msg_d}")
        return False
    wp_lift, msg_l = solve_cartesian_segment(wp_down[-1], p_grasp, p_lift, R_grasp, num_steps=8)
    if wp_lift is None:
        state.add_log(f"🔴 [{arm_id}] 抬升直线规划失败: {msg_l}")
        return False

    theta_place = math.atan2(p_pl_ab[1], p_pl_ab[0])
    best_place_dir = choose_best_cube_x_dir(theta_place, place_edge_dir)
    q_pl_above, ok_pa, err_pa, _ = solve_hover_ik(p_pl_ab, target_x_dir=best_place_dir)
    if not ok_pa:
        state.add_log(f"🔴 [{arm_id}] 放置上方悬停逆解失败 ({err_pa:.1f}mm)")
        return False
    _, T6_pa, _ = piper_all_links_fk(q_pl_above)
    R_pa = T6_pa[:3, :3]

    wp_pl_down, msg_pd = solve_cartesian_segment(q_pl_above, p_pl_ab, p_pl_dn, R_pa, num_steps=10)
    if wp_pl_down is None:
        state.add_log(f"🔴 [{arm_id}] 放置下探直线规划失败: {msg_pd}")
        return False
    wp_pl_up, msg_pu = solve_cartesian_segment(wp_pl_down[-1], p_pl_dn, p_pl_ab, R_pa, num_steps=8)
    if wp_pl_up is None:
        state.add_log(f"🔴 [{arm_id}] 放置后抬升规划失败: {msg_pu}")
        return False

    # 5. 执行动作序列 (全程动态响应全局通用速度滑块)
    try:
        if state.abort_workflow_flag: return False
        spd_f = state.get_arm_speed_factor(arm_id)
        state.add_log(f"👐 [{arm_id.upper()}] 提前张开夹爪至 55mm (全局速率:{int(spd_f*100)}%)...")
        node.send_gripper(arm_id, 55.0, effort)

        if not node.send_trajectory(arm_id, [q_ready, q_pre], 4.0 / spd_f):
            return False

        t0 = time.time()
        while time.time() - t0 < 1.5:
            if state.arm_runtime[arm_id].get("gripper_mm", 0.0) >= 42.0:
                break
            node.send_gripper(arm_id, 55.0, effort)
            time.sleep(0.1)

        if state.abort_workflow_flag: return False
        spd_f = state.get_arm_speed_factor(arm_id)
        state.add_log(f"⬇️ [{arm_id.upper()}] 垂直下探对齐方块平面...")
        if not node.send_trajectory(arm_id, wp_down, 2.8 / spd_f):
            return False

        state.add_log(f"🤏 [{arm_id.upper()}] 闭合夹紧方块 {cube_key} (开度:{grip_w}mm, 力矩:{effort}mN·m)...")
        node.send_gripper(arm_id, grip_w, effort)
        time.sleep(0.9)
        with state.lock:
            state.carrying_arm = arm_id
            state.carrying_cube = cube_key

        if state.abort_workflow_flag: return False
        spd_f = state.get_arm_speed_factor(arm_id)
        state.add_log(f"⬆️ [{arm_id.upper()}] 垂直抬升...")
        if not node.send_trajectory(arm_id, wp_lift, 2.2 / spd_f):
            return False

        if state.abort_workflow_flag: return False
        if yield_thread is not None:
            yield_thread.join()
            if not yield_res["ok"]:
                state.add_log(f"❌ [{other_arm.upper()}] 并发避让失败，已安全中止进入公共区")
                return False
        spd_f = state.get_arm_speed_factor(arm_id)
        state.add_log(f"✈️ [{arm_id.upper()}] 平移至目标上方并摆正...")
        if not node.send_trajectory(arm_id, [q_pl_above], 3.8 / spd_f):
            return False

        if state.abort_workflow_flag: return False
        spd_f = state.get_arm_speed_factor(arm_id)
        state.add_log(f"⬇️ [{arm_id.upper()}] 贴近桌面垂直下探就位...")
        if not node.send_trajectory(arm_id, wp_pl_down, max(2.2, 3.0 / spd_f)):
            return False
        # 短暂稳态缓冲，确保机械臂末端完全静止贴准桌面后再松爪
        time.sleep(0.15)

        with state.lock:
            state.carrying_arm = None
            state.carrying_cube = None
        state.add_log(f"👐 [{arm_id.upper()}] 贴地释放方块 {cube_key}...")
        node.send_gripper(arm_id, 55.0, effort)
        time.sleep(0.7)

        spd_f = state.get_arm_speed_factor(arm_id)
        state.add_log(f"⬆️ [{arm_id.upper()}] 垂直抬升脱离...")
        node.send_trajectory(arm_id, wp_pl_up, 2.2 / spd_f)

        spd_f = state.get_arm_speed_factor(arm_id)
        if retreat_home:
            state.add_log(f"🏠 [{arm_id.upper()}] 放置完成，直接退回安全折叠原点...")
            q_home = get_home_joints_rad()
            theta_exit = float(wp_pl_up[-1][0])
            q_exit = choose_safe_ready_pose(theta_exit, wp_pl_up[-1], q_home, cam_pos)
            node.send_trajectory(arm_id, [q_exit, q_home], 3.6 / spd_f)
            node.send_gripper(arm_id, 0.0, effort)
        else:
            q_yield = get_safe_yield_pose(arm_id)
            state.add_log(f"🛡️ [{arm_id.upper()}] 退至外侧安全区等待 (无需回原点，距对侧>81cm)...")
            node.send_trajectory(arm_id, [q_yield], 2.6 / spd_f)
            node.send_gripper(arm_id, 0.0, effort)
        return True
    finally:
        if yield_thread is not None and yield_thread.is_alive():
            yield_thread.join(timeout=3.0)
        with state.lock:
            state.carrying_arm = None
            state.carrying_cube = None


def run_cube_swap_workflow(target_step="all"):
    """
    双方块堆叠与空间互换总调度工作流
    target_step: "all" (执行全流程 1->2->3->4) 或 1, 2, 3, 4
    """
    with state.lock:
        if state.task_busy or state.swap_workflow_busy:
            state.add_log("⚠️ 系统繁忙，请稍后再试！")
            return
        state.task_busy = True
        state.swap_workflow_busy = True
        state.abort_workflow_flag = False
        # 一经启动，立即锁死初始记忆位置，防止分步等待期间被后台检测改写
        state.anchors_locked = True

    try:
        cfg = state.config
        T_ab_mat = cfg.get("T_baseA_baseB")
        if T_ab_mat is None:
            state.add_log("❌ 尚未标定双臂相对变换关系 (T_baseA_baseB)，请先完成双臂手眼标定！")
            return
        T_baseA_baseB = np.array(T_ab_mat, dtype=float)
        T_baseB_baseA = np.linalg.inv(T_baseA_baseB)

        tz_a = float(cfg["arms"]["arm_a"].get("table_z_base", -0.005))
        tz_b = float(cfg["arms"]["arm_b"].get("table_z_base", -0.005))

        # 公共区域黄金交汇点 P_mid (位于两臂中垂线上)
        p_mid_A = np.array([0.180, -0.315, tz_a + 0.004])
        p_mid_B = (T_baseB_baseA @ np.array([p_mid_A[0], p_mid_A[1], p_mid_A[2], 1.0]))[:3]

        with state.lock:
            init_A = state.cubes["A"]["init_p_base"]
            init_B = state.cubes["B"]["init_p_base"]
            init_edge_A = state.cubes["A"]["init_edge_dir"]
            init_edge_B = state.cubes["B"]["init_edge_dir"]
            curr_A = state.cubes["A"]["p_base"]
            curr_B = state.cubes["B"]["p_base"]
            curr_edge_A = state.cubes["A"]["edge_dir"]
            curr_edge_B = state.cubes["B"]["edge_dir"]

        p_init_A = init_A if init_A is not None else curr_A
        p_init_B = init_B if init_B is not None else curr_B
        edge_init_A = init_edge_A if init_edge_A is not None else curr_edge_A
        edge_init_B = init_edge_B if init_edge_B is not None else curr_edge_B

        if p_init_A is None or p_init_B is None:
            state.add_log("❌ 未能识别到双方块初始坐标！请确保左侧放方块A、右侧放方块B后再试。")
            state.anchors_locked = False
            return

        # 强防呆校验：确保记录到的初始点不是公共交接区（间距 < 70mm 说明点位被污染了）
        dist_a_mid = float(np.hypot(p_init_A[0] - p_mid_A[0], p_init_A[1] - p_mid_A[1]))
        dist_b_mid = float(np.hypot(p_init_B[0] - p_mid_B[0], p_init_B[1] - p_mid_B[1]))
        if dist_a_mid < 0.070 or dist_b_mid < 0.070:
            state.add_log(
                f"❌ 初始记忆点异常 (A距公区={dist_a_mid*1000:.0f}mm, B距公区={dist_b_mid*1000:.0f}mm)！"
                "请将方块摆回左/右侧初始位置并点击【🔄 重置记忆点】！"
            )
            state.anchors_locked = False
            return

        with state.lock:
            state.cubes["A"]["init_p_base"] = p_init_A.copy()
            state.cubes["B"]["init_p_base"] = p_init_B.copy()
            if edge_init_A is not None: state.cubes["A"]["init_edge_dir"] = edge_init_A.copy()
            if edge_init_B is not None: state.cubes["B"]["init_edge_dir"] = edge_init_B.copy()

        # 用于反推真实桌面高度的顶面参考值
        top_z_A = float(p_init_A[2])
        top_z_B = float(p_init_B[2])

        steps_to_run = [1, 2, 3, 4] if target_step == "all" else [int(target_step)]

        for step in steps_to_run:
            if state.abort_workflow_flag:
                state.add_log("🛑 双方块协同工作流已被用户安全中止")
                break

            # ------------------------------------------------------------------
            # 步骤 1: 机械臂 A 将 方块 A 运送至公共区域 (底座)，同时摆正对齐坐标轴
            # ------------------------------------------------------------------
            if step == 1:
                state.swap_workflow_status = "阶段 1/4: 双臂同步启动 (A搬底座 + B进避让)"
                state.add_log("🧱 [阶段 1/4] 双臂同步启动：机械臂 A 抓取方块 A 送往公共底座，同时机械臂 B 进入避让待命位...")
                # 实时获取最新的方块A坐标与角度以精准夹取 (与单臂原地抓取完全一致)
                with state.lock:
                    live_A = state.cubes["A"]["p_base"]
                    live_eA = state.cubes["A"]["edge_dir"]
                    det_A = state.cubes["A"]["detected"]
                if det_A and live_A is not None:
                    pick_pt = live_A.copy()
                    pick_ed = live_eA.copy() if live_eA is not None else edge_init_A
                    top_z_A = float(pick_pt[2])
                    state.add_log(f"🎯 [ARM_A] 实时锁定方块 A: ({pick_pt[0]*1000:.1f}, {pick_pt[1]*1000:.1f}, {pick_pt[2]*1000:.1f})mm")
                else:
                    pick_pt = p_init_A
                    pick_ed = edge_init_A
                    state.add_log(f"⚠️ [ARM_A] 采用记忆方块 A 位姿: ({pick_pt[0]*1000:.1f}, {pick_pt[1]*1000:.1f})mm")

                ok = execute_subtask_pick_place(
                    arm_id="arm_a",
                    p_pick_base=pick_pt,
                    p_place_base=p_mid_A,
                    pick_edge_dir=pick_ed,
                    place_edge_dir=np.array([1.0, 0.0, 0.0]), # 放置到中间时强制摆正
                    cube_top_z_ref=top_z_A,
                    cube_key="A",
                    is_stack_place=False,
                    is_stack_pick=False,
                    async_yield_other=True
                )
                if not ok:
                    state.add_log("❌ [阶段 1/4] 执行失败，已安全中止")
                    return
                with state.lock:
                    state.swap_workflow_step = 1
                state.add_log("✅ [阶段 1/4] 成功完成！方块 A 已稳置底座，A臂已退至前向避让位。")
                time.sleep(0.8)

            # ------------------------------------------------------------------
            # 步骤 2: 机械臂 B 将 方块 B 堆叠在 方块 A 上方 (+25mm)
            # ------------------------------------------------------------------
            elif step == 2:
                state.swap_workflow_status = "阶段 2/4: B臂搬运方块B堆叠在方块A上方"
                state.add_log("🧱 [阶段 2/4] 机械臂 B 抓取方块 B -> 堆叠在方块 A 顶面 (+25mm) 并摆正 -> 避让让位...")
                # 核心修复：直接读取视觉最新的方块 B 实时基座位姿 (与B臂原地抓取保持完全同等的高精度逻辑)
                with state.lock:
                    live_B = state.cubes["B"]["p_base"]
                    live_eB = state.cubes["B"]["edge_dir"]
                    det_B = state.cubes["B"]["detected"]
                if det_B and live_B is not None:
                    pick_pt = live_B.copy()
                    pick_ed = live_eB.copy() if live_eB is not None else edge_init_B
                    top_z_B = float(pick_pt[2])
                    state.add_log(f"🎯 [ARM_B] 实时锁定方块 B: ({pick_pt[0]*1000:.1f}, {pick_pt[1]*1000:.1f}, {pick_pt[2]*1000:.1f})mm")
                else:
                    pick_pt = p_init_B
                    pick_ed = edge_init_B
                    state.add_log(f"⚠️ [ARM_B] 采用记忆方块 B 位姿: ({pick_pt[0]*1000:.1f}, {pick_pt[1]*1000:.1f})mm")

                ok = execute_subtask_pick_place(
                    arm_id="arm_b",
                    p_pick_base=pick_pt,
                    p_place_base=p_mid_B,
                    pick_edge_dir=pick_ed,
                    place_edge_dir=np.array([1.0, 0.0, 0.0]),
                    cube_top_z_ref=top_z_B,
                    cube_key="B",
                    is_stack_place=True,
                    is_stack_pick=False
                )
                if not ok:
                    state.add_log("❌ [阶段 2/4] 执行失败，已安全中止")
                    return
                with state.lock:
                    state.swap_workflow_step = 2
                state.add_log("🎉 [阶段 2/4] 成功完成！方块 B 已成功叠在方块 A 上方！B臂已退至前向避让位。")
                time.sleep(0.8)

            # ------------------------------------------------------------------
            # 步骤 3: 机械臂 A 从公共区域取下顶层方块 B，送至方块 A 的初始位置
            # ------------------------------------------------------------------
            elif step == 3:
                state.swap_workflow_status = "阶段 3/4: A臂取下顶层方块B送至A初始位并回归原点"
                state.add_log("🧱 [阶段 3/4] 机械臂 A 前出取下顶层方块 B (+25mm) -> 送往方块 A 初始位置 -> 直接回归原点...")
                # 若视觉在公共区看到了实时方块，直接修正微偏并对齐真实角度，同时记录顶层实测高度
                mid_pick_pt = p_mid_A.copy()
                mid_pick_ed = np.array([1.0, 0.0, 0.0])
                mid_pick_top_z_obs = None
                with state.lock:
                    for ck in ("A", "B"):
                        c_cam = state.cubes[ck]["p_cam"]
                        c_yaw = state.cubes[ck]["yaw_cam"]
                        if state.cubes[ck]["detected"] and c_cam is not None:
                            pb_chk, pe_chk = state.transform_cam_to_arm("arm_a", c_cam, c_yaw)
                            if pb_chk is not None and np.hypot(pb_chk[0] - p_mid_A[0], pb_chk[1] - p_mid_A[1]) < 0.045:
                                mid_pick_pt[0], mid_pick_pt[1] = pb_chk[0], pb_chk[1]
                                mid_pick_top_z_obs = float(pb_chk[2])
                                if pe_chk is not None: mid_pick_ed = pe_chk
                                break

                ok = execute_subtask_pick_place(
                    arm_id="arm_a",
                    p_pick_base=mid_pick_pt,
                    p_place_base=p_init_A,
                    pick_edge_dir=mid_pick_ed,
                    place_edge_dir=edge_init_A if edge_init_A is not None else np.array([1.0, 0.0, 0.0]),
                    cube_top_z_ref=top_z_A,
                    cube_key="B",
                    is_stack_place=False,
                    is_stack_pick=True,
                    retreat_home=True,
                    pick_top_z_obs=mid_pick_top_z_obs
                )
                if not ok:
                    state.add_log("❌ [阶段 3/4] 执行失败，已安全中止")
                    return
                with state.lock:
                    state.swap_workflow_step = 3
                state.add_log("✅ [阶段 3/4] 成功完成！顶层方块 B 已安置在方块 A 初始位置，A臂已直接回归原点。")
                time.sleep(0.8)

            # ------------------------------------------------------------------
            # 步骤 4: 机械臂 B 从公共区域取下底层方块 A，送至方块 B 的初始位置
            # ------------------------------------------------------------------
            elif step == 4:
                state.swap_workflow_status = "阶段 4/4: B臂取下底层方块A送至B初始位"
                state.add_log("🧱 [阶段 4/4] 机械臂 B 前出取下底层方块 A -> 送往方块 B 初始位置 -> 归位完成...")
                mid_pick_pt = p_mid_B.copy()
                mid_pick_ed = np.array([1.0, 0.0, 0.0])
                with state.lock:
                    for ck in ("A", "B"):
                        c_cam = state.cubes[ck]["p_cam"]
                        c_yaw = state.cubes[ck]["yaw_cam"]
                        if state.cubes[ck]["detected"] and c_cam is not None:
                            pb_chk, pe_chk = state.transform_cam_to_arm("arm_b", c_cam, c_yaw)
                            if pb_chk is not None and np.hypot(pb_chk[0] - p_mid_B[0], pb_chk[1] - p_mid_B[1]) < 0.045:
                                mid_pick_pt[0], mid_pick_pt[1] = pb_chk[0], pb_chk[1]
                                if pe_chk is not None: mid_pick_ed = pe_chk
                                break

                ok = execute_subtask_pick_place(
                    arm_id="arm_b",
                    p_pick_base=mid_pick_pt,
                    p_place_base=p_init_B,
                    pick_edge_dir=mid_pick_ed,
                    place_edge_dir=edge_init_B if edge_init_B is not None else np.array([1.0, 0.0, 0.0]),
                    cube_top_z_ref=top_z_B,
                    cube_key="A",
                    is_stack_place=False,
                    is_stack_pick=False,
                    retreat_home=True,
                    allow_other_home=True
                )
                if not ok:
                    state.add_log("❌ [阶段 4/4] 执行失败，已安全中止")
                    return
                with state.lock:
                    state.swap_workflow_step = 4
                    state.anchors_locked = False # 完整流程全部结束，允许后续重新锁定新位置
                state.add_log("🏆 [阶段 4/4] 双方块【堆叠与空间互换】全流程圆满成功完成！")
                # 修复核心：直接调用内部回原点函数（不受当前工作流 task_busy 锁定拦截），确保 A 臂与 B 臂双双安全退回折叠原点 (Home)
                state.add_log("🏠 全流程圆满结束，正在指挥双臂全部平稳退回安全折叠原点 (Home)...")
                try:
                    _execute_retract_home(arm_id="all", skip_if_at_home=True)
                except Exception as _e_hm:
                    state.add_log(f"⚠️ 双臂回原点通知: {_e_hm}")

        if state.swap_workflow_step == 4:
            state.swap_workflow_status = "🎉 工作流完成 (双方块空间互换已实现)"
        else:
            state.swap_workflow_status = f"暂停于第 {state.swap_workflow_step}/4 步 (初始记忆已安全锁定，可继续下一步)"
    except Exception as e:
        state.add_log(f"🔴 工作流异常: {e}")
    finally:
        with state.lock:
            state.task_busy = False
            state.swap_workflow_busy = False


# ==============================================================================
# HTTP 服务与接口路由
# ==============================================================================
class VisualHttpHandler(BaseHTTPRequestHandler):
    def log_message(self, format, *args):
        pass

    def _send_json(self, data, status=200):
        raw = json.dumps(data, ensure_ascii=False).encode('utf-8')
        self.send_response(status)
        self.send_header('Content-Type', 'application/json; charset=utf-8')
        self.send_header('Cache-Control', 'no-store')
        self.send_header('Content-Length', str(len(raw)))
        self.end_headers()
        self.wfile.write(raw)

    def _send_jpeg(self, jpeg_bytes: bytes):
        self.send_response(200)
        self.send_header('Content-Type', 'image/jpeg')
        self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
        self.send_header('Content-Length', str(len(jpeg_bytes)))
        self.end_headers()
        self.wfile.write(jpeg_bytes)

    @staticmethod
    def _build_calib_payload(arm_id: str, cal_pts: list, pending_cam):
        preview_rmse = None
        pt_errs = []
        if len(cal_pts) >= 4:
            _, rmse, errs, _ = solve_rigid_transform_svd(
                [p["p_cam"] for p in cal_pts],
                [p["p_base"] for p in cal_pts]
            )
            preview_rmse = round(rmse, 2)
            pt_errs = [round(e, 2) for e in errs]
        return {
            "target_arm": arm_id,
            "touch_mode": state.calib_touch_mode,
            "pending_cam_mm": [round(x * 1000, 1) for x in pending_cam] if pending_cam is not None else None,
            "points": [
                {
                    "idx": i + 1,
                    "p_cam_mm": [round(x * 1000, 1) for x in p["p_cam"]],
                    "p_base_mm": [round(x * 1000, 1) for x in p["p_base"]],
                    "err_mm": pt_errs[i] if i < len(pt_errs) else None
                }
                for i, p in enumerate(cal_pts)
            ],
            "preview_rmse_mm": preview_rmse,
        }

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path in ('/', '/index.html'):
            html_str = render_single_arm_html() if MODE == 'single' else render_dual_arm_html()
            html = html_str.encode('utf-8')
            self.send_response(200)
            self.send_header('Content-Type', 'text/html; charset=utf-8')
            self.send_header('Content-Length', str(len(html)))
            self.end_headers()
            self.wfile.write(html)

        elif parsed.path == '/api/latest_frame.jpg':
            with state.lock:
                jpeg = state.annotated_jpeg
            self._send_jpeg(jpeg)

        elif parsed.path == '/api/hand_cam_frame.jpg':
            with state.lock:
                jpeg = state.hand_cam_jpeg
            self._send_jpeg(jpeg)

        elif parsed.path == '/api/status':
            now = time.time()
            with state.lock:
                cam_online = (now - state.last_img_time < 2.2)
                hand_online = (now - state.hand_cam_time < 2.5)
                cfg = state.config
                tag_cam = state.tag_pos_cam
                pa, _ = state.transform_cam_to_arm("arm_a", tag_cam) if tag_cam is not None else (None, None)
                pb, _ = state.transform_cam_to_arm("arm_b", tag_cam) if tag_cam is not None else (None, None)

                dual_dist_mm = None
                if cfg.get("T_baseA_baseB") is not None:
                    t_ab = np.array(cfg["T_baseA_baseB"], dtype=float)[:3, 3]
                    dual_dist_mm = round(float(np.linalg.norm(t_ab)) * 1000.0, 1)

                ref_arm = state.get_world_ref_arm()
                if cfg.get("T_baseA_baseB") is not None and MODE == 'dual':
                    t_ab = np.array(cfg["T_baseA_baseB"], dtype=float)[:3, 3]
                    traj_center = [240.0, round(float(t_ab[1]) * 500.0, 1), 0.0]
                else:
                    traj_center = [240.0, -50.0 if ref_arm == "arm_a" else 50.0, 0.0]

                cal_arm = PRIMARY_ARM if MODE == 'single' else state.calib_target_arm
                calib_arms = {
                    aid: self._build_calib_payload(aid, state.calib_points[aid], state.calib_pending_cam[aid])
                    for aid in ("arm_a", "arm_b")
                }

                data = {
                    "mode": MODE,
                    "primary_arm": PRIMARY_ARM,
                    "camera_online": cam_online,
                    "hand_cam_online": hand_online,
                    "hand_cam_topic": state.hand_cam_topic,
                    "camera_intrinsics_ok": (state.camera_matrix is not None),
                    "tag_detected": state.tag_detected,
                    "tag_id": state.tag_id,
                    "tag_cam_mm": [round(float(x) * 1000, 1) for x in tag_cam] if tag_cam is not None else None,
                    "tag_arm_a_mm": [round(float(x) * 1000, 1) for x in pa] if pa is not None else None,
                    "tag_arm_b_mm": [round(float(x) * 1000, 1) for x in pb] if pb is not None else None,
                    "raw_can_interfaces": state.raw_can_interfaces,
                    "arms": {
                        aid: {
                            "label": cfg["arms"][aid]["label"],
                            "calibrated": bool(cfg["arms"][aid].get("calibrated")),
                            "calib_rmse_mm": cfg["arms"][aid].get("calib_rmse_mm"),
                            "calib_points_count": cfg["arms"][aid].get("calib_points_count", 0),
                            "grasp_effort": cfg["arms"][aid].get("grasp_effort", 350),
                            "grasp_target_mm": cfg["arms"][aid].get("grasp_target_mm", 26.5),
                            "grasp_x_offset_mm": cfg["arms"][aid].get("grasp_x_offset_mm", 0.0),
                            "grasp_y_offset_mm": cfg["arms"][aid].get("grasp_y_offset_mm", 0.0),
                            "grasp_z_offset_mm": cfg["arms"][aid].get("grasp_z_offset_mm", -25.5),
                            "speed_pct": cfg["arms"][aid].get("speed_pct", 100),
                            "runtime": {k: v for k, v in state.arm_runtime[aid].items() if k != "joints"},
                        }
                        for aid in ("arm_a", "arm_b")
                    },
                    "cubes": {
                        "A": {
                            "detected": state.cubes["A"]["detected"],
                            "id": state.cubes["A"]["id"],
                            "p_cam_mm": [round(float(x) * 1000, 1) for x in state.cubes["A"]["p_cam"]] if state.cubes["A"]["p_cam"] is not None else None,
                            "p_base_mm": [round(float(x) * 1000, 1) for x in state.cubes["A"]["p_base"]] if state.cubes["A"]["p_base"] is not None else None,
                            "init_p_base_mm": [round(float(x) * 1000, 1) for x in state.cubes["A"]["init_p_base"]] if state.cubes["A"]["init_p_base"] is not None else None,
                        },
                        "B": {
                            "detected": state.cubes["B"]["detected"],
                            "id": state.cubes["B"]["id"],
                            "p_cam_mm": [round(float(x) * 1000, 1) for x in state.cubes["B"]["p_cam"]] if state.cubes["B"]["p_cam"] is not None else None,
                            "p_base_mm": [round(float(x) * 1000, 1) for x in state.cubes["B"]["p_base"]] if state.cubes["B"]["p_base"] is not None else None,
                            "init_p_base_mm": [round(float(x) * 1000, 1) for x in state.cubes["B"]["init_p_base"]] if state.cubes["B"]["init_p_base"] is not None else None,
                        }
                    },
                    "swap_workflow": {
                        "busy": state.swap_workflow_busy,
                        "step": state.swap_workflow_step,
                        "anchors_locked": state.anchors_locked,
                        "status": state.swap_workflow_status
                    },
                    "has_dual_rel_tf": (cfg.get("T_baseA_baseB") is not None),
                    "dual_dist_mm": dual_dist_mm,
                    "clicked_uv": state.clicked_uv,
                    "clicked_eval": state.clicked_eval,
                    "selected_arm_mode": state.selected_arm_mode,
                    "task_busy": state.task_busy,
                    "task_status": state.task_status,
                    "logs": list(state.task_logs),
                    "calib": calib_arms[cal_arm],
                    "calib_arms": calib_arms,
                    "traj_3d": {
                        "points": list(state.traj_points),
                        "curr": state.traj_curr,
                        "a": {"points": list(state.traj_a_points), "curr": state.traj_a_curr},
                        "b": {"points": list(state.traj_b_points), "curr": state.traj_b_curr},
                        "center": traj_center,
                    },
                }
            self._send_json(data)
        else:
            self.send_error(404)

    def do_POST(self):
        parsed = urlparse(self.path)
        length = int(self.headers.get('Content-Length', 0))
        raw = self.rfile.read(length).decode('utf-8') if length > 0 else '{}'
        try:
            body = json.loads(raw)
        except Exception:
            body = {}

        default_arm = PRIMARY_ARM if MODE == 'single' else body.get("arm_id", state.calib_target_arm)
        if default_arm not in ("arm_a", "arm_b"):
            default_arm = PRIMARY_ARM

        if parsed.path == '/api/workflow/swap_cubes':
            step = body.get("step", "all")
            t = threading.Thread(target=run_cube_swap_workflow, args=(step,), daemon=True)
            t.start()
            self._send_json({"status": "ok", "step": step})

        elif parsed.path == '/api/workflow/abort':
            with state.lock:
                state.abort_workflow_flag = True
            state.add_log("🛑 收到用户紧急停止指令，正在安全刹停当前动作...")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/workflow/reset_anchors':
            with state.lock:
                state.anchors_locked = False
                for ck in ("A", "B"):
                    state.cubes[ck]["init_p_base"] = None
                    state.cubes[ck]["init_p_cam"] = None
                    state.cubes[ck]["init_edge_dir"] = None
                state.swap_workflow_step = 0
                state.swap_workflow_status = "待命 (请将方块A置于左侧，方块B置于右侧)"
            state.add_log("🔄 已解锁并重置双方块初始位姿记忆与步进状态！")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/start_camera':
            subprocess.Popen(["bash", os.path.join(WORK_DIR, "start_realsense.sh")])
            state.add_log("📷 正在后台拉起 RealSense D435 驱动 (约需 3~5 秒)...")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/set_hand_cam_source':
            src = str(body.get("source", "/hand_camera/color/image_raw")).strip()
            if src:
                worker_node.update_hand_cam_topic(src)
                state.add_log(f"📷 预留手眼一体相机话题已设为: {src}")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/clear_trajectory':
            with state.lock:
                state.traj_points.clear()
                if state.traj_curr is not None:
                    state.traj_points.append(state.traj_curr)
                state.traj_a_points.clear()
                if state.traj_a_curr is not None:
                    state.traj_a_points.append(state.traj_a_curr)
                state.traj_b_points.clear()
                if state.traj_b_curr is not None:
                    state.traj_b_points.append(state.traj_b_curr)
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/reconnect_arms':
            worker_node.send_dual_command({"cmd": "reconnect"})
            state.add_log("🔌 正在重新扫描双臂 CAN 总线...")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/enable_arm':
            targets = [PRIMARY_ARM] if MODE == 'single' else ([body["arm_id"]] if body.get("arm_id") in ("arm_a", "arm_b") else ["arm_a", "arm_b"])
            for aid in targets:
                worker_node.send_dual_command({"cmd": "enable_arm", "arm": aid})
            state.add_log(f"⚡ 已发送上电使能与 CAN 伺服锁定指令 -> {targets}")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/wiggle_arm':
            worker_node.send_dual_command({"cmd": "wiggle", "arm": default_arm})
            state.add_log(f"👋 已向 [{default_arm.upper()}] 发送安全认臂轻摇指令！")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/swap_ab':
            worker_node.send_dual_command({"cmd": "swap_ab"})
            time.sleep(0.25)
            with state.lock:
                state.config = load_dual_config()
            state.add_log("🔄 已互换 机械臂 A 与 机械臂 B 硬件端口映射！")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/arm_teach_mode':
            enable = bool(body.get("enable", True))
            worker_node.send_dual_command({"cmd": "enter_teach" if enable else "exit_teach", "arm": default_arm})
            state.add_log(f"{'🔓' if enable else '🔒'} [{default_arm.upper()}] {'已切入自由示教拖动' if enable else '已锁回 CAN 伺服'}")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/calib/select_arm':
            with state.lock:
                state.calib_target_arm = default_arm
                state.calib_touch_mode = body.get("touch_mode", "tip_touch")
                state.calib_pending_cam[default_arm] = None
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/calib/capture_cam':
            samples = []
            t0 = time.time()
            while len(samples) < 10 and (time.time() - t0 < 2.0):
                with state.lock:
                    if state.tag_detected and state.tag_pos_cam is not None:
                        samples.append(state.tag_pos_cam.copy())
                time.sleep(0.06)
            if len(samples) < 3:
                self._send_json({"status": "error", "msg": "未稳定识别到 AprilTag，请拿开遮挡后重试！"})
                return
            p_cam = np.median(np.array(samples), axis=0).tolist()
            with state.lock:
                state.calib_pending_cam[default_arm] = p_cam
            state.add_log(f"📸 [{default_arm.upper()}] 锁存方块相机坐标 ({p_cam[0]*1000:.1f}, {p_cam[1]*1000:.1f}, {p_cam[2]*1000:.1f})mm")
            self._send_json({"status": "ok", "p_cam": p_cam})

        elif parsed.path == '/api/calib/record_arm':
            with state.lock:
                p_cam = state.calib_pending_cam[default_arm]
                tmode = state.calib_touch_mode
                q = state.arm_runtime[default_arm]["joints"]
                alive = state.arm_runtime[default_arm]["motors_alive"]

            if p_cam is None:
                self._send_json({"status": "error", "msg": "请先点击【1. 视觉锁定方块】！"})
                return
            if not alive or q is None:
                self._send_json({"status": "error", "msg": f"{default_arm} 24V未上电或 CAN 离线，请检查电源！"})
                return

            tip_offset = 0.136 if tmode == "tip_touch" else (0.136 - 0.015)
            _, _, T_tip = piper_all_links_fk(q, tip_offset_z=tip_offset)
            p_base = T_tip[:3, 3].tolist()

            with state.lock:
                state.calib_points[default_arm].append({"p_cam": list(p_cam), "p_base": p_base, "joints": q.tolist()})
                state.calib_pending_cam[default_arm] = None
                cnt = len(state.calib_points[default_arm])

            state.add_log(f"📍 [{default_arm.upper()}] 记录第 {cnt} 点成功! P_base=({p_base[0]*1000:.1f},{p_base[1]*1000:.1f},{p_base[2]*1000:.1f})mm")
            self._send_json({"status": "ok", "count": cnt})

        elif parsed.path == '/api/calib/delete_point':
            idx = int(body.get("idx", -1))
            with state.lock:
                pts = state.calib_points[default_arm]
                if 0 <= idx < len(pts):
                    pts.pop(idx)
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/calib/clear_points':
            with state.lock:
                state.calib_points[default_arm] = []
                state.calib_pending_cam[default_arm] = None
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/calib/solve_and_save':
            with state.lock:
                pts = list(state.calib_points[default_arm])
            if len(pts) < 4:
                self._send_json({"status": "error", "msg": f"当前仅采集 {len(pts)} 点，至少需 4 个分散点！"})
                return

            pts_c = np.array([p["p_cam"] for p in pts])
            pts_b = np.array([p["p_base"] for p in pts])
            T, rmse, _, _ = solve_rigid_transform_svd(pts_c, pts_b)
            if T[2, 3] < 0.05:
                self._send_json({"status": "error", "msg": f"相机相对{default_arm}高度异常({T[2,3]*1000:.0f}mm)！"})
                return

            table_z_est = float(np.median(pts_b[:, 2]) - 0.030)
            with state.lock:
                cfg = state.config
                acfg = cfg["arms"][default_arm]
                acfg["calibrated"] = True
                acfg["T_base_cam"] = T.tolist()
                acfg["table_z_base"] = round(table_z_est, 5)
                acfg["calib_rmse_mm"] = round(rmse, 2)
                acfg["calib_points_count"] = len(pts)
                save_dual_config(cfg)
                state.config = load_dual_config()
                state.config_mtime = os.path.getmtime(CONFIG_PATH)

            worker_node.send_dual_command({"cmd": "exit_teach", "arm": default_arm})
            state.add_log(f"🎉 [{default_arm.upper()}] 外参标定保存成功！RMSE={rmse:.2f}mm, 相机高 Z={T[2,3]*1000:.0f}mm")
            self._send_json({"status": "ok", "rmse_mm": round(rmse, 2)})

        elif parsed.path == '/api/select_point':
            # 由于网页展示画面旋转了 180 度，点击坐标 (u, v) 映射回原相机真实坐标系 (639 - u, 479 - v)
            raw_u = int(body.get('u', 320))
            raw_v = int(body.get('v', 240))
            u = max(0, min(639, 639 - raw_u))
            v = max(0, min(479, 479 - raw_v))
            p_cam = worker_node.get_depth_3d_cam(u, v)
            if p_cam is None:
                self._send_json({"status": "error", "msg": "相机尚未就绪"})
                return
            ev_map = {aid: worker_node.evaluate_arm_point(aid, p_cam, is_place=True) for aid in ("arm_a", "arm_b")}
            with state.lock:
                state.clicked_uv = (u, v)
                state.clicked_cam_pos = p_cam
                state.clicked_eval = ev_map
            self._send_json({"status": "ok", "eval": ev_map})

        elif parsed.path == '/api/clear_point':
            with state.lock:
                state.clicked_uv = None
                state.clicked_cam_pos = None
                state.clicked_eval = {}
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/set_arm_mode':
            mode = body.get("mode", "auto")
            with state.lock:
                state.selected_arm_mode = PRIMARY_ARM if MODE == 'single' else (mode if mode in ("auto", "arm_a", "arm_b", "relay") else "auto")
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/set_gripper':
            eff = int(max(150, min(1500, float(body.get('effort', 350)))))
            wid = round(max(15.0, min(29.5, float(body.get('width_mm', 26.5)))), 1)
            zoff = round(max(-32.0, min(-10.0, float(body.get('z_offset_mm', -25.5)))), 1)
            xoff = round(max(-25.0, min(25.0, float(body.get('x_offset_mm', 0.0)))), 1)
            yoff = round(max(-25.0, min(25.0, float(body.get('y_offset_mm', 0.0)))), 1)
            spd = int(max(20, min(150, float(body.get('speed_pct', 100)))))
            with state.lock:
                cfg = state.config
                target_list = ("arm_a", "arm_b") if body.get("arm_id") == "all" else (default_arm,)
                for aid in target_list:
                    if aid in cfg["arms"]:
                        cfg["arms"][aid]["grasp_effort"] = eff
                        cfg["arms"][aid]["grasp_target_mm"] = wid
                        cfg["arms"][aid]["grasp_z_offset_mm"] = zoff
                        cfg["arms"][aid]["grasp_x_offset_mm"] = xoff
                        cfg["arms"][aid]["grasp_y_offset_mm"] = yoff
                        cfg["arms"][aid]["speed_pct"] = spd
                save_dual_config(cfg)
                state.config_mtime = os.path.getmtime(CONFIG_PATH)
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/pick_and_return':
            if state.task_busy:
                self._send_json({"status": "error", "msg": "机械臂正在忙，请稍候！"})
                return
            arm_ovr = PRIMARY_ARM if MODE == 'single' else body.get("arm_mode")
            threading.Thread(target=run_task_worker, args=('return', None, arm_ovr), daemon=True).start()
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/pick_and_place':
            if state.task_busy:
                self._send_json({"status": "error", "msg": "机械臂正在忙，请稍候！"})
                return
            with state.lock:
                c_cam = state.clicked_cam_pos.copy() if state.clicked_cam_pos is not None else None
            if c_cam is None:
                self._send_json({"status": "error", "msg": "请先在左侧画面上点击选定目标放置点！"})
                return
            arm_ovr = PRIMARY_ARM if MODE == 'single' else body.get("arm_mode")
            threading.Thread(target=run_task_worker, args=('place', c_cam, arm_ovr), daemon=True).start()
            self._send_json({"status": "ok"})

        elif parsed.path == '/api/retract_home':
            if state.task_busy:
                self._send_json({"status": "error", "msg": "任务正在执行中！"})
                return
            ret_target = PRIMARY_ARM if MODE == 'single' else body.get("arm_id", "all")
            threading.Thread(target=run_retract_home_worker, args=(ret_target,), daemon=True).start()
            self._send_json({"status": "ok"})

        else:
            self.send_error(404)


def main():
    global worker_node
    rclpy.init()
    worker_node = DualArmVisualWorkerNode()

    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(worker_node)
    threading.Thread(target=executor.spin, daemon=True).start()

    ThreadingHTTPServer.allow_reuse_address = True
    server = ThreadingHTTPServer(('0.0.0.0', PORT), VisualHttpHandler)
    print("=" * 68)
    print(f"🚀 PIPER 视觉抓放与 3D 轨迹总控台上线 (mode={MODE}, port={PORT})")
    print(f"🌐 浏览器访问: http://192.168.31.70:{PORT}")
    print("=" * 68)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        server.server_close()
        worker_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
