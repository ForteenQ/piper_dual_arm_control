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

        # 3D 质点轨迹跟踪缓冲
        self.traj_points = collections.deque(maxlen=600)
        self.traj_curr = None
        self.carrying_arm = None

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
        self.task_logs = collections.deque(maxlen=40)

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

    def record_cube_3d_point(self, p_world_m: np.ndarray, table_z_m: float, is_tag_top: bool = True):
        """将方块中心转为相对桌面的 3D 毫米坐标并加入轨迹 (去抖过滤>=2mm步进)"""
        z_center_m = float(p_world_m[2]) - float(table_z_m) - (0.015 if is_tag_top else 0.0)
        pt_mm = [
            round(float(p_world_m[0]) * 1000.0, 1),
            round(float(p_world_m[1]) * 1000.0, 1),
            round(max(0.0, z_center_m * 1000.0), 1),
        ]
        with self.lock:
            self.traj_curr = pt_mm
            if not self.traj_points:
                self.traj_points.append(pt_mm)
            else:
                last = self.traj_points[-1]
                dist = math.sqrt((pt_mm[0] - last[0])**2 + (pt_mm[1] - last[1])**2 + (pt_mm[2] - last[2])**2)
                if 2.0 <= dist <= 260.0:
                    self.traj_points.append(pt_mm)


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
        """在遮挡抓取搬运期间，用运动学指尖接续追踪 3D 质点"""
        with state.lock:
            carry = state.carrying_arm
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
        state.record_cube_3d_point(p_world, tz, is_tag_top=False)

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

            det = self.detector.detect(frame, K, D, depth)
            if det is not None:
                # 将角点映射到 180 度翻转图像系绘制标定框与局部坐标轴
                raw_corners = det["corners"].reshape((4, 2))
                tc_rot = np.zeros_like(raw_corners, dtype=np.float32)
                tc_rot[:, 0] = 639.0 - raw_corners[:, 0]
                tc_rot[:, 1] = 479.0 - raw_corners[:, 1]
                cv2.polylines(display, [tc_rot.astype(np.int32)], True, (0, 255, 0), 2)
                # 画出对角交叉或中心点
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
                    self.last_detect_time = time.time()

                # 更新 3D 桌面质点轨迹
                ref_arm = state.get_world_ref_arm()
                p_ref, _ = state.transform_cam_to_arm(ref_arm, smooth_cam)
                if p_ref is not None:
                    tz = float(cfg["arms"][ref_arm].get("table_z_base", -0.005))
                    state.record_cube_3d_point(p_ref, tz, is_tag_top=True)
                else:
                    state.record_cube_3d_point(
                        np.array([smooth_cam[0], smooth_cam[1], 0.515 - smooth_cam[2]]), 0.0, is_tag_top=True
                    )

                cx_px, cy_px = 639 - int(det["uv"][0]), 479 - int(det["uv"][1])
                if MODE == 'single':
                    ps, _ = state.transform_cam_to_arm(PRIMARY_ARM, smooth_cam)
                    txt = (f"ID:{det['id']} ({ps[0]*1000:.0f},{ps[1]*1000:.0f},{ps[2]*1000:.0f})"
                           if ps is not None else
                           f"ID:{det['id']} CAM:({smooth_cam[0]*1000:.0f},{smooth_cam[1]*1000:.0f},{smooth_cam[2]*1000:.0f})")
                else:
                    pa, _ = state.transform_cam_to_arm("arm_a", smooth_cam)
                    pb, _ = state.transform_cam_to_arm("arm_b", smooth_cam)
                    if pa is not None:
                        txt = f"ID:{det['id']} A:({pa[0]*1000:.0f},{pa[1]*1000:.0f},{pa[2]*1000:.0f})"
                    elif pb is not None:
                        txt = f"ID:{det['id']} B:({pb[0]*1000:.0f},{pb[1]*1000:.0f},{pb[2]*1000:.0f})"
                    else:
                        txt = f"ID:{det['id']} CAM:({smooth_cam[0]*1000:.0f},{smooth_cam[1]*1000:.0f},{smooth_cam[2]*1000:.0f})"
                cv2.putText(display, txt, (max(10, cx_px - 105), max(25, cy_px - 22)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.52, (0, 255, 0), 2)
            else:
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
    z_floor = table_z + 0.004
    z_grasp = max(z_floor, obj_pos[2] + z_off)
    z_pre = obj_pos[2] + 0.095
    z_lift = z_grasp + 0.075

    p_pre = np.array([gx, gy, z_pre])
    p_grasp = np.array([gx, gy, z_grasp])
    p_lift = np.array([gx, gy, z_lift])

    state.add_log(f"🦾 [{arm_id.upper()}] 锁定抓取点: ({gx*1000:.1f}, {gy*1000:.1f}, {z_grasp*1000:.1f})mm")

    theta_obj = math.atan2(gy, gx)
    q_pre, ok_pre, err_pre, _ = solve_hover_ik(p_pre, target_x_dir=edge_dir)
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
        z_pl_down = max(table_z + 0.008, tb_pos[2] + 0.012)
        z_pl_above = z_pl_down + 0.090
        p_pl_ab = np.array([tb_pos[0], tb_pos[1], z_pl_above])
        p_pl_dn = np.array([tb_pos[0], tb_pos[1], z_pl_down])

        q_pl_above, ok_pa, err_pa, _ = solve_hover_ik(p_pl_ab)
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

    # === 实机执行序列 ===
    try:
        # 1. 抓取前预备：立即张开夹爪至 55mm (充分敞开)
        state.add_log(f"👐 [{arm_id.upper()}] 预抓取准备: 展开夹爪至 55mm (速率:{int(spd_pct)}%)...")
        node.send_gripper(arm_id, 55.0, effort)

        if not node.send_trajectory(arm_id, [q_ready, q_pre], 4.2 / spd_f):
            state.add_log(f"❌ [{arm_id}] 飞抵悬停失败")
            return False

        # 核心防撞安全阻塞：在下探前严格校验夹爪是否已充分张开 (>= 42mm)
        # 解决断电重启后第一次抓取夹爪未提前张开导致顶压方块的隐患
        t_wait_open = time.time()
        while time.time() - t_wait_open < 1.5:
            with state.lock:
                g_cur = state.arm_runtime[arm_id].get("gripper_mm", 0.0)
            if g_cur >= 42.0:
                break
            node.send_gripper(arm_id, 55.0, effort)
            time.sleep(0.12)

        if not node.send_trajectory(arm_id, wp_down, 3.0 / spd_f):
            state.add_log(f"❌ [{arm_id}] 垂直下探失败")
            return False

        state.add_log(f"🤏 [{arm_id.upper()}] 闭合夹持方块 (目标开度:{grip_w}mm, 力矩:{effort}mN·m)...")
        node.send_gripper(arm_id, grip_w, effort)
        time.sleep(0.9)
        with state.lock:
            state.carrying_arm = arm_id

        if not node.send_trajectory(arm_id, wp_lift, 2.2 / spd_f):
            return False
        time.sleep(0.3)

        if mode == 'place' and wp_pl_down is not None:
            state.add_log(f"✈️ [{arm_id}] 搬运前往目标点...")
            if not node.send_trajectory(arm_id, [q_pl_above], 3.8 / spd_f):
                return False
            if not node.send_trajectory(arm_id, wp_pl_down, 2.8 / spd_f):
                return False
            with state.lock:
                state.carrying_arm = None
            node.send_gripper(arm_id, 50.0, effort)
            time.sleep(0.6)
            node.send_trajectory(arm_id, wp_pl_up, 2.0 / spd_f)
            q_ret_from = wp_pl_up[-1]
        else:
            state.add_log(f"⏬ [{arm_id}] 原地放回...")
            wp_ret_dn, _ = solve_cartesian_segment(wp_lift[-1], p_lift, p_grasp, R_grasp, num_steps=10)
            if wp_ret_dn:
                node.send_trajectory(arm_id, wp_ret_dn, 2.6 / spd_f)
            with state.lock:
                state.carrying_arm = None
            node.send_gripper(arm_id, 50.0, effort)
            time.sleep(0.6)
            wp_ret_up, _ = solve_cartesian_segment(
                wp_ret_dn[-1] if wp_ret_dn else wp_lift[-1], p_grasp, p_lift, R_grasp, num_steps=6
            )
            if wp_ret_up:
                node.send_trajectory(arm_id, wp_ret_up, 1.8 / spd_f)
            q_ret_from = wp_ret_up[-1] if wp_ret_up else wp_lift[-1]

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


def run_retract_home_worker(arm_id="all"):
    with state.lock:
        if state.task_busy:
            return
        state.task_busy = True
    try:
        targets = ("arm_a", "arm_b") if arm_id == "all" else (arm_id,)
        q_home = get_home_joints_rad()
        for aid in targets:
            with state.lock:
                rt = state.arm_runtime[aid]
                alive = rt["motors_alive"]
                q_curr = rt["joints"]
            if not alive or q_curr is None:
                continue
            T_bc = state.get_arm_T_base_cam(aid)
            cam_pos = T_bc[:3, 3] if T_bc is not None else None
            state.add_log(f"🏠 [{aid.upper()}] 正在退回安全折叠原点...")
            q_mid = choose_safe_ready_pose(float(q_curr[0]), q_curr, q_home, cam_pos)
            worker_node.send_trajectory(aid, [q_mid, q_home], 3.8)
            worker_node.send_gripper(aid, 0.0, 350)
        state.add_log("✅ 安全折叠原点复位完成！")
    finally:
        with state.lock:
            state.task_busy = False


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
                    traj_center = [160.0, round(float(t_ab[1]) * 500.0, 1), 0.0]
                else:
                    traj_center = [220.0, 0.0 if ref_arm != "arm_a" else 260.0, 0.0]

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

        if parsed.path == '/api/start_camera':
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
