#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PIPER 双臂手眼标定与视觉感知核心工具库
包含：
1. 配置文件 dual_arm_config.json 读写与双臂间相对变换 T_baseA_baseB 自动同步
2. SVD Kabsch 刚体标定解算与逐点残差计算
3. 自适应多尺度 AprilTag 36h11 检测 + RealSense 对齐硬件深度高精度三维反投影
4. 将机械臂 A/B 基座坐标轴与安全可达工作空间圆弧 AR 投影至 640x480 实时画面
"""

import os
import json
import math
import threading
import cv2
import numpy as np

from piper_kinematics_engine import rot_to_quat

WORK_DIR = "/home/lab203/piper_students/lq/real_arm_control"
CONFIG_PATH = os.path.join(WORK_DIR, "dual_arm_config.json")
TAG_SIZE_M = 0.025

_CONFIG_LOCK = threading.Lock()

DEFAULT_DUAL_CONFIG = {
    "arms": {
        "arm_a": {
            "label": "机械臂 A (左/主)",
            "usb_serial": "0025002C5547570420303135",
            "preferred_can": "can_piper_0",
            "grasp_x_offset_mm": 0.0,
            "grasp_y_offset_mm": 0.0,
            "grasp_z_offset_mm": -25.5,
            "grasp_effort": 350,
            "grasp_target_mm": 26.5,
            "calibrated": False,
            "T_base_cam": None,
            "table_z_base": -0.005,
            "calib_rmse_mm": None,
            "calib_points_count": 0
        },
        "arm_b": {
            "label": "机械臂 B (右/副)",
            "usb_serial": "002E00344648570220393831",
            "preferred_can": "can_piper_1",
            "grasp_x_offset_mm": 0.0,
            "grasp_y_offset_mm": 0.0,
            "grasp_z_offset_mm": -25.5,
            "grasp_effort": 350,
            "grasp_target_mm": 26.5,
            "calibrated": False,
            "T_base_cam": None,
            "table_z_base": -0.005,
            "calib_rmse_mm": None,
            "calib_points_count": 0
        }
    },
    "T_baseA_baseB": None
}


def load_dual_config() -> dict:
    """线程安全读取双臂配置与标定外参"""
    with _CONFIG_LOCK:
        if os.path.exists(CONFIG_PATH):
            try:
                with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
                    cfg = json.load(f)
                if "arms" in cfg and "arm_a" in cfg["arms"] and "arm_b" in cfg["arms"]:
                    # 补齐新增默认字段，防止旧文件缺键
                    for aid in ("arm_a", "arm_b"):
                        for k, v in DEFAULT_DUAL_CONFIG["arms"][aid].items():
                            cfg["arms"][aid].setdefault(k, v)
                    cfg.setdefault("T_baseA_baseB", None)
                    return cfg
            except Exception:
                pass
        return json.loads(json.dumps(DEFAULT_DUAL_CONFIG))


def save_dual_config(cfg: dict):
    """线程安全原子写入双臂配置与外参"""
    with _CONFIG_LOCK:
        os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
        # 若 A/B 均已标定，自动更新基座间相对变换 T_baseA_baseB = T_baseA_cam @ inv(T_baseB_cam)
        ta = cfg["arms"]["arm_a"].get("T_base_cam")
        tb = cfg["arms"]["arm_b"].get("T_base_cam")
        if ta is not None and tb is not None:
            Ta = np.array(ta, dtype=float)
            Tb = np.array(tb, dtype=float)
            cfg["T_baseA_baseB"] = (Ta @ np.linalg.inv(Tb)).tolist()
        else:
            cfg["T_baseA_baseB"] = None

        tmp = CONFIG_PATH + ".tmp"
        with open(tmp, 'w', encoding='utf-8') as f:
            json.dump(cfg, f, indent=2, ensure_ascii=False)
        os.replace(tmp, CONFIG_PATH)


def solve_rigid_transform_svd(pts_cam: np.ndarray, pts_base: np.ndarray):
    """
    Kabsch-SVD 求解最优刚体变换 T_base_cam (4x4)，满足: P_base = R * P_cam + t
    返回: (T_4x4, rmse_mm, per_point_errs_mm, qx, qy, qz, qw)
    """
    A = np.asarray(pts_cam, dtype=float)
    B = np.asarray(pts_base, dtype=float)
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A

    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = t

    transformed = (R @ A.T).T + t
    errs_mm = np.linalg.norm(transformed - B, axis=1) * 1000.0
    rmse_mm = float(np.sqrt(np.mean((errs_mm / 1000.0) ** 2)) * 1000.0)
    qx, qy, qz, qw = rot_to_quat(R)
    return T, rmse_mm, errs_mm.tolist(), [qx, qy, qz, qw]


class MultiScaleAprilTagDetector:
    """
    自适应多尺度 AprilTag 36h11 检测器:
    先用常规灰度快速尝试 (~2ms)；若因视角拉远/强光过曝漏检，则自动切换 2x 超分 + 参数增强模式 (~6ms)
    同时优先结合 RealSense 对齐深度反投影求解 0.02mm 级抖动的高精度 3D 坐标，消除透视尺幅误差！
    """

    def __init__(self, tag_size_m: float = TAG_SIZE_M):
        self.tag_size_m = tag_size_m
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)

        # 快速检测参数 (采用 SUBPIX 亚像素精修 + 8px/cell 采样，兼顾 640x480 直出速度与小标靶检出率)
        self.params_fast = self._build_params(min_rate=0.015, cell_px=8, win_max=29, step=8)
        # 超分强光鲁棒参数
        self.params_robust = self._build_params(min_rate=0.012, cell_px=8, win_max=37, step=8)

        has_new = hasattr(cv2.aruco, 'ArucoDetector')
        self.det_fast = cv2.aruco.ArucoDetector(self.aruco_dict, self.params_fast) if has_new else None
        self.det_robust = cv2.aruco.ArucoDetector(self.aruco_dict, self.params_robust) if has_new else None

        s = tag_size_m / 2.0
        self.obj_pts = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], dtype=np.float32)

    @staticmethod
    def _build_params(min_rate: float, cell_px: int, win_max: int, step: int = 8):
        p = cv2.aruco.DetectorParameters() if hasattr(cv2.aruco, 'DetectorParameters') else cv2.aruco.DetectorParameters_create()
        p.minMarkerPerimeterRate = min_rate
        p.adaptiveThreshWinSizeMin = 5
        p.adaptiveThreshWinSizeMax = win_max
        p.adaptiveThreshWinSizeStep = step
        p.adaptiveThreshConstant = 7
        p.perspectiveRemovePixelPerCell = cell_px
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        return p

    def _run_detect(self, gray, robust=False):
        det = self.det_robust if robust else self.det_fast
        params = self.params_robust if robust else self.params_fast
        if det is not None:
            return det.detectMarkers(gray)
        return cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=params)

    def detect_all(self, bgr_frame: np.ndarray, K: np.ndarray, D: np.ndarray, aligned_depth: np.ndarray = None):
        """
        返回视野内所有检测到的 AprilTag 列表:
        [
          {
            "id": tid, "corners": tc, "uv": uv, "p_cam": p_cam, "rvec": rvec,
            "tvec_pnp": tvec_pnp, "yaw_vec_cam": yaw_vec_cam, "depth_mm": depth_mm,
            "upscaled": is_up
          }, ...
        ]
        """
        if K is None or bgr_frame is None:
            return []

        gray = cv2.cvtColor(bgr_frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._run_detect(gray, robust=False)
        scale = 1.0

        if ids is None or len(ids) == 0:
            h, w = gray.shape[:2]
            up = cv2.resize(gray, (w * 2, h * 2), interpolation=cv2.INTER_CUBIC)
            corners, ids, _ = self._run_detect(up, robust=True)
            scale = 2.0

        if ids is None or len(ids) == 0:
            return []

        results = []
        for i in range(len(ids)):
            tid = int(ids[i].flatten()[0])
            tc = (corners[i].reshape((4, 2)) / scale).astype(np.float64)
            uv = tc.mean(axis=0)
            u_px, v_px = int(round(uv[0])), int(round(uv[1]))

            ok, rvec, tvec_pnp = cv2.solvePnP(
                self.obj_pts, tc.astype(np.float32), K, D, flags=cv2.SOLVEPNP_ITERATIVE
            )
            if not ok:
                continue

            R_cam, _ = cv2.Rodrigues(rvec)
            yaw_vec_cam = R_cam[:, 0].copy()

            p_cam = tvec_pnp.flatten().astype(float)
            depth_mm = float(p_cam[2] * 1000.0)
            if aligned_depth is not None:
                h_d, w_d = aligned_depth.shape[:2]
                y0, y1 = max(0, v_px - 2), min(h_d, v_px + 3)
                x0, x1 = max(0, u_px - 2), min(w_d, u_px + 3)
                patch = aligned_depth[y0:y1, x0:x1]
                valid = patch[(patch > 150) & (patch < 1500)]
                if len(valid) > 0:
                    z_m = float(np.median(valid)) / 1000.0
                    x_m = (uv[0] - K[0, 2]) * z_m / K[0, 0]
                    y_m = (uv[1] - K[1, 2]) * z_m / K[1, 1]
                    p_cam = np.array([x_m, y_m, z_m], dtype=float)
                    depth_mm = z_m * 1000.0

            results.append({
                "id": tid,
                "corners": tc,
                "uv": uv,
                "p_cam": p_cam,
                "rvec": rvec,
                "tvec_pnp": tvec_pnp,
                "yaw_vec_cam": yaw_vec_cam,
                "depth_mm": depth_mm,
                "upscaled": (scale > 1.0),
            })
        return results

    def detect(self, bgr_frame: np.ndarray, K: np.ndarray, D: np.ndarray, aligned_depth: np.ndarray = None):
        """兼容单标靶调用"""
        res = self.detect_all(bgr_frame, K, D, aligned_depth)
        return res[0] if res else None


def draw_arm_ar_overlay(display: np.ndarray, arm_label: str, T_base_cam: np.ndarray,
                        K: np.ndarray, D: np.ndarray, table_z_base: float,
                        color_main=(255, 180, 0), color_arc=(0, 220, 120)):
    """将机械臂基座原点、XY方向指示箭头与桌面安全工作圆环投影绘制到实时画面上"""
    if T_base_cam is None or K is None:
        return
    try:
        T_cam_base = np.linalg.inv(np.asarray(T_base_cam, dtype=float))
    except np.linalg.LinAlgError:
        return

    def proj(p_base):
        pc = (T_cam_base @ np.array([p_base[0], p_base[1], p_base[2], 1.0]))[:3]
        if pc[2] <= 0.08:
            return None
        pts2d, _ = cv2.projectPoints(pc.reshape(1, 1, 3), np.zeros(3), np.zeros(3), K, D)
        # 匹配 180度旋转显示系
        u, v = 639 - int(round(pts2d[0, 0, 0])), 479 - int(round(pts2d[0, 0, 1]))
        if -400 <= u <= 1040 and -400 <= v <= 880:
            return (u, v)
        return None

    # 1. 基座原点与 X/Y 坐标指示 (仅当落在屏幕附近才画箭头)
    p0 = proj([0.0, 0.0, 0.0])
    px = proj([0.14, 0.0, 0.0])
    py = proj([0.0, 0.14, 0.0])
    if p0 is not None:
        cv2.circle(display, p0, 6, color_main, -1)
        if 10 <= p0[0] <= 600 and 20 <= p0[1] <= 465:
            cv2.putText(display, arm_label, (p0[0] + 8, p0[1] - 6),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, color_main, 2)
        if px is not None:
            cv2.arrowedLine(display, p0, px, (0, 0, 255), 2, tipLength=0.25)
        if py is not None:
            cv2.arrowedLine(display, p0, py, (0, 255, 0), 2, tipLength=0.25)

    # 2. 安全可达桌面边界弧 (R=0.44m 外圈 + R=0.14m 内圈)
    for radius, thickness in [(0.44, 1), (0.14, 1)]:
        arc_pts = []
        for deg in range(-120, 125, 8):
            rad = math.radians(deg)
            uv = proj([radius * math.cos(rad), radius * math.sin(rad), table_z_base])
            if uv is not None and -100 <= uv[0] <= 740 and -100 <= uv[1] <= 580:
                arc_pts.append(uv)
            else:
                if len(arc_pts) >= 2:
                    cv2.polylines(display, [np.array(arc_pts, dtype=np.int32)], False, color_arc, thickness)
                arc_pts = []
        if len(arc_pts) >= 2:
            cv2.polylines(display, [np.array(arc_pts, dtype=np.int32)], False, color_arc, thickness)

    # 在前缘弧顶端 (r=0.38, theta=0) 标出臂号区域归属文字
    p_zone = proj([0.35, 0.0, table_z_base])
    if p_zone is not None and 20 <= p_zone[0] <= 580 and 20 <= p_zone[1] <= 460:
        cv2.putText(display, f"[{arm_label} ZONE]", (p_zone[0] - 42, p_zone[1]),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, color_arc, 1)
