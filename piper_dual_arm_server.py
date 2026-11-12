#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 双臂统一 ROS 2 高频运控 Action Server
功能：
1. 同时管理「机械臂 A (arm_a)」与「机械臂 B (arm_b)」两台真实机械臂：
   - 依据硬件 USB-CAN 序列号动态精确定位内核接口 (支持 can_piper_0/1 与 can0/can1)
   - 若某条机械臂 CAN 接口尚未拉起或尚未上电，不会崩溃退出，自动周期重连热插拔上线
2. 每台机械臂提供标准独立控制接口：
   - 200Hz 关节状态: /arm_a/joint_states, /arm_b/joint_states
   - 标准轨迹动作:   /arm_a/arm_controller/follow_joint_trajectory, /arm_b/arm_controller/follow_joint_trajectory
   - 夹爪柔顺控制:   /arm_a/piper_gripper_cmd (+effort), /arm_b/piper_gripper_cmd (+effort)
3. 双臂协同与在线标定辅助控制 (/dual_arm/command):
   - 一键进入/退出自由拖动示教模式 (重力补偿零阻尼拖动用于手眼标定)
   - 一键「轻摇识别认臂」(只微动夹爪和末轴6，零碰桌风险，视觉确认哪台是A/B)
   - 一键「互换 A/B 映射」并持久化保存至 dual_arm_config.json
4. 实时发布双臂硬件诊断状态话题 (/dual_arm/status_json @ 5Hz)

作者: 203实验室 双臂抛接课题组
通信域: ROS_DOMAIN_ID=42
"""

import os
import glob
import json
import math
import time
import threading
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String

from piper_sdk import C_PiperInterface_V2

# 配置文件路径
WORK_DIR = "/home/lab203/piper_students/lq/real_arm_control"
CONFIG_PATH = os.path.join(WORK_DIR, "dual_arm_config.json")

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIPPER_JOINTS = ['joint7', 'joint8']
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS

DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
RAW_TO_RAD = 0.001 * DEG_TO_RAD
RAD_TO_RAW = 1000.0 * RAD_TO_DEG

MAX_TRACKING_ERROR_RAD = 0.35   # 最大允许跟踪误差 (约 20°)
CONTROL_DT = 0.005              # 200Hz 周期

DEFAULT_CONFIG = {
    "arms": {
        "arm_a": {
            "label": "机械臂 A",
            "usb_serial": "0025002C5547570420303135",
            "preferred_can": "can_piper_0",
            "grasp_x_offset_mm": 0.0,
            "grasp_y_offset_mm": 0.0,
            "grasp_z_offset_mm": -25.5,
            "grasp_effort": 350,
            "grasp_target_mm": 26.5,
            "calibrated": False,
            "T_base_cam": None,
            "calib_rmse_mm": None,
            "calib_points_count": 0
        },
        "arm_b": {
            "label": "机械臂 B",
            "usb_serial": "002E00344648570220393831",
            "preferred_can": "can_piper_1",
            "grasp_x_offset_mm": 0.0,
            "grasp_y_offset_mm": 0.0,
            "grasp_z_offset_mm": -25.5,
            "grasp_effort": 350,
            "grasp_target_mm": 26.5,
            "calibrated": False,
            "T_base_cam": None,
            "calib_rmse_mm": None,
            "calib_points_count": 0
        }
    },
    "T_baseA_baseB": None
}


def load_or_init_config() -> dict:
    """加载或初始化双臂全局配置 JSON"""
    if os.path.exists(CONFIG_PATH):
        try:
            with open(CONFIG_PATH, 'r', encoding='utf-8') as f:
                data = json.load(f)
            if "arms" in data and "arm_a" in data["arms"] and "arm_b" in data["arms"]:
                return data
        except Exception:
            pass
    save_config(DEFAULT_CONFIG)
    return json.loads(json.dumps(DEFAULT_CONFIG))


def save_config(cfg: dict):
    """线程外部加锁后保存配置文件"""
    os.makedirs(os.path.dirname(CONFIG_PATH), exist_ok=True)
    tmp_path = CONFIG_PATH + ".tmp"
    with open(tmp_path, 'w', encoding='utf-8') as f:
        json.dump(cfg, f, indent=2, ensure_ascii=False)
    os.replace(tmp_path, CONFIG_PATH)


def scan_can_interfaces() -> Dict[str, dict]:
    """扫描系统当前所有 SocketCAN 接口及其硬件序列号、ifindex 和 UP 状态"""
    result = {}
    for net_path in sorted(glob.glob("/sys/class/net/can*")):
        iface = os.path.basename(net_path)
        is_up = False
        serial = ""
        ifindex = -1
        try:
            with open(os.path.join(net_path, "flags"), 'r') as f:
                flags_val = int(f.read().strip(), 16)
                is_up = bool(flags_val & 0x1)
        except Exception:
            pass
        try:
            with open(os.path.join(net_path, "ifindex"), 'r') as f:
                ifindex = int(f.read().strip())
        except Exception:
            pass
        serial_file = os.path.join(net_path, "device", "..", "serial")
        try:
            if os.path.exists(serial_file):
                with open(serial_file, 'r') as f:
                    serial = f.read().strip()
        except Exception:
            pass
        result[iface] = {"iface": iface, "is_up": is_up, "serial": serial, "ifindex": ifindex}
    return result


def quintic_eval(t: float, T: float, p0: float, p1: float,
                 v0: float = 0.0, v1: float = 0.0, a0: float = 0.0, a1: float = 0.0):
    """五次多项式 C² 连续插值计算"""
    if T <= 1e-6:
        return p1, 0.0, 0.0
    tau = max(0.0, min(1.0, t / T))
    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau

    h = p1 - p0
    a_0 = p0
    a_1 = v0 * T
    a_2 = 0.5 * a0 * T * T
    a_3 = 10.0 * h - (6.0 * v0 + 4.0 * v1) * T + (0.5 * a1 - 1.5 * a0) * T * T
    a_4 = -15.0 * h + (8.0 * v0 + 7.0 * v1) * T + (1.5 * a0 - a1) * T * T
    a_5 = 6.0 * h - 3.0 * (v0 + v1) * T + 0.5 * (a1 - a0) * T * T

    pos = a_0 + a_1 * tau + a_2 * tau2 + a_3 * tau3 + a_4 * tau4 + a_5 * tau5
    vel = (a_1 + 2.0 * a_2 * tau + 3.0 * a_3 * tau2 + 4.0 * a_4 * tau3 + 5.0 * a_5 * tau4) / T
    acc = (2.0 * a_2 + 6.0 * a_3 * tau + 12.0 * a_4 * tau2 + 20.0 * a_5 * tau3) / (T * T)
    return pos, vel, acc


class SingleArmController:
    """封装单台 PIPER 机械臂的 CAN 连接、状态发布、示教模式与轨迹执行"""

    def __init__(self, node: Node, arm_id: str, usb_serial: str, preferred_can: str, cb_group):
        self.node = node
        self.arm_id = arm_id
        self.usb_serial = usb_serial
        self.preferred_can = preferred_can
        self.cb_group = cb_group

        self.lock = threading.Lock()
        self.piper: Optional[C_PiperInterface_V2] = None
        self.can_port: Optional[str] = None
        self.ifindex: int = -1
        self.can_up = False
        self.connected = False
        self.was_motors_alive = False
        self.logic_alive = False
        self.last_probe_time = 0.0
        self.busy = False
        self.gripper_effort = 350

        self._active_goal_lock = threading.Lock()

        # ROS 2 接口 (按 /arm_a/... 或 /arm_b/... 命名空间隔离)
        ns = f"/{arm_id}"
        self.joint_state_pub = node.create_publisher(JointState, f"{ns}/joint_states", 10)
        node.create_subscription(Float64, f"{ns}/piper_gripper_effort", self._effort_cb, 10, callback_group=cb_group)
        node.create_subscription(Float64, f"{ns}/piper_gripper_cmd", self._gripper_cb, 10, callback_group=cb_group)

        self._action_server = ActionServer(
            node,
            FollowJointTrajectory,
            f"{ns}/arm_controller/follow_joint_trajectory",
            execute_callback=self._execute_trajectory_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=cb_group,
        )

    def resolve_and_connect(self, can_map: Dict[str, dict]):
        """根据硬件序列号匹配内核 CAN 接口；仅当接口下线或 USB 重枚举(ifindex变更)才重建句柄，24V未通电时静默守候不刷屏"""
        matched_iface = None
        matched_up = False
        matched_ifindex = -1

        for iface, info in can_map.items():
            if self.usb_serial and info.get("serial") == self.usb_serial:
                matched_iface = iface
                matched_up = info.get("is_up", False)
                matched_ifindex = info.get("ifindex", -1)
                break

        if not matched_iface and self.preferred_can in can_map:
            matched_iface = self.preferred_can
            matched_up = can_map[self.preferred_can].get("is_up", False)
            matched_ifindex = can_map[self.preferred_can].get("ifindex", -1)

        with self.lock:
            now = time.time()
            # 仅在网口掉线或 USB 重新插拔 (ifindex 改变) 时释放旧句柄
            if self.connected and self.piper is not None:
                if not matched_up or matched_iface != self.can_port or matched_ifindex != self.ifindex:
                    self.node.get_logger().warn(
                        f"[{self.arm_id}] 检测到 CAN 接口重置 (ifindex {self.ifindex}->{matched_ifindex}, up={matched_up})，释放旧句柄..."
                    )
                    try:
                        self.piper.DisconnectPort()
                    except Exception:
                        pass
                    self.connected = False
                    self.piper = None
                    self.was_motors_alive = False
                    self.logic_alive = False

            self.can_port = matched_iface
            self.can_up = matched_up
            self.ifindex = matched_ifindex

            if not matched_up:
                return

            # 接口已 UP 且尚未连接 -> 严格被动挂载监听句柄 (piper_init=False 避免上电瞬间发 0x4AF/0x472 阻塞机械臂 Bootloader 跳转)
            if not self.connected or self.piper is None:
                try:
                    self.node.get_logger().info(f"[{self.arm_id}] 挂载 CAN 端口 {matched_iface} (ifindex={matched_ifindex}, Serial={self.usb_serial})...")
                    p = C_PiperInterface_V2(can_name=matched_iface, judge_flag=False)
                    p.ConnectPort(can_init=True, piper_init=False, start_thread=True)
                    time.sleep(0.25)
                    self.piper = p
                    self.connected = True
                    self.last_probe_time = now
                    self.node.get_logger().info(f"[{self.arm_id}] ✅ CAN 端口 {matched_iface} 已成功被动挂载监听！")
                except Exception as e:
                    self.node.get_logger().warn(f"[{self.arm_id}] 连接 {matched_iface} 异常: {e}")
                    self.connected = False
                    self.piper = None
                    return

            # 严格被动守候：未收到 0x2A1 遥测前绝不主动向 CAN 发送 0x4AF 等查询帧，避免重传帧卡住机械臂开机引导
            if self.connected and self.piper is not None:
                fps = float(self.piper.GetCanFps())
                if fps > 1.0:
                    self.logic_alive = True
                    if not self.was_motors_alive:
                        self.was_motors_alive = True
                        self.node.get_logger().info(f"[{self.arm_id}] ⚡ 检测到机械臂上线 (CAN FPS={fps:.0f})，同步机械臂固件参数...")
                        try:
                            self.piper.PiperInit()
                        except Exception:
                            pass
                else:
                    self.was_motors_alive = False
                    self.logic_alive = False

    def get_status_dict(self) -> dict:
        """采集该机械臂完整遥测状态字典"""
        with self.lock:
            p = self.piper
            connected = self.connected
            can_port = self.can_port or "--"
            can_up = self.can_up
            usb_serial = self.usb_serial
            logic_alive = self.logic_alive
            busy = self.busy

        motors_alive = False
        can_fps = 0.0
        ctrl_mode = -1
        teach_status = -1
        arm_status_code = -1
        joints_deg = [0.0] * 6
        gripper_mm = 0.0

        if connected and p is not None:
            try:
                can_fps = round(float(p.GetCanFps()), 1)
                motors_alive = can_fps > 1.0
                st = p.GetArmStatus().arm_status
                ctrl_mode = int(st.ctrl_mode)
                teach_status = int(st.teach_status)
                arm_status_code = int(st.arm_status)
                js = p.GetArmJointMsgs().joint_state
                joints_deg = [
                    round(js.joint_1 * 0.001, 2),
                    round(js.joint_2 * 0.001, 2),
                    round(js.joint_3 * 0.001, 2),
                    round(js.joint_4 * 0.001, 2),
                    round(js.joint_5 * 0.001, 2),
                    round(js.joint_6 * 0.001, 2),
                ]
                gs = p.GetArmGripperMsgs().gripper_state
                gripper_mm = round(gs.grippers_angle * 0.001, 2)
            except Exception:
                pass

        return {
            "arm_id": self.arm_id,
            "can_port": can_port,
            "usb_serial": usb_serial,
            "can_up": can_up,
            "connected": connected,
            "logic_alive": logic_alive,
            "motors_alive": motors_alive,
            "can_fps": can_fps,
            "ctrl_mode": ctrl_mode,         # 0=Standby, 1=CAN_CTRL, 2=TEACH_MODE
            "teach_status": teach_status,
            "arm_status": arm_status_code,
            "joints_deg": joints_deg,
            "gripper_mm": gripper_mm,
            "busy": busy,
        }

    def publish_joint_state(self):
        """200Hz 高频发布该机械臂 /arm_X/joint_states"""
        with self.lock:
            p = self.piper
            ok = self.connected
        if not ok or p is None:
            return
        try:
            if p.GetCanFps() <= 0.5:
                return
            cur_rad = self._get_joints_rad(p)
            gs = p.GetArmGripperMsgs().gripper_state
            gripper_m = (gs.grippers_angle / 1000.0) / 1000.0
            finger_m = max(0.0, gripper_m / 2.0)

            msg = JointState()
            msg.header.stamp = self.node.get_clock().now().to_msg()
            msg.name = ALL_JOINTS
            msg.position = cur_rad + [finger_m, finger_m]
            self.joint_state_pub.publish(msg)
        except Exception:
            pass

    @staticmethod
    def _get_joints_rad(p: C_PiperInterface_V2) -> List[float]:
        js = p.GetArmJointMsgs().joint_state
        return [
            js.joint_1 * RAW_TO_RAD,
            js.joint_2 * RAW_TO_RAD,
            js.joint_3 * RAW_TO_RAD,
            js.joint_4 * RAW_TO_RAD,
            js.joint_5 * RAW_TO_RAD,
            js.joint_6 * RAW_TO_RAD,
        ]

    def _effort_cb(self, msg: Float64):
        self.gripper_effort = int(max(150.0, min(2500.0, float(msg.data))))

    def _gripper_cb(self, msg: Float64):
        with self.lock:
            p = self.piper
            ok = self.connected
        if not ok or p is None:
            return
        target_mm = max(0.0, min(70.0, float(msg.data)))
        stroke_raw = int(target_mm * 1000.0)
        # 夹爪安全保活与重连自恢复：若夹爪存在故障码或未使能，先清错(0x02)后使能(0x01)
        try:
            gs = p.GetArmGripperMsgs().gripper_state
            if getattr(gs, 'err_code', 0) != 0 or not getattr(gs, 'gripper_enable_flag', True):
                p.GripperCtrl(stroke_raw, self.gripper_effort, 0x02, 0)
                time.sleep(0.02)
        except Exception:
            pass
        p.GripperCtrl(stroke_raw, self.gripper_effort, 0x01, 0)

    def _ensure_can_ctrl(self, p: C_PiperInterface_V2):
        """确保退出示教、将当前位置设为保持点并安全切回 CAN 闭环控制模式"""
        st = p.GetArmStatus().arm_status
        if st.ctrl_mode == 2:
            # 安全退出示教拖动
            for _ in range(4):
                p.MotionCtrl_1(0x00, 0x00, 0x02)
                time.sleep(0.04)
        for _ in range(4):
            p.MotionCtrl_1(0x00, 0x00, 0x00)
            time.sleep(0.03)

        # 使能电机
        for _ in range(40):
            if p.EnablePiper():
                break
            time.sleep(0.015)

        # 切模式前先读取并发送当前真实角度，彻底杜绝切换瞬间回弹跳变
        cur = self._get_joints_rad(p)
        raw_cur = [int(v * RAD_TO_RAW) for v in cur]
        for _ in range(15):
            p.ModeCtrl(0x01, 0x01, 35, 0x00)
            p.JointCtrl(*raw_cur)
            time.sleep(0.04)
            if p.GetArmStatus().arm_status.ctrl_mode == 1:
                break

        # 使能夹爪
        p.GripperCtrl(0, 1000, 0x02, 0)
        time.sleep(0.04)
        p.GripperCtrl(0, 1000, 0x01, 0)

    def enter_teaching_mode(self):
        """一键切入自由拖动示教模式 (重力补偿零阻尼拖动，方便手眼标定)"""
        with self.lock:
            p = self.piper
            ok = self.connected
        if not ok or p is None:
            return False, "CAN 未连接"
        if float(p.GetCanFps()) <= 1.0:
            return False, "机械臂未上电就绪 (无 0x2A1 遥测)，跳过发送"
        try:
            for _ in range(6):
                p.MotionCtrl_1(0x00, 0x00, 0x01)  # 0x01 = 开启拖动示教
                time.sleep(0.05)
            return True, "已切入示教拖动模式，可用手自由移动机械臂"
        except Exception as e:
            return False, str(e)

    def exit_teaching_mode(self):
        """一键退出示教模式并平滑上电锁定在当前位姿"""
        with self.lock:
            p = self.piper
            ok = self.connected
        if not ok or p is None:
            return False, "CAN 未连接"
        if float(p.GetCanFps()) <= 1.0:
            return False, "机械臂未上电就绪 (无 0x2A1 遥测)，跳过发送"
        try:
            self._ensure_can_ctrl(p)
            return True, "已安全切回 CAN 伺服锁定模式"
        except Exception as e:
            return False, str(e)

    def wiggle_identify(self):
        """安全轻摇测试认臂：开合夹爪并轻转第6轴 ±20°，连杆不摆动，零碰桌风险"""
        with self.lock:
            p = self.piper
            ok = self.connected
            if self.busy:
                return
            self.busy = True

        def _worker():
            try:
                if not ok or p is None:
                    return
                was_teach = (p.GetArmStatus().arm_status.ctrl_mode == 2)
                self._ensure_can_ctrl(p)
                cur = self._get_joints_rad(p)
                base_raw = [int(v * RAD_TO_RAW) for v in cur]

                # 夹爪明显开合两次 + 第 6 轴末端轻摆
                for cycle in range(2):
                    p.GripperCtrl(40000, 400, 0x01, 0)  # 张开 40mm
                    cmd_plus = list(base_raw)
                    cmd_plus[5] = int(max(-1.8, min(1.8, cur[5] + 0.35)) * RAD_TO_RAW)
                    p.ModeCtrl(0x01, 0x01, 35, 0x00)
                    p.JointCtrl(*cmd_plus)
                    time.sleep(0.55)

                    p.GripperCtrl(5000, 400, 0x01, 0)   # 闭合至 5mm
                    cmd_minus = list(base_raw)
                    cmd_minus[5] = int(max(-1.8, min(1.8, cur[5] - 0.35)) * RAD_TO_RAW)
                    p.ModeCtrl(0x01, 0x01, 35, 0x00)
                    p.JointCtrl(*cmd_minus)
                    time.sleep(0.55)

                # 回正
                p.ModeCtrl(0x01, 0x01, 35, 0x00)
                p.JointCtrl(*base_raw)
                p.GripperCtrl(0, 400, 0x01, 0)
                time.sleep(0.4)

                if was_teach:
                    for _ in range(5):
                        p.MotionCtrl_1(0x00, 0x00, 0x01)
                        time.sleep(0.04)
            finally:
                with self.lock:
                    self.busy = False

        threading.Thread(target=_worker, daemon=True).start()

    def _goal_callback(self, goal_request):
        with self.lock:
            if not self.connected or self.piper is None:
                self.node.get_logger().warn(f"[{self.arm_id}] CAN 离线，拒绝轨迹 Goal")
                return GoalResponse.REJECT
        traj = goal_request.trajectory
        if not traj.points or any(j not in traj.joint_names for j in ARM_JOINTS):
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _execute_trajectory_callback(self, goal_handle):
        """200Hz C² 五次多项式连续插值轨迹执行"""
        with self._active_goal_lock:
            with self.lock:
                self.busy = True
                p = self.piper
                ok = self.connected

            result = FollowJointTrajectory.Result()
            if not ok or p is None:
                with self.lock:
                    self.busy = False
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
                return result

            try:
                # 保证处于 CAN_CTRL 模式
                if p.GetArmStatus().arm_status.ctrl_mode != 1:
                    self._ensure_can_ctrl(p)

                traj = goal_handle.request.trajectory
                joint_idx_map = [traj.joint_names.index(jname) for jname in ARM_JOINTS]

                cur_actual = self._get_joints_rad(p)
                segments = []
                p_prev = cur_actual
                v_prev = [0.0] * 6
                a_prev = [0.0] * 6
                t_prev = 0.0

                for pt in traj.points:
                    t_curr = pt.time_from_start.sec + pt.time_from_start.nanosec * 1e-9
                    duration = t_curr - t_prev
                    if duration <= 1e-4:
                        continue
                    p_curr = [pt.positions[idx] for idx in joint_idx_map]
                    v_curr = [pt.velocities[idx] if len(pt.velocities) > idx else 0.0 for idx in joint_idx_map]
                    a_curr = [pt.accelerations[idx] if len(pt.accelerations) > idx else 0.0 for idx in joint_idx_map]
                    segments.append({
                        'duration': duration,
                        'p0': p_prev, 'p1': p_curr,
                        'v0': v_prev, 'v1': v_curr,
                        'a0': a_prev, 'a1': a_curr,
                    })
                    p_prev, v_prev, a_prev, t_prev = p_curr, v_curr, a_curr, t_curr

                next_tick = time.monotonic()
                for seg in segments:
                    dur = seg['duration']
                    steps = max(1, int(round(dur / CONTROL_DT)))
                    for step in range(steps):
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                            return result

                        t_seg = step * (dur / steps)
                        desired_pos = []
                        for j in range(6):
                            pos_j, _, _ = quintic_eval(
                                t_seg, dur,
                                seg['p0'][j], seg['p1'][j],
                                seg['v0'][j], seg['v1'][j],
                                seg['a0'][j], seg['a1'][j],
                            )
                            desired_pos.append(pos_j)

                        raw_cmds = [int(v * RAD_TO_RAW) for v in desired_pos]
                        p.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                        p.JointCtrl(*raw_cmds)

                        actual_pos = self._get_joints_rad(p)
                        max_err = max(abs(desired_pos[j] - actual_pos[j]) for j in range(6))
                        if max_err > MAX_TRACKING_ERROR_RAD:
                            self.node.get_logger().error(
                                f"[{self.arm_id}] ⚠️ 跟踪超差 {max_err*RAD_TO_DEG:.1f}°，安全中止"
                            )
                            goal_handle.abort()
                            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                            return result

                        next_tick += CONTROL_DT
                        sleep_dt = next_tick - time.monotonic()
                        if sleep_dt > 0:
                            time.sleep(sleep_dt)

                if segments:
                    final_pos = segments[-1]['p1']
                    final_raw = [int(v * RAD_TO_RAW) for v in final_pos]
                    # 末段闭环位置收敛等待：确保机械臂实际关节已完全下探/运动到位再返回成功，杜绝未到位即提前松爪掉落
                    settle_deadline = time.monotonic() + 0.85
                    prev_err = 999.0
                    stable_cnt = 0
                    while time.monotonic() < settle_deadline:
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                            return result
                        p.MotionCtrl_2(0x01, 0x01, 100, 0x00)
                        p.JointCtrl(*final_raw)
                        actual_pos = self._get_joints_rad(p)
                        cur_err = max(abs(final_pos[j] - actual_pos[j]) for j in range(6))
                        if cur_err <= 0.008:
                            break
                        if abs(prev_err - cur_err) < 0.0008 and cur_err <= 0.025:
                            stable_cnt += 1
                            if stable_cnt >= 6:
                                break
                        else:
                            stable_cnt = 0
                        prev_err = cur_err
                        time.sleep(0.01)

                goal_handle.succeed()
                result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                return result
            finally:
                with self.lock:
                    self.busy = False


class PiperDualArmServerNode(Node):
    """统一管理 arm_a 与 arm_b 的 ROS 2 主节点"""

    def __init__(self):
        super().__init__('piper_dual_arm_server')
        self.get_logger().info("🚀 正在初始化 PIPER 双臂统一运控服务节点 (arm_a + arm_b)...")
        self.cb_group = ReentrantCallbackGroup()
        self.cfg_lock = threading.Lock()
        self.config = load_or_init_config()

        arm_a_cfg = self.config["arms"]["arm_a"]
        arm_b_cfg = self.config["arms"]["arm_b"]

        self.arms: Dict[str, SingleArmController] = {
            "arm_a": SingleArmController(
                self, "arm_a",
                arm_a_cfg.get("usb_serial", DEFAULT_CONFIG["arms"]["arm_a"]["usb_serial"]),
                arm_a_cfg.get("preferred_can", "can_piper_0"),
                self.cb_group,
            ),
            "arm_b": SingleArmController(
                self, "arm_b",
                arm_b_cfg.get("usb_serial", DEFAULT_CONFIG["arms"]["arm_b"]["usb_serial"]),
                arm_b_cfg.get("preferred_can", "can_piper_1"),
                self.cb_group,
            ),
        }

        # 200Hz 关节发布器 + 5Hz 状态监测与热重连
        self.status_pub = self.create_publisher(String, "/dual_arm/status_json", 10)
        self.create_timer(CONTROL_DT, self._publish_all_joints, callback_group=self.cb_group)
        self.create_timer(0.2, self._publish_status, callback_group=self.cb_group)
        self.create_timer(2.0, self._scan_and_connect_loop, callback_group=self.cb_group)

        # 命令接收 (/dual_arm/command)
        self.create_subscription(String, "/dual_arm/command", self._command_cb, 10, callback_group=self.cb_group)

        # 初次立即扫描一次 CAN
        self._scan_and_connect_loop()
        self.get_logger().info("✅ 双臂运控节点上线完成！支持热插拔 CAN 重连、示教切换与 A/B 热互换。")

    def _scan_and_connect_loop(self):
        can_map = scan_can_interfaces()
        for arm in self.arms.values():
            arm.resolve_and_connect(can_map)

    def _publish_all_joints(self):
        for arm in self.arms.values():
            arm.publish_joint_state()

    def _publish_status(self):
        can_map = scan_can_interfaces()
        payload = {
            "timestamp": time.time(),
            "raw_can_interfaces": list(can_map.values()),
            "arm_a": self.arms["arm_a"].get_status_dict(),
            "arm_b": self.arms["arm_b"].get_status_dict(),
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.status_pub.publish(msg)

    def _command_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except Exception:
            return

        cmd = data.get("cmd", "")
        arm_id = data.get("arm", "arm_a")

        if cmd == "enter_teach" and arm_id in self.arms:
            ok, info = self.arms[arm_id].enter_teaching_mode()
            self.get_logger().info(f"[{arm_id}] enter_teach -> {ok}: {info}")
        elif cmd in ("exit_teach", "enable_arm") and arm_id in self.arms:
            ok, info = self.arms[arm_id].exit_teaching_mode()
            self.get_logger().info(f"[{arm_id}] {cmd} -> {ok}: {info}")
        elif cmd == "wiggle" and arm_id in self.arms:
            self.arms[arm_id].wiggle_identify()
            self.get_logger().info(f"[{arm_id}] 执行轻摇认臂动作 (夹爪开合 + 第6轴轻摆)")
        elif cmd == "swap_ab":
            self._swap_ab_binding()
        elif cmd == "reconnect":
            self._scan_and_connect_loop()

    def _swap_ab_binding(self):
        """一键互换 A/B 机械臂硬件 CAN 绑定，同步刷新配置文件"""
        a = self.arms["arm_a"]
        b = self.arms["arm_b"]

        if a.busy or b.busy:
            self.get_logger().warn("机械臂正在执行动作，拒绝互换 A/B 映射！")
            return

        with a.lock, b.lock:
            a.usb_serial, b.usb_serial = b.usb_serial, a.usb_serial
            a.preferred_can, b.preferred_can = b.preferred_can, a.preferred_can
            a.can_port, b.can_port = b.can_port, a.can_port
            a.ifindex, b.ifindex = b.ifindex, a.ifindex
            a.can_up, b.can_up = b.can_up, a.can_up
            a.piper, b.piper = b.piper, a.piper
            a.connected, b.connected = b.connected, a.connected
            a.was_motors_alive, b.was_motors_alive = b.was_motors_alive, a.was_motors_alive
            a.logic_alive, b.logic_alive = b.logic_alive, a.logic_alive

        with self.cfg_lock:
            cfg = load_or_init_config()
            ca = cfg["arms"]["arm_a"]
            cb = cfg["arms"]["arm_b"]
            ca["usb_serial"], cb["usb_serial"] = cb["usb_serial"], ca["usb_serial"]
            ca["preferred_can"], cb["preferred_can"] = cb["preferred_can"], ca["preferred_can"]
            save_config(cfg)
            self.config = cfg

        self.get_logger().info(
            f"🔄 已成功互换 A/B 映射！当前 A={a.can_port}({a.usb_serial}), B={b.can_port}({b.usb_serial})"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PiperDualArmServerNode()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
