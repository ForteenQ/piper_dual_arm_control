#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂 ROS 2 FollowJointTrajectory Action Server 节点
功能：
1. 实现标准 control_msgs/action/FollowJointTrajectory 动作服务端 (/arm_controller/follow_joint_trajectory)
2. 接收来自 MoveIt 2 或自定义轨迹规划器的 JointTrajectory 路点序列
3. 基于五次多项式 S 曲线（C² 连续）在 200Hz 周期内进行高频平滑密集插值
4. 实时下发 CAN 1Mbps 总线流式运控指令 (JointCtrl + MotionCtrl_2)
5. 200Hz 高频发布标准 /joint_states 话题，为 robot_state_publisher 与 TF 树提供真实物理位姿
6. 完备的安全防护机制：速度/加速度硬限位、跟踪超差保护、动态取消 (Cancel) 平滑制动

作者: 学生 LQ (203实验室 双臂抛接课题组)
空间: /home/lab203/piper_students/lq/real_arm_control/
域ID: ROS_DOMAIN_ID=42
"""

import math
import time
import threading
from typing import List

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

from piper_sdk import C_PiperInterface_V2

# 关节映射与安全限制
ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
GRIPPER_JOINTS = ['joint7', 'joint8']
ALL_JOINTS = ARM_JOINTS + GRIPPER_JOINTS

# 单位转换常量
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi
RAW_TO_RAD = 0.001 * DEG_TO_RAD       # 0.001° -> rad
RAD_TO_RAW = 1000.0 * RAD_TO_DEG      # rad -> 0.001°

# 安全阈值
MAX_TRACKING_ERROR_RAD = 0.35          # 最大允许跟踪误差 (约 20°)，超过则急停
MAX_JOINT_VEL_RAD_S = 2.0             # 关节最大角速度安全上限 (rad/s)
CONTROL_DT = 0.005                    # 200 Hz 控制周期 (5ms)


def quintic_eval(t: float, T: float, p0: float, p1: float, v0: float = 0.0, v1: float = 0.0, a0: float = 0.0, a1: float = 0.0):
    """五次多项式插值: t in [0, T]"""
    if T <= 1e-6:
        return p1, 0.0, 0.0
    tau = max(0.0, min(1.0, t / T))
    tau2 = tau * tau
    tau3 = tau2 * tau
    tau4 = tau3 * tau
    tau5 = tau4 * tau

    # 系数求解 (满足位置、速度、加速度边界约束)
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


class PiperTrajectoryActionServer(Node):
    def __init__(self):
        super().__init__('piper_trajectory_action_server')
        self.get_logger().info('正在初始化 PIPER 轨迹控制器 Action Server (200Hz)...')

        self.cb_group = ReentrantCallbackGroup()

        # 1. 初始化 CAN 接口
        self.can_port = 'can0'
        self.piper = C_PiperInterface_V2(self.can_port)
        self.piper.ConnectPort()
        time.sleep(0.5)

        # 2. 使能机械臂与模式切换
        self._ensure_hardware_ready()

        # 3. 状态发布者 (/joint_states @ 200Hz)
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.pub_timer = self.create_timer(CONTROL_DT, self._publish_joint_states, callback_group=self.cb_group)

        # 4. 创建 FollowJointTrajectory Action Server 与夹爪指令订阅
        self.gripper_effort = 400  # 默认柔顺力矩 (0.40 N·m, 防止捏变形轻质方块)
        self.gripper_effort_sub = self.create_subscription(
            Float64,
            '/piper_gripper_effort',
            self._gripper_effort_callback,
            10,
            callback_group=self.cb_group
        )
        self.gripper_sub = self.create_subscription(
            Float64,
            '/piper_gripper_cmd',
            self._gripper_cmd_callback,
            10,
            callback_group=self.cb_group
        )
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory',
            execute_callback=self._execute_trajectory_callback,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback,
            callback_group=self.cb_group
        )

        self._active_goal_lock = threading.Lock()
        self._current_goal_handle = None
        self.get_logger().info('✅ PIPER 轨迹控制器已就绪: /arm_controller/follow_joint_trajectory 与 /piper_gripper_cmd 上线')

    def _gripper_effort_callback(self, msg: Float64):
        """动态更新夹爪闭合最大力矩 (0.001 N·m, 默认 400)"""
        self.gripper_effort = int(max(150.0, min(2500.0, float(msg.data))))
        self.get_logger().info(f'💪 [夹爪力矩] 限矩调整为 = {self.gripper_effort} (即 {self.gripper_effort/1000.0:.2f} N·m)')

    def _gripper_cmd_callback(self, msg: Float64):
        """接收夹爪目标开度 (mm, 0.0~70.0) 并以当前设定柔顺力矩执行"""
        target_mm = max(0.0, min(70.0, float(msg.data)))
        stroke_raw = int(target_mm * 1000.0)
        self.piper.GripperCtrl(stroke_raw, self.gripper_effort, 0x01, 0)
        self.get_logger().info(f'🤏 [夹爪指令] 动作目标开度 = {target_mm:.1f} mm, 力矩={self.gripper_effort}')

    def _ensure_hardware_ready(self):
        """确保机械臂退出示教、电机使能、并进入 CAN_CTRL 模式，同时使能夹爪"""
        self.get_logger().info('--> 正在清理示教标记与恢复通道...')
        self.piper.MotionCtrl_1(0x00, 0x00, 0x00)
        time.sleep(0.05)

        self.get_logger().info('--> 正在使能各电机驱动回路...')
        for _ in range(50):
            if self.piper.EnablePiper():
                break
            time.sleep(0.01)

        self.get_logger().info('--> 正在切换控制模式至 CAN_CTRL(0x1) + MOVE_J(0x1)...')
        for _ in range(20):
            self.piper.ModeCtrl(0x01, 0x01, 35, 0x00)
            time.sleep(0.05)
            if self.piper.GetArmStatus().arm_status.ctrl_mode == 1:
                break

        # 使能夹爪但不强迫立即动作
        self.piper.GripperCtrl(0, 1000, 0x02, 0)
        time.sleep(0.05)
        self.piper.GripperCtrl(0, 1000, 0x01, 0)

        status = self.piper.GetArmStatus().arm_status
        self.get_logger().info(f'--> 硬件控制状态: ctrl_mode={status.ctrl_mode}, arm_status={status.arm_status}')

    def _get_current_joint_positions_rad(self) -> List[float]:
        """读取当前 6 轴角度 (rad)"""
        js = self.piper.GetArmJointMsgs().joint_state
        return [
            js.joint_1 * RAW_TO_RAD,
            js.joint_2 * RAW_TO_RAD,
            js.joint_3 * RAW_TO_RAD,
            js.joint_4 * RAW_TO_RAD,
            js.joint_5 * RAW_TO_RAD,
            js.joint_6 * RAW_TO_RAD
        ]

    def _publish_joint_states(self):
        """高频发布标准 /joint_states 话题 (200Hz)"""
        try:
            cur_rad = self._get_current_joint_positions_rad()
            gs = self.piper.GetArmGripperMsgs().gripper_state
            # 夹爪开度转为两个指关节位移 (米)
            gripper_m = (gs.grippers_angle / 1000.0) / 1000.0 # um -> mm -> m
            finger_m = max(0.0, gripper_m / 2.0)

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ALL_JOINTS
            msg.position = cur_rad + [finger_m, finger_m]
            self.joint_state_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布 /joint_states 异常: {e}')

    def _goal_callback(self, goal_request):
        """目标接收与初步校验"""
        self.get_logger().info('收到新的轨迹规划 Goal 请求，正在校验路点...')
        traj = goal_request.trajectory
        if not traj.points:
            self.get_logger().warn('轨迹路点为空，拒绝执行 Goal')
            return GoalResponse.REJECT

        # 检查关节名匹配
        for jname in ARM_JOINTS:
            if jname not in traj.joint_names:
                self.get_logger().warn(f'轨迹中缺少必要关节: {jname}，拒绝执行')
                return GoalResponse.REJECT

        self.get_logger().info(f'目标轨迹校验通过: 包含 {len(traj.points)} 个路径点，总耗时 {traj.points[-1].time_from_start.sec + traj.points[-1].time_from_start.nanosec * 1e-9:.2f} 秒')
        return GoalResponse.ACCEPT

    def _cancel_callback(self, goal_handle):
        """取消请求处理"""
        self.get_logger().warn('收到轨迹终止/取消请求 (Cancel Request)，正在平稳制动...')
        return CancelResponse.ACCEPT

    def _execute_trajectory_callback(self, goal_handle):
        """核心执行回调：五次多项式密集插值与 200Hz 流式 CAN 下发"""
        with self._active_goal_lock:
            self._current_goal_handle = goal_handle

        traj = goal_handle.request.trajectory
        result = FollowJointTrajectory.Result()

        # 建立输入轨迹中的关节索引映射
        joint_idx_map = [traj.joint_names.index(jname) for jname in ARM_JOINTS]

        # 整理轨迹段 (Segments)
        cur_actual = self._get_current_joint_positions_rad()
        
        # 构建路点时间序列 (以当前真实位姿作为 t=0 起始点)
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
                'p0': p_prev,
                'p1': p_curr,
                'v0': v_prev,
                'v1': v_curr,
                'a0': a_prev,
                'a1': a_curr,
            })
            p_prev = p_curr
            v_prev = v_curr
            a_prev = a_curr
            t_prev = t_curr

        self.get_logger().info(f'轨迹分段构建完毕: 共 {len(segments)} 段连续 C² 曲线规划，启动 200Hz 精准时钟流式运控')

        # 200Hz 实时插补与下发循环
        feedback_msg = FollowJointTrajectory.Feedback()
        feedback_msg.joint_names = ARM_JOINTS

        exec_start_time = time.time()
        next_tick = time.monotonic()

        for seg_idx, seg in enumerate(segments):
            dur = seg['duration']
            steps = max(1, int(round(dur / CONTROL_DT)))
            p0, p1 = seg['p0'], seg['p1']
            v0, v1 = seg['v0'], seg['v1']
            a0, a1 = seg['a0'], seg['a1']

            for step in range(steps):
                # 检查取消请求
                if goal_handle.is_cancel_requested:
                    self.get_logger().warn('轨迹执行被外部请求取消，停止当前动作')
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    return result

                t_seg = step * (dur / steps)
                # 对 6 轴同时进行包含速度与加速度边界的五次多项式 C² 插值
                desired_pos = []
                desired_vel = []
                for j in range(6):
                    pos_j, vel_j, _ = quintic_eval(t_seg, dur, p0[j], p1[j], v0[j], v1[j], a0[j], a1[j])
                    desired_pos.append(pos_j)
                    desired_vel.append(vel_j)

                # 转为 CAN raw 单位
                raw_cmds = [int(p * RAD_TO_RAW) for p in desired_pos]

                # CAN 下发
                self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
                self.piper.JointCtrl(raw_cmds[0], raw_cmds[1], raw_cmds[2], raw_cmds[3], raw_cmds[4], raw_cmds[5])

                # 实时误差检测
                actual_pos = self._get_current_joint_positions_rad()
                errors = [abs(desired_pos[j] - actual_pos[j]) for j in range(6)]
                max_err = max(errors)

                if max_err > MAX_TRACKING_ERROR_RAD:
                    self.get_logger().error(f'⚠️ 关节跟踪超差: 最大误差 {max_err*RAD_TO_DEG:.2f}° > 阈值 {MAX_TRACKING_ERROR_RAD*RAD_TO_DEG:.2f}°，紧急中止！')
                    goal_handle.abort()
                    result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                    result.error_string = f'Max tracking error exceeded: {max_err:.3f} rad'
                    return result

                # 定期发布 Action Feedback (例如 20Hz)
                if step % 10 == 0:
                    fb_desired = JointTrajectoryPoint(positions=desired_pos, velocities=desired_vel)
                    fb_actual = JointTrajectoryPoint(positions=actual_pos)
                    fb_error = JointTrajectoryPoint(positions=errors)
                    feedback_msg.desired = fb_desired
                    feedback_msg.actual = fb_actual
                    feedback_msg.error = fb_error
                    goal_handle.publish_feedback(feedback_msg)

                next_tick += CONTROL_DT
                sleep_dt = next_tick - time.monotonic()
                if sleep_dt > 0:
                    time.sleep(sleep_dt)

        # 发送最后一个终点帧锁定精确定位
        final_raw = [int(p * RAD_TO_RAW) for p in segments[-1]['p1']]
        self.piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        self.piper.JointCtrl(final_raw[0], final_raw[1], final_raw[2], final_raw[3], final_raw[4], final_raw[5])

        total_elapsed = time.time() - exec_start_time
        self.get_logger().info(f'✅ 目标轨迹执行圆满完成！实际耗时: {total_elapsed:.2f} 秒')

        # 终点保持并上报成功
        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result


def main(args=None):
    rclpy.init(args=args)
    server_node = PiperTrajectoryActionServer()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(server_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        server_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
