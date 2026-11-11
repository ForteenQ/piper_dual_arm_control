#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂 - 安全展开至工作就绪姿态 (Ready Pose)
通过 ROS 2 FollowJointTrajectory 动作接口下发 8.0 秒慢速平滑展开轨迹

目标就绪姿态 (Ready Pose):
  - Joint 1:   0.0° (基座回正)
  - Joint 2: +20.0° (大臂平缓抬起)
  - Joint 3: -30.0° (小臂展开，脱离零位折叠区)
  - Joint 4:   0.0° (腕部自转回正)
  - Joint 5: +20.0° (腕部俯仰就绪)
  - Joint 6:   0.0° (末端工具法兰回正)

作者: 学生 LQ (203实验室 双臂抛接课题组)
空间: /home/lab203/piper_students/lq/real_arm_control/
域ID: ROS_DOMAIN_ID=42
"""

import math
import time
import threading
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
DEG_TO_RAD = math.pi / 180.0
RAD_TO_DEG = 180.0 / math.pi

# 目标就绪角度 (度 -> 弧度)
TARGET_READY_DEG = [0.0, 20.0, -30.0, 0.0, 20.0, 0.0]
TARGET_READY_RAD = [deg * DEG_TO_RAD for deg in TARGET_READY_DEG]
TRAJECTORY_DURATION_SEC = 8.0  # 8 秒慢速极平滑展开


class PiperReadyPoseUnfoldClient(Node):
    def __init__(self):
        super().__init__('piper_ready_pose_unfold_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self._current_joint_states = None
        self._action_done_event = threading.Event()
        self._result_code = None
        self._last_fb_time = 0.0

        self.create_subscription(JointState, '/joint_states', self._js_callback, 10)
        self.get_logger().info('正在等待 /joint_states 话题与 /arm_controller/follow_joint_trajectory 动作服务...')

    def _js_callback(self, msg: JointState):
        curr = []
        for jname in ARM_JOINTS:
            if jname in msg.name:
                idx = msg.name.index(jname)
                curr.append(msg.position[idx])
        if len(curr) == 6:
            self._current_joint_states = curr

    def execute_unfold(self) -> bool:
        # 1. 检查动作服务器
        self.get_logger().info('--> 正在连接轨迹动作服务...')
        if not self._action_client.wait_for_server(timeout_sec=8.0):
            self.get_logger().error('❌ 未能在 8 秒内连接到 /arm_controller/follow_joint_trajectory！请确认 action server 正在运行。')
            return False

        # 2. 等待当前物理位姿
        self.get_logger().info('--> 正在读取当前实机关节位姿...')
        start_wait = time.time()
        while self._current_joint_states is None and (time.time() - start_wait < 5.0):
            time.sleep(0.05)

        if self._current_joint_states is None:
            self.get_logger().error('❌ 未能从 /joint_states 话题获取有效 6 轴位姿！')
            return False

        p0 = list(self._current_joint_states)
        p0_deg = [round(x * RAD_TO_DEG, 2) for x in p0]
        self.get_logger().info(f'【当前起始位姿】: {p0_deg} (deg)')
        self.get_logger().info(f'【目标就绪位姿】: {TARGET_READY_DEG} (deg)')

        # 计算各关节位移量
        deltas = [TARGET_READY_DEG[i] - p0_deg[i] for i in range(6)]
        self.get_logger().info(f'【各轴位移总量】: {[round(d, 2) for d in deltas]} (deg)')
        self.get_logger().info(f'【规划运行时间】: {TRAJECTORY_DURATION_SEC:.1f} 秒 (五次多项式 S 曲线平滑插值)')

        # 构造 FollowJointTrajectory Goal
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ARM_JOINTS

        # 目标路点
        pt_target = JointTrajectoryPoint(
            positions=TARGET_READY_RAD,
            velocities=[0.0] * 6,
            accelerations=[0.0] * 6,
            time_from_start=Duration(sec=int(TRAJECTORY_DURATION_SEC), nanosec=int((TRAJECTORY_DURATION_SEC % 1.0) * 1e9))
        )
        goal_msg.trajectory.points = [pt_target]

        self.get_logger().info('--> 正在向 /arm_controller/follow_joint_trajectory 下发就绪展开 Goal...')
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

        # 等待动作完成 (超时预留 15 秒)
        self._action_done_event.wait(timeout=TRAJECTORY_DURATION_SEC + 7.0)

        if self._result_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info('✅ 就绪姿态安全展开成功完成！')
            time.sleep(0.5)
            if self._current_joint_states:
                final_deg = [round(x * RAD_TO_DEG, 2) for x in self._current_joint_states]
                self.get_logger().info(f'【展开后最终位姿】: {final_deg} (deg)')
            return True
        else:
            self.get_logger().error(f'❌ 动作执行失败或超时，返回码: {self._result_code}')
            return False

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal 被 Action Server 拒绝！')
            self._action_done_event.set()
            return

        self.get_logger().info('✅ Goal 已被接收！机械臂开始慢速平滑展开...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self._result_code = result.error_code
        self._action_done_event.set()

    def _feedback_callback(self, feedback_msg):
        now = time.time()
        if now - self._last_fb_time >= 1.0:
            self._last_fb_time = now
            cur_pos = [round(p * RAD_TO_DEG, 1) for p in feedback_msg.feedback.actual.positions]
            max_err = max([abs(e * RAD_TO_DEG) for e in feedback_msg.feedback.error.positions]) if feedback_msg.feedback.error.positions else 0.0
            print(f"    [实机展开中] 当前位姿: {cur_pos} | 最大轴向误差: {max_err:.2f}°")


def main(args=None):
    rclpy.init(args=args)
    client = PiperReadyPoseUnfoldClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(client)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    success = client.execute_unfold()
    print(f"--> 就绪姿态展开任务结果: {'成功' if success else '失败'}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
