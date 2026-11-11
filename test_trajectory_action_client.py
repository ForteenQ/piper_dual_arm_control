#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂 ROS 2 FollowJointTrajectory 动作测试客户端
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

class PiperTrajectoryTestClient(Node):
    def __init__(self):
        super().__init__('piper_trajectory_test_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
        self._current_joint_states = None
        self._action_done_event = threading.Event()
        self._result_code = None

        self.create_subscription(JointState, '/joint_states', self._js_callback, 10)
        self.get_logger().info('正在等待 /joint_states 话题与 Action Server...')

    def _js_callback(self, msg: JointState):
        if self._current_joint_states is None:
            curr = []
            for jname in ARM_JOINTS:
                if jname in msg.name:
                    idx = msg.name.index(jname)
                    curr.append(msg.position[idx])
            if len(curr) == 6:
                self._current_joint_states = curr

    def run_test(self):
        # 1. 等待 Server
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('未能在 5 秒内连接到 /arm_controller/follow_joint_trajectory！')
            return False

        # 2. 等待初始位姿
        start_wait = time.time()
        while self._current_joint_states is None and (time.time() - start_wait < 5.0):
            time.sleep(0.05)

        if self._current_joint_states is None:
            self.get_logger().error('未能获取有效 6 轴位姿！')
            return False

        p0 = list(self._current_joint_states)
        self.get_logger().info(f'初始基准位姿 (deg): {[round(x * 180.0 / math.pi, 2) for x in p0]}')

        # 构造路点: J6 +15° (0.2618 rad) 后回正
        delta_j6 = 15.0 * math.pi / 180.0
        p1 = list(p0)
        p1[5] += delta_j6
        p2 = list(p0)

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = ARM_JOINTS

        pt1 = JointTrajectoryPoint(positions=p1, velocities=[0.0]*6, time_from_start=Duration(sec=3, nanosec=0))
        pt2 = JointTrajectoryPoint(positions=p2, velocities=[0.0]*6, time_from_start=Duration(sec=6, nanosec=0))
        goal_msg.trajectory.points = [pt1, pt2]

        self.get_logger().info('正在发送 Goal 请求至 /arm_controller/follow_joint_trajectory...')
        send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self._feedback_callback)
        send_goal_future.add_done_callback(self._goal_response_callback)

        # 等待完成
        self._action_done_event.wait(timeout=15.0)
        return self._result_code == FollowJointTrajectory.Result.SUCCESSFUL

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal 被拒绝！')
            self._action_done_event.set()
            return

        self.get_logger().info('✅ Goal 已被接收！实机正在执行轨迹...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self._result_code = result.error_code
        self.get_logger().info(f'🎉 Action 最终执行完成！结果状态码: {result.error_code}')
        self._action_done_event.set()

    def _feedback_callback(self, feedback_msg):
        actual_j6_deg = feedback_msg.feedback.actual.positions[5] * 180.0 / math.pi
        desired_j6_deg = feedback_msg.feedback.desired.positions[5] * 180.0 / math.pi
        err_j6_deg = feedback_msg.feedback.error.positions[5] * 180.0 / math.pi
        print(f"    [Action 反馈] 目标={desired_j6_deg:.1f}° | 实际={actual_j6_deg:.1f}° | 误差={err_j6_deg:.2f}°")


def main(args=None):
    rclpy.init(args=args)
    client = PiperTrajectoryTestClient()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(client)

    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    success = client.run_test()
    print(f"--> 测试客户端执行结果: {'成功' if success else '失败'}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
