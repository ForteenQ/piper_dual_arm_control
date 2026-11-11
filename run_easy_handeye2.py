#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER × Intel RealSense D435
easy_handeye2 手眼标定自动化运行器 (Eye-to-Hand 模式)
"""

import os
import sys
import time
import math
import subprocess

import rclpy
from rclpy.node import Node
import tf2_ros

import easy_handeye2 as hec
import easy_handeye2_msgs as ehm
from easy_handeye2_msgs import srv
from easy_handeye2.handeye_calibration import HandeyeCalibrationParameters
from easy_handeye2.handeye_client import HandeyeClient

from piper_sdk import C_PiperInterface_V2

CALIB_NAME = "piper_d435_eye_to_hand"


def main():
    rclpy.init()

    # 1. 启动 TF 缓冲与监听器
    node = rclpy.create_node('piper_easy_handeye2_runner')
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    print("=" * 60)
    print("🚀 PIPER × easy_handeye2 标定客户端已启动")
    print("模式: Eye-to-Hand (相机固定基座旁，标定板随机械臂运动)")
    print("=" * 60)

    params = HandeyeCalibrationParameters(
        name=CALIB_NAME,
        calibration_type='eye_on_base',
        robot_base_frame='base_link',
        robot_effector_frame='link6',
        tracking_base_frame='camera_color_optical_frame',
        tracking_marker_frame='checkerboard',
        freehand_robot_movement=True
    )

    print("\n⏳ 正在等待 easy_handeye2 标定核心服务响应...")
    client = None
    try:
        client = HandeyeClient(node, params)
    except Exception as e:
        print(f"⚠️ 连接标定服务提示: {e}")
        return

    # 连接机械臂 SDK 获取关节角或控制运动
    piper = None
    try:
        piper = C_PiperInterface_V2("can0")
        piper.ConnectPort()
        print("✅ 机械臂 CAN 通信已建立")
    except Exception as e:
        print(f"⚠️ 机械臂连接提示: {e}")

    def check_tf_status():
        print("\n--- 实时 TF 状态检查 ---")
        try:
            t1 = tf_buffer.lookup_transform('base_link', 'link6', rclpy.time.Time())
            p1 = t1.transform.translation
            print(f"✅ 机械臂末端 TF 正常: base_link -> link6: [{p1.x:.3f}, {p1.y:.3f}, {p1.z:.3f}]")
        except Exception as e:
            print(f"❌ 机械臂末端 TF 未就绪: {e}")

        try:
            t2 = tf_buffer.lookup_transform('camera_color_optical_frame', 'checkerboard', rclpy.time.Time())
            p2 = t2.transform.translation
            dist = math.sqrt(p2.x**2 + p2.y**2 + p2.z**2)
            print(f"✅ 标定板视觉 TF 锁定正常: 距相机 {dist*100.0:.1f} cm: [{p2.x:.3f}, {p2.y:.3f}, {p2.z:.3f}]")
        except Exception as e:
            print(f"⚠️ 标定板视觉 TF 未锁定: {e}")

    def take_sample():
        print("\n📸 正在采集当前位姿样本...")
        try:
            resp = client.take_sample_client.call(ehm.srv.TakeSample.Request())
            samples = resp.samples.samples
            print(f"✅ 采集成功！当前有效样本总数: {len(samples)} 组")
            return len(samples)
        except Exception as e:
            print(f"❌ 采集失败: {e}")
            return 0

    def compute_and_save():
        print("\n⚡ 正在调用 easy_handeye2 计算外参矩阵...")
        try:
            resp = client.compute_calibration_client.call(ehm.srv.ComputeCalibration.Request())
            calib = resp.calibration
            trans = calib.transform.transform.translation
            rot = calib.transform.transform.rotation
            print("=" * 60)
            print("🎉 手眼标定解算成功！")
            print(f"基座到相机平移 (米): X={trans.x:.4f}, Y={trans.y:.4f}, Z={trans.z:.4f}")
            print(f"四元数旋转 [qx, qy, qz, qw]: [{rot.x:.4f}, {rot.y:.4f}, {rot.z:.4f}, {rot.w:.4f}]")
            print("=" * 60)

            # 保存
            save_resp = client.save_calibration_client.call(ehm.srv.SaveCalibration.Request())
            if save_resp.success:
                print("💾 标定结果已保存至 ~/.ros/easy_handeye2/")
        except Exception as e:
            print(f"❌ 解算失败: {e}")

    print("\n交互指令:")
    print("  [1] 检查当前 TF 状态 (Check TF)")
    print("  [2] 采集当前点位 (Take Sample)")
    print("  [3] 计算并保存标定结果 (Compute & Save)")
    print("  [q] 退出 (Quit)")

    while rclpy.ok():
        try:
            choice = input("\n请输入指令 [1/2/3/q]: ").strip()
            if choice == '1':
                check_tf_status()
            elif choice == '2':
                take_sample()
            elif choice == '3':
                compute_and_save()
            elif choice.lower() == 'q':
                break
        except (KeyboardInterrupt, EOFError):
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
