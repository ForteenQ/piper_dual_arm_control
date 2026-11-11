#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER × Intel RealSense D435
easy_handeye2 官方核心标定交互控制台 (Eye-to-Hand 模式)
"""

import os
import sys
import time
import math
import json
import threading

import rclpy
from rclpy.node import Node
import tf2_ros

import easy_handeye2_msgs.srv as srv
from piper_sdk import C_PiperInterface_V2

# 安全关节角预设巡检姿态 (8 组，全部在安全工作空间内)
SWEEP_PRESETS_DEG = [
    [0.0, 20.0, -30.0, 0.0, 20.0, 0.0],       # 1. 中心基准位
    [10.0, 22.0, -28.0, 8.0, 22.0, -10.0],    # 2. 右前微侧
    [-10.0, 22.0, -28.0, -8.0, 22.0, 10.0],   # 3. 左前微侧
    [5.0, 15.0, -35.0, 5.0, 15.0, -5.0],      # 4. 后仰小角度
    [-5.0, 25.0, -25.0, -5.0, 25.0, 5.0],     # 5. 前探小角度
    [12.0, 18.0, -32.0, -10.0, 18.0, 15.0],   # 6. 右侧偏航+滚转
    [-12.0, 18.0, -32.0, 10.0, 18.0, -15.0],  # 7. 左侧偏航+滚转
    [0.0, 23.0, -27.0, 0.0, 23.0, 0.0]        # 8. 俯仰恢复
]


def main():
    rclpy.init()
    node = rclpy.create_node('piper_handeye2_runner')

    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer, node)

    # 启动后台 ROS 2 事件循环线程，确保 TF 与 Service 消息毫秒级处理
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    # 创建 easy_handeye2 服务客户端
    cli_take = node.create_client(srv.TakeSample, '/easy_handeye2/calibration/take_sample')
    cli_remove = node.create_client(srv.RemoveSample, '/easy_handeye2/calibration/remove_sample')
    cli_compute = node.create_client(srv.ComputeCalibration, '/easy_handeye2/calibration/compute_calibration')
    cli_save = node.create_client(srv.SaveCalibration, '/easy_handeye2/calibration/save_calibration')

    print("=" * 65)
    print("🚀 PIPER × easy_handeye2 工业级手眼标定控制台")
    print("模式: Eye-to-Hand (相机固定在工控台，标定板随机械臂运动)")
    print("标定板: GP290-20-12*9 (11x8 内角点, 20.0mm 陶瓷工业板)")
    print("=" * 65)

    print("\n⏳ 正在连接 easy_handeye2 核心服务...")
    if not cli_take.wait_for_service(timeout_sec=5.0):
        print("❌ 未检测到 /easy_handeye2/calibration 服务，请确认后台是否已运行 start_easy_handeye2.sh")
        return
    print("✅ 成功连接到 easy_handeye2 服务端！")

    # 连接机械臂 SDK 获取状态与运控
    piper = None
    try:
        piper = C_PiperInterface_V2("can0")
        piper.ConnectPort()
        print("✅ 机械臂 CAN 实时运控已连接")
    except Exception as e:
        print(f"⚠️ 机械臂 CAN 连接提示: {e}")

    def call_srv(cli, req):
        future = cli.call_async(req)
        t0 = time.time()
        while not future.done() and time.time() - t0 < 8.0:
            time.sleep(0.02)
        return future.result()

    def check_tf():
        print("\n--- 🔍 TF 跟踪与标定板定位检查 ---")
        try:
            t1 = tf_buffer.lookup_transform(
                'base_link', 'link6', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.5)
            )
            p1 = t1.transform.translation
            print(f"  [机械臂] base_link -> link6:   X={p1.x:.3f}m, Y={p1.y:.3f}m, Z={p1.z:.3f}m (就绪)")
        except Exception as e:
            print(f"  [机械臂] ❌ 未检测到 base_link -> link6 TF: {e}")
            print("         💡 提示: 请在另一终端运行 ~/piper 启动机械臂 TF 广播！")

        try:
            t2 = tf_buffer.lookup_transform(
                'camera_color_optical_frame', 'checkerboard', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.5)
            )
            p2 = t2.transform.translation
            dist = math.sqrt(p2.x**2 + p2.y**2 + p2.z**2)
            print(f"  [视觉端] 🎯 GP290 标定板稳定锁定: 距相机={dist*100.0:.1f} cm (X={p2.x:.3f}, Y={p2.y:.3f}, Z={p2.z:.3f})")
        except Exception as e:
            print(f"  [视觉端] ⚠️ 未在相机视野中检测到标定板: {e}")

    def take_sample():
        print("\n📸 正在采集当前位姿...")
        res = call_srv(cli_take, srv.TakeSample.Request())
        if res:
            count = len(res.samples.samples)
            print(f"✅ 采集成功！easy_handeye2 已记录有效位姿样本: {count} 组")
            return count
        else:
            print("❌ 采集失败，请确保标定板在相机视野中且机械臂静止")
            return 0

    def compute_calibration():
        print("\n⚡ 正在调用 easy_handeye2 (OpenCV Tsai-Lenz) 解算手眼外参...")
        res = call_srv(cli_compute, srv.ComputeCalibration.Request())
        if not res or not res.calibration:
            print("❌ 解算失败：样本数量不足或变换矩阵奇异，至少需要 3 组（推荐 8~12 组）不同角度位姿！")
            return None

        calib = res.calibration
        trans = calib.transform.transform.translation
        rot = calib.transform.transform.rotation

        print("\n" + "=" * 65)
        print("🎉🎉🎉 easy_handeye2 手眼标定计算完成！")
        print(f"相机坐标原点相对于机械臂基座 base_link 的空间位姿 (T_base_camera):")
        print(f"  平移 (米): X = {trans.x:+.4f} m, Y = {trans.y:+.4f} m, Z = {trans.z:+.4f} m")
        print(f"  四元数旋转: [x={rot.x:.4f}, y={rot.y:.4f}, z={rot.z:.4f}, w={rot.w:.4f}]")
        print("=" * 65)

        # 保存
        save_res = call_srv(cli_save, srv.SaveCalibration.Request())
        if save_res and save_res.success:
            print("💾 官方标定结果已持久化保存至: ~/.ros2/easy_handeye2/calibrations/")

        # 生成可直接在 ROS 2 launch 中使用的静态 TF 广播脚本
        tf_script_path = "/home/lab203/piper_students/lq/real_arm_control/publish_camera_tf.sh"
        with open(tf_script_path, "w") as f:
            f.write("#!/bin/bash\n")
            f.write("source /opt/ros/humble/setup.bash\n")
            f.write(f"ros2 run tf2_ros static_transform_publisher --x {trans.x:.4f} --y {trans.y:.4f} --z {trans.z:.4f} "
                    f"--qx {rot.x:.4f} --qy {rot.y:.4f} --qz {rot.z:.4f} --qw {rot.w:.4f} "
                    f"--frame-id base_link --child-frame-id camera_link\n")
        os.chmod(tf_script_path, 0o755)
        print(f"🚀 已自动生成静态 TF 发布启动脚本: {tf_script_path}")
        return calib

    def auto_sweep():
        if piper is None:
            print("❌ 机械臂 SDK 未连接，无法执行自动巡检")
            return
        print("\n🔄 启动自动多姿态巡检采样 (8 组安全姿态)...")
        piper.EnableArm(7)
        time.sleep(1)

        success_count = 0
        for i, preset in enumerate(SWEEP_PRESETS_DEG, 1):
            print(f"\n--- 移动至第 {i}/8 组位姿: {preset} 度 ---")
            raw_joints = [int(deg * 1000) for deg in preset]
            piper.JointCtrl(*raw_joints, 0, 15)  # 15% 安全低速
            time.sleep(3.0)  # 等待平稳到达

            # 抓拍一次
            c = take_sample()
            if c > 0:
                success_count += 1
            time.sleep(0.5)

        print(f"\n🏁 自动巡检采样结束！累计采样: {success_count} 组")
        if success_count >= 3:
            compute_calibration()

    while rclpy.ok():
        print("\n" + "-" * 40)
        print("操作选项:")
        print("  [1] 🔍 检查机械臂与标定板 TF 跟踪状态")
        print("  [2] 📸 采集当前点位 (Manual Take Sample)")
        print("  [3] 🔄 自动安全多姿态巡检采样 (Auto Sweep 8 Poses)")
        print("  [4] ⚡ 计算手眼外参矩阵 (Compute Hand-Eye)")
        print("  [5] 💾 保存标定并生成静态 TF 广播脚本")
        print("  [q] 退出控制台")
        print("-" * 40)

        try:
            choice = input("请选择 [1/2/3/4/5/q]: ").strip()
            if choice == '1':
                check_tf()
            elif choice == '2':
                take_sample()
            elif choice == '3':
                auto_sweep()
            elif choice == '4':
                compute_calibration()
            elif choice == '5':
                compute_calibration()
            elif choice.lower() == 'q':
                break
        except (KeyboardInterrupt, EOFError):
            break

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
