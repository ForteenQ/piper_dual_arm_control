#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 夹爪参数配置与零点校准及安全动作脚本
1. 设置末端类型为标准二指夹爪 (max_range=70 mm)
2. 设定当前物理闭合位为零点 (0xAE)
3. 验证参数生效
4. 执行 0 -> 30mm -> 0 的平滑开合
"""
import time
from piper_sdk import C_PiperInterface_V2

def main():
    print("=" * 60)
    print("【步骤 1】配置末端执行器为标准夹爪 (70mm 行程限制)")
    print("=" * 60)
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.5)

    # 1. 写入夹爪参数：teaching_range_per=100, max_range_config=70, teaching_friction=1
    print("正在向 0x47D 发送夹爪行程限制参数 (max_range=70)...")
    for _ in range(5):
        piper.GripperTeachingPendantParamConfig(100, 70, 1)
        time.sleep(0.1)

    # 查询确认
    piper.ArmParamEnquiryAndConfig(4)
    time.sleep(0.3)
    fb = piper.GetGripperTeachingPendantParamFeedback()
    print("当前末端参数配置反馈:", fb)

    # 2. 设定夹爪零点
    print("\n" + "=" * 60)
    print("【步骤 2】设定夹爪当前闭合状态为绝对 0 点 (0xAE)...")
    print("=" * 60)
    piper.GripperCtrl(0, 1000, 0x00, 0)
    time.sleep(1.0)
    piper.GripperCtrl(0, 1000, 0x00, 0xAE)
    time.sleep(1.0)

    # 读取夹爪状态
    gs = piper.GetArmGripperMsgs().gripper_state
    print(f"夹爪当前状态反馈: 开度={gs.grippers_angle/1000.0:.2f} mm, 力矩={gs.grippers_effort}")

    # 3. 执行一次平缓微开测试 (张开 30mm)
    print("\n" + "=" * 60)
    print("【步骤 3】执行 200Hz 安全平滑开合动作 (30mm)")
    print("=" * 60)
    piper.GripperCtrl(0, 1000, 0x01, 0)
    time.sleep(0.2)

    # 锁定 6 轴当前姿态
    js = piper.GetArmJointMsgs().joint_state
    j1, j2, j3, j4, j5, j6 = js.joint_1, js.joint_2, js.joint_3, js.joint_4, js.joint_5, js.joint_6

    dt = 0.005 # 200Hz
    # 2秒张开到 30mm
    steps = int(2.0 / dt)
    print("正在平缓张开至 30.0 mm...")
    for i in range(steps):
        s = i / float(steps)
        factor = s * s * (3.0 - 2.0 * s)
        target_um = int(30000 * factor)
        piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(target_um, 1000, 0x01, 0)
        time.sleep(dt)

    print("已到达 30.0 mm 目标，保持 1.5 秒观测实际开度...")
    for _ in range(int(1.5 / dt)):
        piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(30000, 1000, 0x01, 0)
        time.sleep(dt)

    mid_gs = piper.GetArmGripperMsgs().gripper_state
    print(f"--> [30mm 目标保持期] 实测物理开度: {mid_gs.grippers_angle/1000.0:.2f} mm, 实测力矩: {mid_gs.grippers_effort}")

    print("正在平缓闭合回 0.0 mm...")
    for i in range(steps):
        s = i / float(steps)
        factor = 1.0 - (s * s * (3.0 - 2.0 * s))
        target_um = int(30000 * factor)
        piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(target_um, 1000, 0x01, 0)
        time.sleep(dt)

    time.sleep(0.5)
    final_gs = piper.GetArmGripperMsgs().gripper_state
    print(f"--> [最终闭合] 实测物理开度: {final_gs.grippers_angle/1000.0:.2f} mm, 实测力矩: {final_gs.grippers_effort}")

if __name__ == "__main__":
    main()
