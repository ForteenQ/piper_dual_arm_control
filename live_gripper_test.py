#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂夹爪现场直观开合测试脚本
特性：
1. 锁定 6 轴保持绝对静止不动
2. 设定适当的驱动力矩 (2500，即 2.5 N*m)，克服静摩擦
3. 200Hz 高频流式控制，平滑展开至 50mm (5cm，肉眼极易辨识)
4. 峰值保持 3 秒，留出充足的人工目视观察窗口
5. 平缓闭合回零位
"""
import time
import math
from piper_sdk import C_PiperInterface_V2

def main():
    print("=" * 65)
    print("【实机现场目视测试】PIPER 夹爪 50mm 开合动作准备就绪")
    print("=" * 65)

    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.5)

    # 1. 采集并锁定当前 6 轴绝对角度
    js = piper.GetArmJointMsgs().joint_state
    j1, j2, j3 = js.joint_1, js.joint_2, js.joint_3
    j4, j5, j6 = js.joint_4, js.joint_5, js.joint_6
    print(f"--> 当前 6 轴机械臂保持刚性锁定：J1={j1/1000:.1f}° J2={j2/1000:.1f}° J3={j3/1000:.1f}°")

    # 2. 使能机械臂主控与夹爪
    print("--> 正在使能各电机驱动回路...")
    for _ in range(50):
        if piper.EnablePiper():
            break
        time.sleep(0.01)

    # 清除夹爪可能存在的堵转/保护状态 (0x02)，然后使能 (0x01)
    piper.GripperCtrl(0, 2500, 0x02, 0)
    time.sleep(0.1)
    piper.GripperCtrl(0, 2500, 0x01, 0)
    time.sleep(0.2)

    init_angle = piper.GetArmGripperMsgs().gripper_state.grippers_angle
    print(f"--> 当前夹爪初始读数: {init_angle/1000.0:.2f} mm")

    print("\n>>> 请将视线集中在机械臂末端夹爪！动作将在 3 秒倒计时后开始 <<<")
    for sec in [3, 2, 1]:
        print(f"    倒计时: {sec}...")
        time.sleep(1.0)
    print(">>> 动作开始！<<<")

    # 3. 200Hz 流式运动规划 (目标 50,000 um = 50.0 mm)
    dt = 0.005 # 5ms
    target_stroke_um = 50000 # 50mm (5厘米大开度)
    effort_cmd = 2500 # 2.5 N*m (安全有效力矩)

    # 阶段 A: 平滑张开 (3.0 秒)
    open_steps = int(3.0 / dt)
    print(">>> [1/3 正在张开] 平缓张开至 50 mm (5 厘米)...")
    for i in range(open_steps):
        s = i / float(open_steps)
        factor = s * s * (3.0 - 2.0 * s)
        target_um = int(target_stroke_um * factor)
        piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(target_um, effort_cmd, 0x01, 0)
        time.sleep(dt)

    # 阶段 B: 保持 50mm (5.0 秒现场目视观察窗口)
    print(">>> [2/3 峰值保持] 已张开至 50 mm 最大开度！保持 5.0 秒供现场确认...")
    hold_steps = int(5.0 / dt)
    for i in range(hold_steps):
        piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(target_stroke_um, effort_cmd, 0x01, 0)
        if i % 100 == 0:
            gs = piper.GetArmGripperMsgs().gripper_state
            print(f"    [保持中 {i*dt:.1f}s/5.0s] 目标={target_stroke_um/1000:.1f}mm | 实测反馈={gs.grippers_angle/1000:.2f}mm | 实时力矩={gs.grippers_effort}")
        time.sleep(dt)

    # 阶段 C: 平缓闭合 (3.0 秒)
    close_steps = int(3.0 / dt)
    print(">>> [3/3 正在闭合] 正在平缓闭合回 0 mm...")
    for i in range(close_steps):
        s = i / float(close_steps)
        factor = 1.0 - (s * s * (3.0 - 2.0 * s))
        target_um = int(target_stroke_um * factor)
        piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(target_um, effort_cmd, 0x01, 0)
        time.sleep(dt)

    # 最终保持静息
    time.sleep(0.5)
    final_gs = piper.GetArmGripperMsgs().gripper_state
    print(f"\n--> 动作全部完成！最终闭合读数: {final_gs.grippers_angle/1000.0:.2f} mm (力矩: {final_gs.grippers_effort})")
    print("=" * 65)

if __name__ == "__main__":
    main()
