#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂夹爪安全闭环开合测试脚本 (200Hz 确定性流式控制)
隔离工作区：/home/lab203/piper_students/lq/real_arm_control/
"""
import time
import math
from piper_sdk import C_PiperInterface_V2

def main():
    print("=" * 60)
    print("【方案 C 闭环实测】PIPER 机械臂 200Hz 安全夹爪开合测试启动")
    print("=" * 60)

    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.5)

    # 1. 读取当前 6 轴绝对关节角度，锁定手臂保持静止
    js = piper.GetArmJointMsgs().joint_state
    j1, j2, j3 = js.joint_1, js.joint_2, js.joint_3
    j4, j5, j6 = js.joint_4, js.joint_5, js.joint_6
    print(f"当前 6 轴锁定位置: J1={j1/1000:.2f}°, J2={j2/1000:.2f}°, J3={j3/1000:.2f}°, J4={j4/1000:.2f}°, J5={j5/1000:.2f}°, J6={j6/1000:.2f}°")

    # 2. 使能机械臂驱动与夹爪
    print("正在使能各关节驱动回路...")
    for _ in range(50):
        if piper.EnablePiper():
            break
        time.sleep(0.01)

    piper.GripperCtrl(0, 1000, 0x02, 0) # 清除夹爪状态
    time.sleep(0.05)
    piper.GripperCtrl(0, 1000, 0x01, 0) # 使能夹爪
    time.sleep(0.1)

    init_stroke_mm = piper.GetArmGripperMsgs().gripper_state.grippers_angle / 1000.0
    print(f"初始夹爪物理开度: {init_stroke_mm:.2f} mm")

    # 3. 200Hz 流式平滑控制循环 (周期 0.005s)
    # 阶段 1: 0~1.0s 建立 CAN 控制流，保持当前位姿
    # 阶段 2: 1.0~3.0s (400 周期) 平滑张开至 30.0 mm
    # 阶段 3: 3.0~4.5s (300 周期) 保持 30.0 mm，观察稳态反馈
    # 阶段 4: 4.5~6.5s (400 周期) 平滑闭合至 0.0 mm
    # 阶段 5: 6.5~7.0s (100 周期) 闭合保持并结束
    dt = 0.005 # 5ms (200Hz)
    total_time = 7.0
    total_steps = int(total_time / dt)

    target_max_um = 30000 # 30 mm
    print(f"\n正在执行 200Hz 流式控制：平滑展开至 30.0 mm 并平稳回闭...")

    peak_stroke_mm = 0.0
    log_counter = 0

    for step in range(total_steps):
        t = step * dt
        
        # S 曲线 / 正弦平滑规划目标开度
        if t < 1.0:
            target_um = 0
        elif 1.0 <= t < 3.0:
            # 2 秒从 0 平滑张开到 30000
            s = (t - 1.0) / 2.0
            # 简易平滑函数 s^2 * (3 - 2s)
            factor = s * s * (3.0 - 2.0 * s)
            target_um = int(target_max_um * factor)
        elif 3.0 <= t < 4.5:
            target_um = target_max_um
        elif 4.5 <= t < 6.5:
            # 2 秒从 30000 平滑闭合到 0
            s = (t - 4.5) / 2.0
            factor = 1.0 - (s * s * (3.0 - 2.0 * s))
            target_um = int(target_max_um * factor)
        else:
            target_um = 0

        # 发送 200Hz 心跳帧与关节锁定指令
        piper.MotionCtrl_2(0x01, 0x01, 30, 0x00) # CAN 模式, 关节模式, 30% 速度
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)
        piper.GripperCtrl(target_um, 1000, 0x01, 0)

        # 实时采样当前实际开度与力矩
        cur_gs = piper.GetArmGripperMsgs().gripper_state
        cur_stroke_mm = cur_gs.grippers_angle / 1000.0
        if cur_stroke_mm > peak_stroke_mm:
            peak_stroke_mm = cur_stroke_mm

        log_counter += 1
        if log_counter % 40 == 0: # 每 0.2 秒打印一次
            arm_mode = piper.GetArmStatus().arm_status.ctrl_mode
            print(f"  [t={t:4.2f}s] 目标开度={target_um/1000.0:5.2f} mm | 实测开度={cur_stroke_mm:5.2f} mm | 实测力矩={cur_gs.grippers_effort:3d} | 臂模式={arm_mode}")

        time.sleep(dt)

    time.sleep(0.2)
    final_gs = piper.GetArmGripperMsgs().gripper_state
    final_stroke_mm = final_gs.grippers_angle / 1000.0
    print("\n" + "=" * 60)
    print(f"【闭环实测报告】")
    print(f"  - 初始开度: {init_stroke_mm:.2f} mm")
    print(f"  - 实测最大张开度: {peak_stroke_mm:.2f} mm")
    print(f"  - 最终回位开度: {final_stroke_mm:.2f} mm")
    print("=" * 60)

if __name__ == "__main__":
    main()
