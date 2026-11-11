#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂 Joint 6 末端手腕目视可见安全微动测试脚本 (±15.0°)
安全与控制要点：
1. 先切换模式至 CAN 指令控制 (ModeCtrl: 0x01, 0x01, 30, 0x00) 并校验确认
2. 摆动幅度提升至 ±15.0° (手腕共 30° 旋转弧度，极易肉眼辨识)
3. 运动速度设为 30% (平稳可见，五次多项式 S 曲线平滑过渡，无冲击)
4. J1~J5 轴锁定当前角度绝对静止，仅 J6 轴绕工具轴自转 (零空间碰撞风险)
5. 实时打印编码器物理角度反馈，严格确认物理动作到位
"""
import time
import math
from piper_sdk import C_PiperInterface_V2

def quintic_s_curve(s):
    """五次多项式插值: s in [0, 1] -> [0, 1]"""
    return s * s * s * (10.0 - 15.0 * s + 6.0 * s * s)

def main():
    print("=" * 70)
    print("【实机直观目视测试】PIPER 机械臂 Joint 6 (末端手腕) ±15.0° 明显摆动测试")
    print("=" * 70)

    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.5)

    # 1. 采集当前 6 轴基准位置
    js = piper.GetArmJointMsgs().joint_state
    j1, j2, j3 = js.joint_1, js.joint_2, js.joint_3
    j4, j5, j6_init = js.joint_4, js.joint_5, js.joint_6
    
    print(f"--> 当前各轴基准读数:")
    print(f"    J1={j1/1000.0:.2f}° | J2={j2/1000.0:.2f}° | J3={j3/1000.0:.2f}°")
    print(f"    J4={j4/1000.0:.2f}° | J5={j5/1000.0:.2f}° | J6={j6_init/1000.0:.2f}° (测试轴)")

    # 2. 使能机械臂各轴电机
    print("--> 正在使能各电机驱动回路...")
    for _ in range(50):
        if piper.EnablePiper():
            break
        time.sleep(0.01)

    # 3. 关键切换：显式切换控制模式为 CAN 指令控制模式 (MOVE_J)
    print("--> 正在切换为 CAN 指令控制模式 (CAN_CTRL + MOVE_J)...")
    for _ in range(20):
        piper.ModeCtrl(0x01, 0x01, 30, 0x00)
        time.sleep(0.05)
        ctrl_mode = piper.GetArmStatus().arm_status.ctrl_mode
        if ctrl_mode == 1: # CAN_CTRL
            break

    ctrl_mode = piper.GetArmStatus().arm_status.ctrl_mode
    print(f"--> 当前控制模式状态码: {ctrl_mode} (1=CAN_CTRL 控制模式就绪)")

    # 夹爪保持微力矩贴合 0mm
    piper.GripperCtrl(0, 1500, 0x01, 0)
    time.sleep(0.2)

    delta_deg = 15.0 # ±15.0 度 (共 30° 旋转，极易辨识)
    delta_raw = int(delta_deg * 1000) # 15000 原始值
    dt = 0.005 # 5ms (200Hz)
    speed_percent = 30 # 30% 速度

    print("\n>>> 请将视线集中在机械臂末端手腕（夹爪根部的旋转圆盘）！<<<")
    for sec in [3, 2, 1]:
        print(f"    倒计时: {sec}...")
        time.sleep(1.0)
    print(">>> 动作开始！<<<")

    # 阶段 1: 0 -> +15.0° (2.5秒)
    print(f"\n[1/5 正向旋转] 从 {j6_init/1000.0:.1f}° 转动至 {(j6_init + delta_raw)/1000.0:.1f}° (+15.0°)...")
    steps = int(2.5 / dt)
    for i in range(steps):
        factor = quintic_s_curve(i / float(steps))
        cur_j6 = int(j6_init + delta_raw * factor)
        piper.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, cur_j6)
        piper.GripperCtrl(0, 1500, 0x01, 0)
        if i % 100 == 0:
            fb = piper.GetArmJointMsgs().joint_state.joint_6
            print(f"    进度 {i*dt:.1f}s | 指令={cur_j6/1000.0:.1f}° | 实测反馈={fb/1000.0:.1f}°")
        time.sleep(dt)

    # 阶段 2: 保持 +15.0° (2.0秒观察)
    print(f"[2/5 极限保持] 保持在 +15.0° 极限位置，留出 2.0 秒观察窗口...")
    steps = int(2.0 / dt)
    for i in range(steps):
        piper.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6_init + delta_raw)
        piper.GripperCtrl(0, 1500, 0x01, 0)
        time.sleep(dt)
    fb = piper.GetArmJointMsgs().joint_state.joint_6
    print(f"    +15° 极限实测反馈: {fb/1000.0:.2f}° (实测偏角: {(fb - j6_init)/1000.0:+.2f}°)")

    # 阶段 3: +15.0° -> -15.0° (4.0秒)
    print(f"\n[3/5 反向大旋转] 从 +15.0° 平稳反转至 -15.0° (总行程 30.0°)...")
    steps = int(4.0 / dt)
    for i in range(steps):
        factor = quintic_s_curve(i / float(steps))
        cur_j6 = int((j6_init + delta_raw) - 2.0 * delta_raw * factor)
        piper.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, cur_j6)
        piper.GripperCtrl(0, 1500, 0x01, 0)
        if i % 100 == 0:
            fb = piper.GetArmJointMsgs().joint_state.joint_6
            print(f"    进度 {i*dt:.1f}s | 指令={cur_j6/1000.0:.1f}° | 实测反馈={fb/1000.0:.1f}°")
        time.sleep(dt)

    # 阶段 4: 保持 -15.0° (2.0秒观察)
    print(f"[4/5 极限保持] 保持在 -15.0° 极限位置，留出 2.0 秒观察窗口...")
    steps = int(2.0 / dt)
    for i in range(steps):
        piper.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, j6_init - delta_raw)
        piper.GripperCtrl(0, 1500, 0x01, 0)
        time.sleep(dt)
    fb = piper.GetArmJointMsgs().joint_state.joint_6
    print(f"    -15° 极限实测反馈: {fb/1000.0:.2f}° (实测偏角: {(fb - j6_init)/1000.0:+.2f}°)")

    # 阶段 5: -15.0° -> 0 (2.5秒回正)
    print(f"\n[5/5 平缓复位] 回归初始基准位置 {j6_init/1000.0:.1f}°...")
    steps = int(2.5 / dt)
    for i in range(steps):
        factor = quintic_s_curve(i / float(steps))
        cur_j6 = int((j6_init - delta_raw) + delta_raw * factor)
        piper.MotionCtrl_2(0x01, 0x01, speed_percent, 0x00)
        piper.JointCtrl(j1, j2, j3, j4, j5, cur_j6)
        piper.GripperCtrl(0, 1500, 0x01, 0)
        time.sleep(dt)

    time.sleep(0.5)
    final_js = piper.GetArmJointMsgs().joint_state
    print("\n" + "=" * 70)
    print("--> 动作全部执行完毕！当前各轴最终遥测角度:")
    print(f"    J1={final_js.joint_1/1000.0:.2f}° | J2={final_js.joint_2/1000.0:.2f}° | J3={final_js.joint_3/1000.0:.2f}°")
    print(f"    J4={final_js.joint_4/1000.0:.2f}° | J5={final_js.joint_5/1000.0:.2f}° | J6={final_js.joint_6/1000.0:.2f}°")
    print(f"    J6 复位静差: {abs(final_js.joint_6 - j6_init)/1000.0:.3f}°")
    print("=" * 70)

if __name__ == "__main__":
    main()
