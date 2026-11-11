#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂底层实机状态读取验证脚本
功能：通过 CAN 总线连接机械臂，读取当前模式、状态码、6 个关节绝对位置以及夹爪行程
"""
import time
import math
from piper_sdk import C_PiperInterface_V2

def main():
    print("=" * 60)
    print("正在初始化 PIPER 机械臂 CAN 接口 (can0)...")
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    
    # 稳定等待 CAN 帧填入 SDK 缓存
    print("正在监听总线数据帧...")
    for _ in range(30):
        time.sleep(0.02)
        
    status = piper.GetArmStatus()
    joints = piper.GetArmJointMsgs()
    gripper = piper.GetArmGripperMsgs()
    
    print("\n[1] 机械臂全局状态 (Arm Status):")
    arm_st = status.arm_status
    print(f"  - 控制模式 (ctrl_mode): {arm_st.ctrl_mode}")
    print(f"  - 机械臂状态 (arm_status): {arm_st.arm_status}")
    print(f"  - 示教状态 (mode_feed): {arm_st.mode_feed}")
    print(f"  - 错误代码 (err_code): {arm_st.err_code}")
    
    print("\n[2] 6轴关节当前位置 (Joint States):")
    js = joints.joint_state
    # 官方 SDK 单位：joint_1~6 为 0.001 度 (0.001 deg)
    raw_angles = [
        js.joint_1, js.joint_2, js.joint_3,
        js.joint_4, js.joint_5, js.joint_6
    ]
    for idx, raw in enumerate(raw_angles, start=1):
        deg = raw / 1000.0
        rad = deg * math.pi / 180.0
        print(f"  - Joint {idx}: {deg:8.3f}° ({rad:8.4f} rad) [原始值: {raw}]")
        
    print("\n[3] 夹爪当前状态 (Gripper State):")
    gs = gripper.gripper_state
    # 官方 SDK 单位：grippers_angle 0.001mm (即微米 um)，effort 单位 0.001 N*m 或类似
    stroke_mm = gs.grippers_angle / 1000.0
    stroke_m = stroke_mm / 1000.0
    effort = gs.grippers_effort
    print(f"  - 夹爪开度 (行程): {stroke_mm:.2f} mm ({stroke_m:.4f} m) [原始值: {gs.grippers_angle}]")
    print(f"  - 夹爪力矩/电流反馈: {effort} [原始值]")
    if hasattr(gs, 'status_code'):
        print(f"  - 夹爪状态码: {gs.status_code}")
    print("=" * 60)

if __name__ == "__main__":
    main()
