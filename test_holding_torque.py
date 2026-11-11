#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂 10 秒持续刚度闭环使能测试
目标：在 10 秒持续运行期间，通过高频心跳激活电机功率级，验证底座是否由“绿灯闪烁”转为“绿灯常亮”，以及各关节是否由“松软”进入“通电锁死受力”。
"""
import time
from piper_sdk import C_PiperInterface_V2

def main():
    print("=" * 65)
    print("【刚度使能测试】即将启动 10 秒持续受控闭环...")
    print("=" * 65)

    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.5)

    # 读取当前各轴静止角度
    js = piper.GetArmJointMsgs().joint_state
    j1, j2, j3 = js.joint_1, js.joint_2, js.joint_3
    j4, j5, j6 = js.joint_4, js.joint_5, js.joint_6
    print(f"锁定静止位姿: J1={j1/1000:.1f}° J2={j2/1000:.1f}° J3={j3/1000:.1f}°")

    # 复位与清除示教锁 (MotionCtrl_1 0x02 恢复)
    print("发送系统复位恢复指令 (0x02)...")
    piper.MotionCtrl_1(0x02, 0, 0)
    time.sleep(0.1)

    # 持续 10 秒的 200Hz 高频控制流
    dt = 0.005 # 5ms (200Hz)
    duration = 10.0
    steps = int(duration / dt)

    print("\n" + ">" * 15 + " [10 秒使能中！请立即观察底座绿灯与轻推手臂刚度] " + "<" * 15)

    for i in range(steps):
        t = i * dt
        # 持续高频下发全关节使能 (0x02)
        piper.EnableArm(7, 0x02)
        # 持续下发 CAN 指令控制心跳 (0x01)
        piper.MotionCtrl_2(0x01, 0x01, 30, 0x00)
        # 持续下发当前角度位置保持锁定
        piper.JointCtrl(j1, j2, j3, j4, j5, j6)

        # 夹爪在 3~7 秒期间尝试动作
        if 3.0 <= t <= 7.0:
            piper.GripperCtrl(40000, 3000, 0x01, 0)
        else:
            piper.GripperCtrl(0, 3000, 0x01, 0)

        if i % 100 == 0:
            st = piper.GetArmStatus().arm_status
            gs = piper.GetArmGripperMsgs().gripper_state
            print(f"  [运行中 t={t:4.1f}s/10s] 模式={st.ctrl_mode} | 夹爪反馈={gs.grippers_angle/1000:.2f}mm | 力矩={gs.grippers_effort}")

        time.sleep(dt)

    print("\n" + "=" * 65)
    print("【10 秒测试结束】控制流已平稳退出")
    print("=" * 65)

if __name__ == "__main__":
    main()
