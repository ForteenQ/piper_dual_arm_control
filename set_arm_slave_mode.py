#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
松灵 PIPER 机械臂主从模式配置脚本：
将机械臂从“0xFA 示教输入臂 (TEACHING_MODE)”切换为“0xFC 运动输出臂 (SLAVE/EXECUTION_MODE)”。
"""
import time
from piper_sdk import C_PiperInterface_V2

def main():
    print("=" * 60)
    print("【模式切换】准备配置 PIPER 机械臂为：运动输出从臂 (Slave Mode)")
    print("=" * 60)
    
    piper = C_PiperInterface_V2("can0")
    piper.ConnectPort()
    time.sleep(0.5)
    
    status = piper.GetArmStatus().arm_status
    print(f"当前控制模式 (ctrl_mode): {status.ctrl_mode}")
    
    print("\n正在向 CAN ID 0x470 发送模式切换帧: linkage_config=0xFC (连续发送 10 次以确保固件稳定写入)...")
    for i in range(10):
        # 0xFC: 运动输出臂; 0, 0, 0: 保持默认基 ID 不偏移
        piper.MasterSlaveConfig(0xFC, 0x00, 0x00, 0x00)
        time.sleep(0.1)
        print(f"  -> 第 {i+1}/10 次写入帧发送成功")
        
    print("\n" + "=" * 60)
    print("【写入完成】0xFC 运动输出从臂指令已成功注入机械臂主控板！")
    print("【必要物理动作】请现在关闭机械臂 24V 直流电源，等待 3 秒后重新开机！")
    print("=" * 60)

if __name__ == "__main__":
    main()
