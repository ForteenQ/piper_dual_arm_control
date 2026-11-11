import time
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
time.sleep(0.5)

enable_status = piper.GetArmEnableStatus()
print("GetArmEnableStatus():", enable_status)
print("all(enable_status):", all(enable_status))

# 尝试调用 ModeCtrl 设置为 CAN 指令控制模式 (0x01)
print("\n发送 ModeCtrl(0x01, 0x01, 50, 0x00)...")
piper.ModeCtrl(0x01, 0x01, 50, 0x00)
time.sleep(0.5)

status = piper.GetArmStatus().arm_status
print(f"当前控制模式 (ctrl_mode): {status.ctrl_mode}")
