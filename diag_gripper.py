import time
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
time.sleep(0.5)

print("Arm status:", piper.GetArmStatus().arm_status)
print("Gripper msgs:", piper.GetArmGripperMsgs())
print("Gripper ctrl feedback:", piper.GetArmGripperCtrl())
print("Is enable:", piper.GetArmStatus().arm_status.ctrl_mode)
