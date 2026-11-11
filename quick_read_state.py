import time
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
time.sleep(0.5)

st = piper.GetArmStatus().arm_status
print("当前状态:")
print(f"  ctrl_mode   : {st.ctrl_mode}")
print(f"  arm_status  : {st.arm_status}")
print(f"  teach_status: {st.teach_status}")
print(f"  mode_feed   : {st.mode_feed}")
print(f"  motion_status: {st.motion_status}")
