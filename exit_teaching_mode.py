import time
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
time.sleep(0.5)

print("=== 当前状态 ===")
st = piper.GetArmStatus().arm_status
print(f"ctrl_mode: {st.ctrl_mode}, teach_status: {st.teach_status}")

print("\n正在向 0x150 发送退出拖动示教指令 (grag_teach_ctrl=0x02 结束示教记录)...")
for _ in range(5):
    piper.MotionCtrl_1(0x00, 0x00, 0x02)
    time.sleep(0.05)

time.sleep(0.2)
print("正在向 0x150 发送关闭示教指令 (grag_teach_ctrl=0x00)...")
for _ in range(5):
    piper.MotionCtrl_1(0x00, 0x00, 0x00)
    time.sleep(0.05)

time.sleep(0.2)
print("正在向 0x151 发送切入 CAN 控制模式 (ctrl_mode=0x01)...")
for _ in range(10):
    piper.MotionCtrl_2(0x01, 0x01, 50, 0x00)
    time.sleep(0.05)

time.sleep(0.3)
print("\n=== 指令发送后状态 ===")
st2 = piper.GetArmStatus().arm_status
print(f"ctrl_mode: {st2.ctrl_mode}, teach_status: {st2.teach_status}")
print("全局状态:", st2)
