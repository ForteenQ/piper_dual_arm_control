import time
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
piper.ConnectPort()
time.sleep(0.5)

# 查询末端参数 (4 代表查询夹爪/示教器参数)
piper.ArmParamEnquiryAndConfig(4)
time.sleep(0.2)
param_feedback = piper.GetGripperTeachingPendantParamFeedback()
print("当前末端参数反馈:", param_feedback)
