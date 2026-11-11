with open("/home/lab203/Desktop/piper_sdk/piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_status.py") as f:
    text = f.read()

pos = text.find("class ArmMsgFeedbackStatus")
if pos != -1:
    print(text[pos:pos+1500])
