with open("/home/lab203/Desktop/piper_sdk/piper_sdk/piper_msgs/msg_v2/feedback/arm_feedback_status.py") as f:
    lines = f.readlines()

for idx, line in enumerate(lines[:60]):
    print(f"{idx+1}: {line}", end="")
