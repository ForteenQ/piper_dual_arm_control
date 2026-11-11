with open("/home/lab203/Desktop/piper_sdk/piper_sdk/piper_msgs/msg_v2/can_id.py") as f:
    text = f.read()

import glob
for p in glob.glob("/home/lab203/Desktop/piper_sdk/piper_sdk/piper_msgs/msg_v2/*motion_ctrl_1*.py"):
    with open(p) as f:
        print(p)
        print(f.read())
for p in glob.glob("/home/lab203/Desktop/piper_sdk/piper_sdk/piper_msgs/msg_v2/*/*motion_ctrl_1*.py"):
    with open(p) as f:
        print(p)
        print(f.read())
