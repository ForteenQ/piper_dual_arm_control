import inspect
import sys
sys.path.append("/home/lab203/Desktop/piper_ros-humble/install/piper/lib/piper")
with open("/home/lab203/Desktop/piper_ros-humble/src/piper/piper/piper_ctrl_single_node.py") as f:
    text = f.read()

import re
matches = re.findall(r"def .*gripper.*", text, re.IGNORECASE)
print("Gripper functions in piper_ctrl_single_node:", matches)

# find gripper subscription / callback
for line in text.splitlines():
    if "gripper" in line.lower() and ("sub" in line.lower() or "pub" in line.lower() or "ctrl" in line.lower()):
        print("  ->", line.strip())
