with open("/home/lab203/Desktop/piper_sdk/piper_sdk/interface/piper_interface_v2.py") as f:
    text = f.read()

import re
matches = re.findall(r"def (?:ModeCtrl|MotionCtrl.*|Enable.*)\([^)]*\):", text)
print("Control functions:", matches)

# Check ModeCtrl implementation
pos = text.find("def ModeCtrl")
if pos != -1:
    print("\n=== ModeCtrl implementation ===")
    print(text[pos:pos+1200])

pos2 = text.find("def MotionCtrl_2")
if pos2 != -1:
    print("\n=== MotionCtrl_2 implementation ===")
    print(text[pos2:pos2+1200])
