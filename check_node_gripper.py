with open("/home/lab203/Desktop/piper_ros-humble/src/piper/piper/piper_ctrl_single_node.py") as f:
    lines = f.readlines()

for idx, line in enumerate(lines):
    if "GripperCtrl" in line:
        print(f"Line {idx+1}: {line.strip()}")
