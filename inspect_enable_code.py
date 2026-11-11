with open("/home/lab203/Desktop/piper_sdk/piper_sdk/interface/piper_interface_v2.py") as f:
    lines = f.readlines()

for idx, line in enumerate(lines):
    if "def EnableArm" in line or "def EnablePiper" in line:
        for j in range(idx, min(idx + 35, len(lines))):
            print(lines[j], end="")
        print("-" * 50)
