import inspect
from piper_sdk import C_PiperInterface_V2
piper = C_PiperInterface_V2("can0")
print("=== EnablePiper Doc ===")
print(inspect.getdoc(piper.EnablePiper))
print("=== EnableArm Doc ===")
if hasattr(piper, 'EnableArm'):
    print(inspect.getdoc(piper.EnableArm))
with open("/home/lab203/Desktop/piper_sdk/piper_sdk/interface/piper_interface_v2.py") as f:
    text = f.read()
import re
print("Enable methods in interface:")
for line in text.splitlines():
    if "def enable" in line.lower() or "def motionctrl" in line.lower():
        print("  ->", line.strip())
