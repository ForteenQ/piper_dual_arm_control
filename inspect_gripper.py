import inspect
from piper_sdk import C_PiperInterface_V2

piper = C_PiperInterface_V2("can0")
print("=== GripperCtrl Doc ===")
print(inspect.getdoc(piper.GripperCtrl))
print("=== GripperCtrl Signature ===")
print(inspect.signature(piper.GripperCtrl))
