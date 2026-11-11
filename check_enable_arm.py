with open("/home/lab203/Desktop/piper_sdk/piper_sdk/interface/piper_interface_v2.py") as f:
    text = f.read()

pos = text.find("def EnableArm")
if pos != -1:
    print(text[pos:pos+600])

pos2 = text.find("def DisableArm")
if pos2 != -1:
    print(text[pos2:pos2+600])
