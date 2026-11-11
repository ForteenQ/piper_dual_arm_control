with open("/home/lab203/Desktop/piper_sdk/piper_sdk/interface/piper_interface_v2.py") as f:
    text = f.read()

pos = text.find("def __del__")
if pos != -1:
    print(text[pos:pos+500])
