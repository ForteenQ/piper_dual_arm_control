with open('/home/lab203/Desktop/piper_sdk/piper_sdk/interface/piper_interface_v2.py') as f:
    t = f.read()
for fn in ['def ModeCtrl', 'def MotionCtrl_1', 'def MotionCtrl_2', 'def JointCtrl', 'def EnablePiper']:
    p = t.find(fn)
    if p != -1:
        print('=== ' + fn + ' ===')
        print(t[p:p+350])
