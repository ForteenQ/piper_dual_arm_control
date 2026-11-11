import os
p = '/home/lab203/Desktop/piper_ros-humble/src/piper_moveit/piper_with_gripper_moveit/config'
for f in os.listdir(p):
    print(f)
    if 'controller' in f or 'ros2_control' in f or 'moveit' in f:
        print('--- ' + f + ' ---')
        with open(os.path.join(p, f)) as fc:
            print(fc.read()[:500])
