try:
    import moveit_msgs
    print("moveit_msgs: OK")
except Exception as e:
    print(f"moveit_msgs: {e}")

try:
    import moveit
    print("moveit: OK")
except Exception as e:
    print(f"moveit: {e}")

try:
    import moveit_py
    print("moveit_py: OK")
except Exception as e:
    print(f"moveit_py: {e}")
