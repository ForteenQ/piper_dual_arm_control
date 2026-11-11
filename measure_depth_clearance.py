#!/usr/bin/env python3
import time
import numpy as np
import rclpy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rclpy.init()
    node = rclpy.create_node('depth_clearance_probe')
    br = CvBridge()
    d = None

    def cb(msg):
        nonlocal d
        d = br.imgmsg_to_cv2(msg, desired_encoding='passthrough').astype(np.float32)

    node.create_subscription(Image, '/camera/camera/aligned_depth_to_color/image_raw', cb, 1)
    t0 = time.time()
    while d is None and time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.05)

    if d is None:
        print("No depth image")
        return

    table = np.median(d[200:350, 450:600][d[200:350, 450:600] > 100])
    bracket = d[170:240, 410:445]
    vb = bracket[(bracket > 150) & (bracket < table - 30)]
    bracket_d = np.median(vb) if len(vb) > 0 else 0

    finger_area = d[215:260, 260:330]
    vf = finger_area[(finger_area > 150) & (finger_area < table - 20)]

    print(f"Table depth: {table:.1f} mm")
    print(f"Dabai bracket depth: {bracket_d:.1f} mm (height above table: {table - bracket_d:.1f} mm)")
    if len(vf) > 0:
        print(f"Finger area depth: min={vf.min():.1f}, median={np.median(vf):.1f}, max={vf.max():.1f} mm")
        print(f"Finger height above table: median={table - np.median(vf):.1f} mm, lowest={table - vf.max():.1f} mm")
    else:
        print("No valid finger depth pixels")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
