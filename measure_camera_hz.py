import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

class CameraHzMeter(Node):
    def __init__(self):
        super().__init__('camera_hz_meter')
        self.color_cnt = 0
        self.depth_cnt = 0
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.color_cb, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_cb, 10)

    def color_cb(self, msg):
        self.color_cnt += 1

    def depth_cb(self, msg):
        self.depth_cnt += 1

def main():
    rclpy.init()
    node = CameraHzMeter()
    t0 = time.time()
    while time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    dt = time.time() - t0
    print(f"--> 实测频率: 彩色流(RGB)={node.color_cnt / dt:.1f} FPS | 深度流(Depth)={node.depth_cnt / dt:.1f} FPS (采样{dt:.1f}秒)")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
