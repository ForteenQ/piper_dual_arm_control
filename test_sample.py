#!/usr/bin/env python3
import rclpy
import easy_handeye2_msgs.srv as srv

rclpy.init()
node = rclpy.create_node('test_client')
client = node.create_client(srv.TakeSample, '/easy_handeye2/calibration/take_sample')

if client.wait_for_service(timeout_sec=3.0):
    future = client.call_async(srv.TakeSample.Request())
    rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
    res = future.result()
    if res:
        print(f"TAKE_SAMPLE_SUCCESS: samples count = {len(res.samples.samples)}")
    else:
        print("CALL_FAILED")
else:
    print("SERVICE_NOT_FOUND")

node.destroy_node()
rclpy.shutdown()
