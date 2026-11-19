#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
깊이 영상의 한 점(기본: 화면 중앙)까지의 거리를 1초에 한 번 출력한다.
도커 안(ROS2 Humble)에서 깊이 카메라를 켠 상태로 실행한다.

  python3 depth_center.py                 # 화면 중앙
  python3 depth_center.py --x 100 --y 200 # 원하는 픽셀
"""
import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class DepthCenter(Node):
    def __init__(self, topic, x, y):
        super().__init__("depth_center")
        self.x, self.y = x, y
        self.last = 0.0
        self.create_subscription(Image, topic, self.cb, qos_profile_sensor_data)
        self.get_logger().info("구독: {}".format(topic))

    def cb(self, msg):
        now = self.get_clock().now().nanoseconds / 1e9
        if now - self.last < 1.0:
            return
        self.last = now

        if msg.encoding not in ("16UC1", "mono16"):
            self.get_logger().warn("예상과 다른 인코딩: {}".format(msg.encoding))
            return

        img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
        x = msg.width // 2 if self.x < 0 else self.x
        y = msg.height // 2 if self.y < 0 else self.y

        # 한 픽셀은 값이 비어(0) 있을 수 있어 주변 5x5의 0이 아닌 값 중앙값을 쓴다
        patch = img[max(0, y - 2):y + 3, max(0, x - 2):x + 3]
        valid = patch[patch > 0]
        if valid.size == 0:
            print("({}, {}) 거리 측정 불가 (값 없음)  해상도 {}x{}".format(x, y, msg.width, msg.height))
        else:
            mm = float(np.median(valid))
            print("({}, {}) 거리 {:.0f} mm  ({:.2f} m)  해상도 {}x{}".format(
                x, y, mm, mm / 1000.0, msg.width, msg.height))


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--topic", default="/depth_cam/depth/image_raw")
    p.add_argument("--x", type=int, default=-1)
    p.add_argument("--y", type=int, default=-1)
    args = p.parse_args()

    rclpy.init()
    node = DepthCenter(args.topic, args.x, args.y)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
