#!/usr/bin/env python3
"""RovPang QR 인식 노드 qr_detector (F-10 QR 인증의 '읽기' 부분)

카메라 컬러 영상에서 QR을 읽어 그 문자열을 /rovpang/qr (std_msgs/String) 로 발행한다.
인증 판단(이 배송의 토큰인지, 재사용인지)은 task_manager 의 recipient_auth 가 한다.
  구독: /depth_cam/rgb/image_raw
  발행: /rovpang/qr (std_msgs/String) - QR을 읽을 때마다 (못 읽으면 발행하지 않음)

  ros2 run rovpang_vision qr_detector
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

from rovpang_vision.image_utils import to_bgr
from rovpang_vision.qr_tools import QrReader


class QrDetector(Node):
    def __init__(self):
        super().__init__('qr_detector')
        self.declare_parameter('image_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('rate', 5.0)       # 초당 확인 횟수
        self.reader = QrReader()
        self.latest = None
        self.processed_stamp = None
        self.last_text = None
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(String, '/rovpang/qr', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.count = 0
        self.total_ms = 0.0
        self.found = 0
        self.create_timer(10.0, self.report)
        self.get_logger().info('QR 인식 노드 시작')

    def on_image(self, msg):
        self.latest = msg

    def tick(self):
        msg = self.latest
        if msg is None or msg.header.stamp == self.processed_stamp:
            return
        self.processed_stamp = msg.header.stamp
        try:
            bgr = to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        t0 = time.time()
        results = self.reader.decode(bgr)
        self.total_ms += (time.time() - t0) * 1000
        self.count += 1
        for text, _corners in results:
            self.pub.publish(String(data=text))
            self.found += 1
            if text != self.last_text:            # 새 QR일 때만 로그
                self.get_logger().info('QR 읽음: {}'.format(text))
                self.last_text = text

    def report(self):
        if self.count:
            self.get_logger().info('최근 10초: {}장 확인, QR {}번 읽음, 평균 {:.0f} ms/장'.format(
                self.count, self.found, self.total_ms / self.count))
        else:
            self.get_logger().warn('영상이 들어오지 않습니다. 카메라 실행과 image_topic 을 확인하세요')
        self.count = 0
        self.found = 0
        self.total_ms = 0.0


def main():
    rclpy.init()
    node = QrDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
