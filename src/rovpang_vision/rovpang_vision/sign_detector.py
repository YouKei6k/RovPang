#!/usr/bin/env python3
"""RovPang 배송지 표지판 인식 노드 sign_detector (F-09 AI 배송지 인식)

카메라 컬러 영상에서 빨간 삼각형(A) / 파란 원(B) 표지판을 찾아 /rovpang/sign 으로 발행한다.
목적지와 맞는지 판단(5장 다수결)은 task_manager 가 한다.
  구독: /depth_cam/rgb/image_raw
  발행: /rovpang/sign (rovpang_interfaces/Sign) - 표지판을 찾았을 때만 (label 'A' 또는 'B')

  ros2 run rovpang_vision sign_detector
  ros2 run rovpang_vision sign_detector --ros-args -p min_area:=300 -p min_score:=0.7
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rovpang_interfaces.msg import DeliveryStatus, Sign
from rovpang_vision.image_utils import to_bgr
from rovpang_vision.sign_tools import find_signs
from rovpang_vision.state_gate import StateGate


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        self.declare_parameter('image_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('min_area', 400)      # 이보다 작은 색 영역은 무시 (픽셀)
        self.declare_parameter('min_score', 0.75)    # 모양 점수 기준 (0~1)
        # 필요한 배송 단계에서만 처리 (CPU 아끼기, state_gate.py)
        self.declare_parameter('gate_by_status', True)
        self.declare_parameter('active_states', ['VERIFYING'])
        self.gate = StateGate(self.get_parameter('active_states').value,
                              self.get_parameter('gate_by_status').value)
        self.create_subscription(DeliveryStatus, '/rovpang/status', lambda m: self.gate.update(m.state), 10)
        self.latest = None
        self.processed_stamp = None
        self.last_label = None
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)
        self.pub = self.create_publisher(Sign, '/rovpang/sign', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.count = 0
        self.found = {'A': 0, 'B': 0}
        self.total_ms = 0.0
        self.create_timer(10.0, self.report)
        self.get_logger().info('표지판 인식 노드 시작 (A = 빨간 삼각형, B = 파란 원)')

    def on_image(self, msg):
        self.latest = msg

    def tick(self):
        msg = self.latest
        if msg is None or msg.header.stamp == self.processed_stamp or not self.gate.active():
            return
        self.processed_stamp = msg.header.stamp
        try:
            bgr = to_bgr(msg.encoding, msg.height, msg.width, msg.step, msg.data)
        except ValueError as e:
            self.get_logger().error(str(e), throttle_duration_sec=10.0)
            return
        t0 = time.time()
        found = find_signs(bgr, int(self.get_parameter('min_area').value),
                           float(self.get_parameter('min_score').value))
        self.total_ms += (time.time() - t0) * 1000
        self.count += 1
        if not found:
            return
        label, score, (x, y, w, h), area = found[0]      # 가장 큰 것
        out = Sign()
        out.header = msg.header
        out.label = label
        out.confidence = float(score)
        self.pub.publish(out)
        self.found[label] += 1
        if label != self.last_label:
            self.get_logger().info('표지판 {} (모양 점수 {:.2f}, 위치 ({}, {}) 크기 {}x{} 픽셀)'.format(
                label, score, x, y, w, h))
            self.last_label = label

    def report(self):
        if not self.count and not self.gate.active():
            self.get_logger().info('쉬는 중 (배송 상태 {}, 처리 단계 아님)'.format(self.gate.state))
        elif self.count:
            self.get_logger().info('최근 10초: {}장 확인, A {}번 / B {}번, 평균 {:.0f} ms/장'.format(
                self.count, self.found['A'], self.found['B'], self.total_ms / self.count))
        else:
            self.get_logger().warn('영상이 들어오지 않습니다. 카메라 실행과 image_topic 을 확인하세요')
        self.count = 0
        self.found = {'A': 0, 'B': 0}
        self.total_ms = 0.0


def main():
    rclpy.init()
    node = SignDetector()
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
