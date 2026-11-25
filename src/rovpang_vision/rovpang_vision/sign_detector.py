#!/usr/bin/env python3
"""RovPang 배송지 표지판 인식 노드 sign_detector (F-09 AI 배송지 인식)

카메라 컬러 영상에서 빨간 삼각형(A) / 파란 원(B) 표지판을 찾아 /rovpang/sign 으로 발행한다.
목적지와 맞는지 판단(5장 다수결)은 task_manager 가 한다.
  구독: /depth_cam/rgb/image_raw, /depth_cam/depth/image_raw(컬러에 정렬된 깊이), /depth_cam/rgb/camera_info
  발행: /rovpang/sign (rovpang_interfaces/Sign) - 표지판을 찾았을 때만 (label 'A' 또는 'B')
        + 표지판 위치 (11/25): 화면 가운데에서 벗어난 정도(offset_x), 각도(angle), 거리(distance), 3차원 위치
          -> 도착 후 정렬(팀원 2)에서 사용. 깊이를 못 재면 has_distance=false

  ros2 run rovpang_vision sign_detector
  ros2 run rovpang_vision sign_detector --ros-args -p min_area:=300 -p min_score:=0.7
"""
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from rovpang_interfaces.msg import DeliveryStatus, Sign
from rovpang_vision.image_utils import to_bgr
from rovpang_vision.object_pose import correct_depth, depth_to_meters, median_depth, pixel_to_point
from rovpang_vision.sign_tools import find_signs, sign_offset
from rovpang_vision.state_gate import StateGate


class SignDetector(Node):
    def __init__(self):
        super().__init__('sign_detector')
        self.declare_parameter('image_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('rate', 5.0)
        self.declare_parameter('min_area', 400)      # 이보다 작은 색 영역은 무시 (픽셀)
        self.declare_parameter('min_score', 0.75)    # 모양 점수 기준 (0~1)
        # 표지판 거리 (object_detector 와 같은 카메라·보정값)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter('info_topic', '/depth_cam/rgb/camera_info')
        self.declare_parameter('depth_radius', 5)        # 표지판 중심 주변 몇 픽셀로 깊이 중앙값을 낼지
        self.declare_parameter('max_depth_dt', 0.3)      # 컬러·깊이를 받은 시각 차가 이보다 크면 거리 생략(초)
        self.declare_parameter('depth_gain', 1.068)      # 11/24 자 측정 보정
        self.declare_parameter('depth_bias', 0.0088)
        # 필요한 배송 단계에서만 처리 (CPU 아끼기, state_gate.py)
        self.declare_parameter('gate_by_status', True)
        self.declare_parameter('active_states', ['VERIFYING'])
        self.gate = StateGate(self.get_parameter('active_states').value,
                              self.get_parameter('gate_by_status').value)
        self.create_subscription(DeliveryStatus, '/rovpang/status', lambda m: self.gate.update(m.state), 10)
        self.latest = None
        self.processed_stamp = None
        self.last_label = None
        self.latest_recv = 0.0
        self.latest_depth = None
        self.depth_recv = 0.0
        self.K = None                 # (fx, fy, cx, cy)
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)
        if self.get_parameter('use_depth').value:
            depth_topic = self.get_parameter('depth_topic').value
            qos = self.depth_qos(depth_topic)
            self.create_subscription(Image, depth_topic, self.on_depth, qos)
            self.get_logger().info('깊이 영상 구독: {} ({})'.format(
                depth_topic, 'reliable' if qos.reliability == ReliabilityPolicy.RELIABLE else 'best effort'))
            self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                     self.on_info, qos_profile_sensor_data)
        self.pub = self.create_publisher(Sign, '/rovpang/sign', 10)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)
        self.count = 0
        self.found = {'A': 0, 'B': 0}
        self.total_ms = 0.0
        self.create_timer(10.0, self.report)
        self.get_logger().info('표지판 인식 노드 시작 (A = 빨간 삼각형, B = 파란 원)')

    def depth_qos(self, topic):
        """카메라가 깊이를 reliable 로 보내면 reliable 로 받는다 (object_detector 와 같은 이유, 11/24)"""
        reliable = False
        for _ in range(20):
            infos = self.get_publishers_info_by_topic(topic)
            if infos:
                reliable = any(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
                break
            time.sleep(0.1)
        if reliable:
            return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        return qos_profile_sensor_data

    def on_image(self, msg):
        self.latest = msg
        self.latest_recv = time.time()

    def on_depth(self, msg):
        self.latest_depth = msg
        self.depth_recv = time.time()

    def on_info(self, msg):
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    def measure(self, color_msg, box):
        """표지판 중심의 깊이로 (거리 Z, (X, Y, Z)) 를 구한다. 못 재면 None 이고 이유를 self.why 에 남긴다"""
        d = self.latest_depth
        if d is None:
            self.why = '깊이 영상이 안 들어옴 (depth_topic 확인)'
            return None
        if self.K is None:
            self.why = '카메라 보정값 없음 (info_topic 확인)'
            return None
        dt = abs(self.latest_recv - self.depth_recv)
        if dt > self.get_parameter('max_depth_dt').value:
            self.why = '컬러·깊이 받은 시각 차 {:.2f}초'.format(dt)
            return None
        try:
            depth = depth_to_meters(d.encoding, d.height, d.width, d.step, d.data)
        except ValueError as e:
            self.why = str(e)
            return None
        sx, sy = d.width / color_msg.width, d.height / color_msg.height
        x, y, w, h = box
        u, v = int((x + w / 2.0) * sx), int((y + h / 2.0) * sy)
        r = int(self.get_parameter('depth_radius').value)
        z = median_depth(depth, u, v, r)
        if z is None:
            patch = depth[max(0, v - r):v + r + 1, max(0, u - r):u + r + 1]
            self.why = '표지판 중심 ({}, {}) 깊이 없음: 유효 {}/{}픽셀 (너무 가깝거나 화면·코팅 반사)'.format(
                u, v, int((patch > 0).sum()), patch.size)
            return None
        z = correct_depth(z, self.get_parameter('depth_gain').value, self.get_parameter('depth_bias').value)
        fx, fy, cx, cy = self.K
        return z, pixel_to_point(u, v, z, fx * sx, fy * sy, cx * sx, cy * sy)

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
        out.offset_x, out.angle = sign_offset((x, y, w, h), msg.width, self.K)
        self.why = '깊이 사용 안 함 (use_depth:=false)'
        m = self.measure(msg, (x, y, w, h)) if self.get_parameter('use_depth').value else None
        if m is not None:
            z, (X, Y, Z) = m
            out.has_distance = True
            out.distance = float(z)
            out.position.x, out.position.y, out.position.z = float(X), float(Y), float(Z)
        self.pub.publish(out)
        self.found[label] += 1
        if label != self.last_label:
            self.get_logger().info('표지판 {} (모양 점수 {:.2f}, 위치 ({}, {}) 크기 {}x{} 픽셀)'.format(
                label, score, x, y, w, h))
            self.last_label = label
        self.get_logger().info('표지판 {} 가운데에서 {:+.2f} ({:+.1f}도), 거리 {}'.format(
            label, out.offset_x, out.angle * 57.2958,
            '{:.3f} m'.format(out.distance) if out.has_distance else '측정 못함 - ' + self.why),
            throttle_duration_sec=1.0)

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
