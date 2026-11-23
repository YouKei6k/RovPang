#!/usr/bin/env python3
"""RovPang 물품 탐지 노드 object_detector (F-02 AI 물품 인식)

카메라 컬러 영상을 YOLO(ONNX, onnxruntime CPU)로 탐지해서 /rovpang/detections 로 발행한다.
  구독: /depth_cam/rgb/image_raw (sensor_msgs/Image, rgb8/bgr8)
  발행: /rovpang/detections (rovpang_interfaces/DetectionArray)
        /rovpang/detections/image (sensor_msgs/Image, 상자를 그린 확인용 영상, publish_image=true 일 때)

Jetson Nano CPU에서 320 모델이 약 130ms/장이라, 들어오는 영상(약 28Hz)을 모두 처리하지 않고
rate(기본 3Hz)마다 가장 최근 영상 1장만 처리한다.
3차원 위치(has_position, position)는 아직 채우지 않는다 -> object_pose(F-03)에서 깊이로 계산 예정.

  ros2 run rovpang_vision object_detector
  ros2 run rovpang_vision object_detector --ros-args -p model_path:=/home/ubuntu/share/tmp/rovpang_models/best_640.onnx
"""
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from rovpang_interfaces.msg import Detection, DetectionArray
from rovpang_vision.image_utils import from_bgr, to_bgr
from rovpang_vision.yolo_onnx import YoloOnnx

DEFAULT_MODEL = '/home/ubuntu/share/tmp/rovpang_models/best_320.onnx'


def filter_large_boxes(results, width, height, max_ratio):
    """영상 넓이 대비 max_ratio 보다 큰 상자를 뺀다 (물품보다 훨씬 큰 모니터 등 오탐 제거)."""
    limit = max_ratio * width * height
    return [r for r in results if (r[4] - r[2]) * (r[5] - r[3]) <= limit]


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('model_path', DEFAULT_MODEL)
        self.declare_parameter('image_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('conf', 0.5)          # 이 값 이상만 발행 (task_manager는 0.8 이상만 사용)
        self.declare_parameter('rate', 3.0)          # 초당 처리 횟수
        self.declare_parameter('threads', 4)         # onnxruntime CPU 스레드 수
        self.declare_parameter('publish_image', True)
        # 영상 넓이의 이 비율보다 큰 상자는 버린다. 11/23 실시간 시험에서 모니터를 화면 대부분 크기
        # (0,0)-(518,359), 신뢰도 0.55~0.60 의 buds 로 잘못 잡음. 버즈는 30~50cm 에서 약 64x62 픽셀(2%).
        self.declare_parameter('max_box_ratio', 0.25)

        model = self.get_parameter('model_path').value
        self.detector = YoloOnnx(model, conf=self.get_parameter('conf').value,
                                 threads=int(self.get_parameter('threads').value))
        self.get_logger().info('모델 {} (입력 {}, 클래스 {})'.format(
            model, self.detector.size, list(self.detector.names.values())))

        self.latest = None        # 가장 최근 영상 메시지
        self.processed_stamp = None
        self.create_subscription(Image, self.get_parameter('image_topic').value, self.on_image,
                                 qos_profile_sensor_data)
        self.det_pub = self.create_publisher(DetectionArray, '/rovpang/detections', 10)
        self.img_pub = self.create_publisher(Image, '/rovpang/detections/image', 1)
        self.create_timer(1.0 / self.get_parameter('rate').value, self.tick)

        self.count = 0
        self.total_ms = 0.0
        self.create_timer(10.0, self.report)
        self.warned_encoding = False

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
            if not self.warned_encoding:
                self.get_logger().error(str(e))
                self.warned_encoding = True
            return

        t0 = time.time()
        results = filter_large_boxes(self.detector.detect(bgr), msg.width, msg.height,
                                     self.get_parameter('max_box_ratio').value)
        self.total_ms += (time.time() - t0) * 1000
        self.count += 1

        arr = DetectionArray()
        arr.header = msg.header        # 영상 시각, 카메라 좌표계 그대로
        for name, conf, x1, y1, x2, y2 in results:
            d = Detection()
            d.class_name = name
            d.confidence = float(conf)
            d.x_min, d.y_min, d.x_max, d.y_max = int(x1), int(y1), int(x2), int(y2)
            d.has_position = False     # object_pose(F-03)에서 채울 예정
            arr.detections.append(d)
        self.det_pub.publish(arr)

        if self.get_parameter('publish_image').value and self.img_pub.get_subscription_count() > 0:
            for name, conf, x1, y1, x2, y2 in results:
                cv2.rectangle(bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(bgr, '{} {:.2f}'.format(name, conf), (x1, max(15, y1 - 5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            out = Image()
            out.header = msg.header
            out.encoding, out.height, out.width, out.step, out.data = from_bgr(bgr)
            self.img_pub.publish(out)

    def report(self):
        if self.count:
            self.get_logger().info('최근 10초: {}장 처리, 평균 {:.0f} ms/장'.format(
                self.count, self.total_ms / self.count))
        else:
            self.get_logger().warn('영상이 들어오지 않습니다. 카메라 실행과 image_topic 을 확인하세요')
        self.count = 0
        self.total_ms = 0.0


def main():
    rclpy.init()
    node = ObjectDetector()
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
