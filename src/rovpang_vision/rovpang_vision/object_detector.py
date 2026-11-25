#!/usr/bin/env python3
"""RovPang 물품 탐지 노드 object_detector (F-02 AI 물품 인식)

카메라 컬러 영상을 YOLO(ONNX, onnxruntime CPU)로 탐지해서 /rovpang/detections 로 발행한다.
  구독: /depth_cam/rgb/image_raw (sensor_msgs/Image, rgb8/bgr8)
  발행: /rovpang/detections (rovpang_interfaces/DetectionArray)
        /rovpang/detections/image (sensor_msgs/Image, 상자를 그린 확인용 영상, publish_image=true 일 때)

Jetson Nano CPU에서 320 모델이 약 130ms/장이라, 들어오는 영상(약 28Hz)을 모두 처리하지 않고
rate(기본 3Hz)마다 가장 최근 영상 1장만 처리한다.
3차원 위치(F-03): 상자 중심 주변 깊이의 중앙값과 카메라 보정값으로 카메라 기준 (X, Y, Z)를 계산해
has_position, position 을 채운다 (object_pose.py). 카메라는 depth_registration:=true 로 켜야 한다.
  추가 구독: /depth_cam/depth/image_raw (16UC1, mm), /depth_cam/rgb/camera_info (fx, fy, cx, cy)

  ros2 run rovpang_vision object_detector
  ros2 run rovpang_vision object_detector --ros-args -p model_path:=/home/ubuntu/share/tmp/rovpang_models/best_640.onnx
"""
import time

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from rovpang_interfaces.msg import Detection, DetectionArray
from rovpang_vision.image_utils import from_bgr, to_bgr
from rovpang_vision.object_pose import depth_to_meters, locate
from rovpang_vision.yolo_onnx import YoloOnnx

DEFAULT_MODEL = '/home/ubuntu/share/tmp/rovpang_models/best_320.onnx'


def filter_large_boxes(results, width, height, max_ratio):
    """영상 넓이 대비 max_ratio 보다 큰 상자를 뺀다 (물품보다 훨씬 큰 모니터 등 오탐 제거)."""
    limit = max_ratio * width * height
    return [r for r in results if (r[4] - r[2]) * (r[5] - r[3]) <= limit]


def filter_by_size(results, positions, min_m, max_m):
    """위치를 계산한 탐지 중 실제 크기(가로·세로 중 큰 쪽)가 범위 밖인 것을 뺀다. 위치가 없는 탐지는 그대로 둔다."""
    keep_r, keep_p = [], []
    for r, p in zip(results, positions):
        if p is not None:
            size = max(p[1])
            if size < min_m or size > max_m:
                continue
        keep_r.append(r)
        keep_p.append(p)
    return keep_r, keep_p


class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detector')
        self.declare_parameter('model_path', DEFAULT_MODEL)
        self.declare_parameter('image_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('conf', 0.5)          # 이 값 이상만 발행 (task_manager는 0.8 이상만 사용)
        self.declare_parameter('rate', 3.0)          # 초당 처리 횟수
        # onnxruntime CPU 스레드 수. 4개를 다 쓰면 큰 깊이 영상을 받을 틈이 없어 깊이가 끊겼다(11/24) -> 3개
        self.declare_parameter('threads', 3)
        self.declare_parameter('publish_image', True)
        # 영상 넓이의 이 비율보다 큰 상자는 버린다. 11/23 실시간 시험에서 모니터를 화면 대부분 크기
        # (0,0)-(518,359), 신뢰도 0.55~0.60 의 buds 로 잘못 잡음. 버즈는 30~50cm 에서 약 64x62 픽셀(2%).
        self.declare_parameter('max_box_ratio', 0.25)
        # 3차원 위치 (F-03)
        self.declare_parameter('use_depth', True)
        self.declare_parameter('depth_topic', '/depth_cam/depth/image_raw')
        self.declare_parameter('info_topic', '/depth_cam/rgb/camera_info')   # 정렬된 깊이 = 컬러 카메라 보정값
        self.declare_parameter('depth_radius', 5)        # 중심 주변 11x11 픽셀의 중앙값
        self.declare_parameter('max_depth_dt', 0.2)      # 컬러·깊이 영상 시각 차 허용(초)
        # 실제 크기 검사(F-03 4번): 계산한 가로·세로 중 큰 쪽이 이 범위 밖이면 그 물품이 아니라고 보고 버린다.
        # 버즈 케이스 약 5~6cm. 11/24 시험에서 10~14cm 상자(신뢰도 0.5~0.6)가 buds 로 잡혀 걸러냄.
        # 깊이 보정 (11/24 자 측정 30/50/80cm -> 32.9/54.3/86.3cm): 실제 = (측정 - bias) / gain
        self.declare_parameter('depth_gain', 1.068)
        self.declare_parameter('depth_bias', 0.0088)
        self.declare_parameter('min_size_cm', 3.0)
        self.declare_parameter('max_size_cm', 9.0)

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

        self.latest_depth = None
        self.latest_recv = 0.0
        self.depth_recv = 0.0
        self.K = None                 # (fx, fy, cx, cy)
        self.warned = set()
        if self.get_parameter('use_depth').value:
            depth_topic = self.get_parameter('depth_topic').value
            self.create_subscription(Image, depth_topic, self.on_depth, self.depth_qos(depth_topic))
            self.create_subscription(CameraInfo, self.get_parameter('info_topic').value,
                                     self.on_info, qos_profile_sensor_data)

    def depth_qos(self, topic):
        """깊이 영상 구독 QoS. 카메라가 reliable 로 보내면 reliable 로 받는다.
        best effort 로 받으면 CPU가 바쁠 때 큰 깊이 영상(약 460KB)이 조각째 버려져 계속 끊겼다(11/24)."""
        reliable = False
        for _ in range(20):               # 카메라 노드를 찾을 때까지 최대 약 2초
            infos = self.get_publishers_info_by_topic(topic)
            if infos:
                reliable = any(i.qos_profile.reliability == ReliabilityPolicy.RELIABLE for i in infos)
                break
            time.sleep(0.1)
        self.get_logger().info('깊이 영상 구독: {} ({})'.format(topic, 'reliable' if reliable else 'best effort'))
        if reliable:
            return QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE)
        return qos_profile_sensor_data

    def warn_once(self, key, text):
        if key not in self.warned:
            self.warned.add(key)
            self.get_logger().warn(text)

    def on_depth(self, msg):
        self.latest_depth = msg
        self.depth_recv = time.time()

    def on_info(self, msg):
        if self.K is None:
            self.K = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])
            self.get_logger().info('카메라 보정값 fx={:.1f} fy={:.1f} cx={:.1f} cy={:.1f} ({}x{})'.format(
                *self.K, msg.width, msg.height))

    def compute_positions(self, color_msg, results):
        """탐지마다 3차원 위치를 계산한다. 반환: 결과와 같은 순서의 ((X,Y,Z), (가로,세로)) 또는 None"""
        none = [None] * len(results)
        dmsg = self.latest_depth
        if not results or dmsg is None or self.K is None:
            if dmsg is None:
                self.warn_once('nodepth', '깊이 영상이 없습니다 (depth_topic 확인). 위치 없이 발행합니다')
            elif self.K is None:
                self.warn_once('noinfo', '카메라 보정값이 없습니다 (info_topic 확인). 위치 없이 발행합니다')
            return none
        if 'color' not in dmsg.header.frame_id:
            self.warn_once('noreg', '깊이 영상 frame_id={} : 컬러에 정렬되지 않았습니다. '
                           'depth_registration:=true 로 카메라를 켜세요'.format(dmsg.header.frame_id))
        # 시각 차는 '받은 시각'으로 비교한다. 11/24 시험에서 깊이 영상 header 시각이 컬러와 19초 어긋나
        # (카메라 드라이버가 깊이에 다른 시계를 씀) header 로 비교하면 모든 프레임이 버려졌다.
        dt = abs(self.latest_recv - self.depth_recv)
        if dt > self.get_parameter('max_depth_dt').value:
            self.get_logger().warn('컬러·깊이 영상을 받은 시각 차가 큽니다 ({:.2f}초). 깊이 영상이 끊겼는지 확인하세요'.format(dt),
                                   throttle_duration_sec=5.0)
            return none
        hdt = abs((color_msg.header.stamp.sec - dmsg.header.stamp.sec)
                  + (color_msg.header.stamp.nanosec - dmsg.header.stamp.nanosec) * 1e-9)
        if hdt > 1.0:
            self.warn_once('hdt', '참고: 컬러·깊이 header 시각이 {:.1f}초 다릅니다 (받은 시각으로 비교함)'.format(hdt))
        try:
            depth = depth_to_meters(dmsg.encoding, dmsg.height, dmsg.width, dmsg.step, dmsg.data)
        except ValueError as e:
            self.warn_once('denc', str(e))
            return none
        sx, sy = dmsg.width / color_msg.width, dmsg.height / color_msg.height   # 크기가 다르면 비율로 맞춤
        fx, fy, cx, cy = self.K
        K = (fx * sx, fy * sy, cx * sx, cy * sy)
        radius = int(self.get_parameter('depth_radius').value)
        out = []
        for _, _, x1, y1, x2, y2 in results:
            box = (int(x1 * sx), int(y1 * sy), int(x2 * sx), int(y2 * sy))
            out.append(locate(box, depth, K, radius,
                              self.get_parameter('depth_gain').value, self.get_parameter('depth_bias').value))
        return out

    def on_image(self, msg):
        self.latest = msg
        self.latest_recv = time.time()

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

        positions = (self.compute_positions(msg, results) if self.get_parameter('use_depth').value
                     else [None] * len(results))
        results, positions = filter_by_size(results, positions,
                                            self.get_parameter('min_size_cm').value / 100.0,
                                            self.get_parameter('max_size_cm').value / 100.0)

        arr = DetectionArray()
        arr.header = msg.header        # 영상 시각, 카메라 좌표계 그대로
        for (name, conf, x1, y1, x2, y2), pos in zip(results, positions):
            d = Detection()
            d.class_name = name
            d.confidence = float(conf)
            d.x_min, d.y_min, d.x_max, d.y_max = int(x1), int(y1), int(x2), int(y2)
            d.has_position = pos is not None
            if pos is not None:
                (X, Y, Z), (bw, bh) = pos
                d.position.x, d.position.y, d.position.z = float(X), float(Y), float(Z)
                self.get_logger().info('{} {:.2f} 위치 X={:+.3f} Y={:+.3f} Z={:.3f} m, 크기 약 {:.1f}x{:.1f} cm'.format(
                    name, conf, X, Y, Z, bw * 100, bh * 100), throttle_duration_sec=1.0)
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
