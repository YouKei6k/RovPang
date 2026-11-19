#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
RovPang 학습 데이터 수집 스크립트 (ROS2 토픽 버전)

깊이 카메라를 켜 둔 상태에서, 도커 안(ROS2 Humble, ubuntu 계정)에서 실행한다.
  ros2 launch peripherals depth_camera.launch.py depth_registration:=true

YOLO가 실제로 받게 될 토픽(/depth_cam/rgb/image_raw)에서 그대로 사진을 저장하므로
학습 사진과 실제 동작 영상의 색감·화질이 같다.
--with-depth 를 붙이면 같은 순간의 정렬된 깊이 영상도 16비트 PNG로 함께 저장한다.

사용 예)
  python3 capture_ros.py --label box                    # Enter = 1장, q + Enter = 종료
  python3 capture_ros.py --label sign_A --interval 1 --count 100
  python3 capture_ros.py --label box --with-depth       # 위치 추정 시험용
"""
import argparse
import os
import select
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# 도커의 /home/ubuntu/share/tmp 는 로봇 본체의 /home/hiwonder/docker/tmp 와 같은 폴더
DEFAULT_OUT = "/home/ubuntu/share/tmp/rovpang_dataset"


def to_bgr(msg):
    """sensor_msgs/Image(rgb8/bgr8) -> OpenCV BGR 배열 (cv_bridge 없이)."""
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = img[:, :msg.width * 3].reshape(msg.height, msg.width, 3)
    if msg.encoding == "rgb8":
        return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if msg.encoding == "bgr8":
        return img.copy()
    raise ValueError("지원하지 않는 컬러 인코딩: {}".format(msg.encoding))


def to_depth(msg):
    """sensor_msgs/Image(16UC1) -> uint16 배열 (단위 mm)."""
    img = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.step // 2)
    return img[:, :msg.width].copy()


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


class Capture(Node):
    def __init__(self, args):
        super().__init__("rovpang_capture")
        self.args = args
        self.folder = os.path.join(args.out, args.label)
        os.makedirs(self.folder, exist_ok=True)
        self.idx = self.next_index()
        self.saved = 0
        self.rgb = None
        self.depth = None
        self.last_auto = 0.0
        self.done = False

        self.create_subscription(Image, args.rgb_topic, self.on_rgb, qos_profile_sensor_data)
        if args.with_depth:
            self.create_subscription(Image, args.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_timer(0.05, self.tick)

        print("저장 폴더: {}".format(self.folder))
        if args.interval > 0:
            print("{}초마다 자동 저장. 멈추려면 q + Enter (또는 Ctrl+C)".format(args.interval))
        else:
            print("Enter = 1장 저장, q + Enter = 종료")

    def next_index(self):
        nums = []
        for name in os.listdir(self.folder):
            if name.startswith(self.args.label + "_") and name.endswith(".jpg"):
                try:
                    nums.append(int(name[len(self.args.label) + 1:].split("_")[0]))
                except ValueError:
                    pass
        return max(nums) + 1 if nums else 0

    def on_rgb(self, msg):
        self.rgb = msg

    def on_depth(self, msg):
        self.depth = msg

    def tick(self):
        want = False
        if select.select([sys.stdin], [], [], 0)[0]:
            line = sys.stdin.readline().strip().lower()
            if line == "q":
                self.done = True
                return
            if self.args.interval <= 0:
                want = True

        now = time.time()
        if self.args.interval > 0 and now - self.last_auto >= self.args.interval:
            want = True
            self.last_auto = now

        if want:
            self.save()

    def save(self):
        if self.rgb is None:
            print("아직 컬러 영상을 받지 못했습니다. 카메라가 켜져 있는지 확인하세요.")
            return
        base = "{}_{:04d}".format(self.args.label, self.idx)
        bgr = to_bgr(self.rgb)
        cv2.imwrite(os.path.join(self.folder, base + ".jpg"), bgr)
        msg = "저장: {}.jpg ({}x{})".format(base, bgr.shape[1], bgr.shape[0])

        if self.args.with_depth:
            if self.depth is None:
                msg += "  [깊이 없음]"
            else:
                gap = abs(stamp(self.rgb) - stamp(self.depth))
                cv2.imwrite(os.path.join(self.folder, base + "_depth.png"), to_depth(self.depth))
                msg += "  + 깊이 (시각 차 {:.3f}s)".format(gap)
                if gap > 0.05:
                    msg += "  [주의: 컬러와 깊이 시각 차가 큼]"
        print(msg)

        self.idx += 1
        self.saved += 1
        if self.args.count and self.saved >= self.args.count:
            print("{}장 저장 완료".format(self.saved))
            self.done = True


def main():
    p = argparse.ArgumentParser(description="RovPang dataset capture (ROS2)")
    p.add_argument("--label", required=True, help="대상 이름 (box, sign_A, sign_B, background ...)")
    p.add_argument("--out", default=DEFAULT_OUT)
    p.add_argument("--interval", type=float, default=0.0, help="자동 저장 간격(초), 0 = 수동")
    p.add_argument("--count", type=int, default=0, help="이 장수만큼 저장하면 종료, 0 = 제한 없음")
    p.add_argument("--with-depth", action="store_true", help="정렬된 깊이 영상도 함께 저장")
    p.add_argument("--rgb-topic", default="/depth_cam/rgb/image_raw")
    p.add_argument("--depth-topic", default="/depth_cam/depth/image_raw")
    args = p.parse_args()

    rclpy.init()
    node = Capture(args)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        print("이번에 저장한 사진: {}장 -> {}".format(node.saved, node.folder))
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
