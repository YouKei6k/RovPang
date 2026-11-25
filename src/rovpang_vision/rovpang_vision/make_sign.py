#!/usr/bin/env python3
"""시험·인쇄용 표지판 그림 만들기: A = 빨간 삼각형, B = 파란 원

  ros2 run rovpang_vision make_sign            # sign_A.png, sign_B.png (600x600)
  ros2 run rovpang_vision make_sign --size 1200

휴대폰·모니터에 띄우거나 인쇄해서 sign_detector 시험에 쓴다.
"""
import argparse

import cv2

from rovpang_vision.sign_tools import make_sign_image


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--size', type=int, default=600, help='그림 한 변 픽셀 수')
    args, _ = p.parse_known_args()
    for label in ('A', 'B'):
        path = 'sign_{}.png'.format(label)
        cv2.imwrite(path, make_sign_image(label, args.size))
        print('{} 저장 ({}x{} 픽셀)'.format(path, args.size, args.size))


if __name__ == '__main__':
    main()
