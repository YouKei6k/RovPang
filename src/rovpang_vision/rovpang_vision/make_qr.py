#!/usr/bin/env python3
"""시험용 QR 그림 만들기 (관리자 프로그램이 배송마다 QR을 보여 주는 것을 대신)

  ros2 run rovpang_vision make_qr --text rovpang-test-01             # -> qr_rovpang-test-01.png
  ros2 run rovpang_vision make_qr --text rovpang-test-01 --out a.png

만든 그림을 PC로 가져와 휴대폰·모니터에 띄우거나 인쇄해서 카메라에 보여 준다.
같은 토큰으로 send_request --token rovpang-test-01 을 보내면 인증 성공을 시험할 수 있다.
"""
import argparse

import cv2

from rovpang_vision.qr_tools import make_image


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--text', required=True, help='QR에 넣을 토큰 문자열')
    p.add_argument('--out', default=None)
    p.add_argument('--size', type=int, default=10, help='QR 한 칸의 픽셀 수')
    args, _ = p.parse_known_args()
    out = args.out or 'qr_{}.png'.format(args.text)
    img = make_image(args.text, module_px=args.size)
    cv2.imwrite(out, img)
    print('{} 저장 ({}x{} 픽셀, 내용: {})'.format(out, img.shape[1], img.shape[0], args.text))


if __name__ == '__main__':
    main()
