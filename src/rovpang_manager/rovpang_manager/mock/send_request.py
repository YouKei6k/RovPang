#!/usr/bin/env python3
"""시험용 배송 요청 보내기 (관리자 프로그램 대신)

  ros2 run rovpang_manager send_request --dest A --item buds
  ros2 run rovpang_manager send_request --dest A --item buds --token rovpang-test-01
      (--token: 미리 만든 QR(make_qr)과 같은 토큰으로 실제 QR 인증 시험. 없으면 무작위 토큰)
"""
import argparse
import secrets
import time

import rclpy

from rovpang_interfaces.msg import DeliveryRequest


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--dest', default='A', choices=['A', 'B'])
    p.add_argument('--item', default='buds')
    p.add_argument('--token', default=None, help='QR 토큰 직접 지정 (기본: 무작위)')
    args, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = rclpy.create_node('send_request')
    pub = node.create_publisher(DeliveryRequest, '/rovpang/delivery_request', 10)

    # 미션 관리 노드가 구독할 때까지 최대 3초 기다림
    t0 = time.time()
    while pub.get_subscription_count() == 0 and time.time() - t0 < 3.0:
        rclpy.spin_once(node, timeout_sec=0.1)

    msg = DeliveryRequest()
    msg.delivery_id = time.strftime('D%m%d-%H%M%S')
    msg.item_class = args.item
    msg.destination = args.dest
    msg.qr_token = args.token or secrets.token_hex(4)
    pub.publish(msg)
    print('배송 요청 보냄: id={} 물품={} 목적지={} QR토큰={}'.format(
        msg.delivery_id, msg.item_class, msg.destination, msg.qr_token))
    time.sleep(0.5)

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
