#!/usr/bin/env python3
"""RovPang 미션 관리 노드 (F-14 배송 상태 관리) - 골격 버전

배송 요청(/rovpang/delivery_request)을 받으면 아래 순서로 팀원 2의 액션·서비스를 호출한다.

  DETECTING : 팔 LOOK 자세 -> /rovpang/detections 에서 요청 물품 찾기
  PICKING   : /rovpang/pick
  MOVING    : 팔 CARRY 자세 -> /rovpang/go_to (목적지)
  VERIFYING : 팔 LOOK 자세 -> /rovpang/sign 다수결로 목적지 확인 -> /rovpang/qr 토큰 확인
  PLACING   : /rovpang/place
  RETURNING : 팔 HOME 자세 -> /rovpang/go_to START
  DONE

단계가 바뀔 때마다 /rovpang/status 를 발행하고 ~/rovpang_logs/deliveries.csv 에 기록한다.
/rovpang/cancel (std_srvs/Trigger) 로 진행 중인 배송을 취소할 수 있다.
"""
import csv
import os
import threading
import time
from collections import Counter

import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from std_srvs.srv import Trigger

from rovpang_interfaces.action import GoTo, Pick, Place
from rovpang_interfaces.msg import DeliveryRequest, DeliveryStatus, DetectionArray, Sign
from rovpang_interfaces.srv import SetArmPose

S = DeliveryStatus


class MissionFailed(Exception):
    def __init__(self, code, message=''):
        super().__init__(message)
        self.code = code
        self.message = message


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        # 파라미터 (값은 초안, 요구사항 정의서와 맞춤)
        self.declare_parameter('confidence_min', 0.8)
        self.declare_parameter('detect_timeout', 10.0)
        self.declare_parameter('sign_frames', 5)
        self.declare_parameter('sign_timeout', 10.0)
        self.declare_parameter('qr_timeout', 30.0)
        self.declare_parameter('action_timeout', 60.0)
        self.declare_parameter('log_dir', os.path.expanduser('~/rovpang_logs'))

        cb = ReentrantCallbackGroup()
        self.status_pub = self.create_publisher(DeliveryStatus, '/rovpang/status', 10)
        self.create_subscription(DeliveryRequest, '/rovpang/delivery_request', self.on_request, 10,
                                 callback_group=cb)
        self.create_subscription(DetectionArray, '/rovpang/detections', self.on_detections, 10,
                                 callback_group=cb)
        self.create_subscription(Sign, '/rovpang/sign', self.on_sign, 10, callback_group=cb)
        self.create_subscription(String, '/rovpang/qr', self.on_qr, 10, callback_group=cb)
        self.create_service(Trigger, '/rovpang/cancel', self.on_cancel, callback_group=cb)

        self.pick_client = ActionClient(self, Pick, '/rovpang/pick', callback_group=cb)
        self.place_client = ActionClient(self, Place, '/rovpang/place', callback_group=cb)
        self.goto_client = ActionClient(self, GoTo, '/rovpang/go_to', callback_group=cb)
        self.arm_client = self.create_client(SetArmPose, '/rovpang/set_arm_pose', callback_group=cb)

        self.lock = threading.Lock()
        self.state = S.STATE_IDLE
        self.request = None
        self.cancel_event = threading.Event()
        self.active_goal = None
        self.used_tokens = set()

        self.last_detections = None
        self.sign_buffer = None      # 수집 중일 때만 list
        self.last_qr = None          # (문자열, 받은 시각)

        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        self.log_path = os.path.join(log_dir, 'deliveries.csv')
        if not os.path.exists(self.log_path):
            with open(self.log_path, 'w', newline='') as f:
                csv.writer(f).writerow(['time', 'delivery_id', 'state', 'error_code', 'message'])

        self.set_state(S.STATE_IDLE, '', '대기 중')
        self.get_logger().info('미션 관리 노드 시작 (로그: {})'.format(self.log_path))

    # ---------- 상태 발행·기록 ----------
    def set_state(self, state, error_code='', message=''):
        self.state = state
        msg = DeliveryStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.delivery_id = self.request.delivery_id if self.request else ''
        msg.state = state
        msg.error_code = error_code
        msg.message = message
        self.status_pub.publish(msg)
        with open(self.log_path, 'a', newline='') as f:
            csv.writer(f).writerow([time.strftime('%Y-%m-%d %H:%M:%S'), msg.delivery_id,
                                    state, error_code, message])
        self.get_logger().info('[{}] {} {}'.format(state, error_code, message).strip())

    # ---------- 구독 콜백 ----------
    def on_request(self, req):
        with self.lock:
            if self.state not in (S.STATE_IDLE, S.STATE_DONE, S.STATE_FAILED, S.STATE_CANCELED):
                self.get_logger().warn('진행 중인 배송이 있어 요청 {} 를 거절합니다'.format(req.delivery_id))
                return
            if req.destination not in (GoTo.Goal.DEST_A, GoTo.Goal.DEST_B):
                self.get_logger().warn('알 수 없는 목적지: {}'.format(req.destination))
                return
            self.request = req
            self.cancel_event.clear()
        threading.Thread(target=self.run_mission, args=(req,), daemon=True).start()

    def on_detections(self, msg):
        self.last_detections = msg

    def on_sign(self, msg):
        if self.sign_buffer is not None and msg.label:
            self.sign_buffer.append(msg.label)

    def on_qr(self, msg):
        self.last_qr = (msg.data, time.time())

    def on_cancel(self, request, response):
        if self.state in (S.STATE_IDLE, S.STATE_DONE, S.STATE_FAILED, S.STATE_CANCELED):
            response.success = False
            response.message = '진행 중인 배송이 없습니다'
            return response
        self.cancel_event.set()
        if self.active_goal is not None:
            self.active_goal.cancel_goal_async()
        response.success = True
        response.message = '취소 요청을 보냈습니다'
        return response

    # ---------- 기다리기 도우미 ----------
    def check_cancel(self):
        if self.cancel_event.is_set():
            raise MissionFailed('CANCELED', '관리자 취소')

    def wait_future(self, future, timeout):
        t0 = time.time()
        while not future.done():
            self.check_cancel()
            if time.time() - t0 > timeout:
                return None
            time.sleep(0.05)
        return future.result()

    def call_action(self, client, goal, name):
        if not client.wait_for_server(timeout_sec=5.0):
            raise MissionFailed('SERVER_UNAVAILABLE', '{} 액션 서버 없음'.format(name))
        handle = self.wait_future(client.send_goal_async(goal), 10.0)
        if handle is None or not handle.accepted:
            raise MissionFailed('GOAL_REJECTED', '{} 목표 거절'.format(name))
        self.active_goal = handle
        try:
            res = self.wait_future(handle.get_result_async(),
                                   self.get_parameter('action_timeout').value)
        finally:
            self.active_goal = None
        if res is None:
            handle.cancel_goal_async()
            raise MissionFailed('TIMEOUT', '{} 응답 시간 초과'.format(name))
        return res.result

    def set_arm(self, pose):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            raise MissionFailed('SERVER_UNAVAILABLE', 'set_arm_pose 서비스 없음')
        req = SetArmPose.Request()
        req.pose = pose
        res = self.wait_future(self.arm_client.call_async(req), 15.0)
        if res is None or not res.success:
            raise MissionFailed('ARM_POSE_FAILED', '팔 자세 {} 실패'.format(pose))

    # ---------- 단계 ----------
    def detect_item(self, item_class):
        conf_min = self.get_parameter('confidence_min').value
        t0 = time.time()
        while time.time() - t0 < self.get_parameter('detect_timeout').value:
            self.check_cancel()
            msg = self.last_detections
            if msg is not None:
                cands = [d for d in msg.detections
                         if d.class_name == item_class and d.has_position and d.confidence >= conf_min]
                if cands:
                    best = max(cands, key=lambda d: d.confidence)
                    target = PointStamped()
                    target.header = msg.header
                    target.point = best.position
                    return target, best.confidence
            time.sleep(0.1)
        raise MissionFailed('ITEM_NOT_FOUND', '{} 탐지 실패'.format(item_class))

    def verify_sign(self, destination):
        need = self.get_parameter('sign_frames').value
        self.sign_buffer = []
        t0 = time.time()
        try:
            while len(self.sign_buffer) < need:
                self.check_cancel()
                if time.time() - t0 > self.get_parameter('sign_timeout').value:
                    raise MissionFailed('SIGN_NOT_FOUND', '표지판 인식 실패')
                time.sleep(0.1)
            label, count = Counter(self.sign_buffer).most_common(1)[0]
        finally:
            self.sign_buffer = None
        if label != destination:
            raise MissionFailed('WRONG_DESTINATION',
                                '요청 {} / 인식 {} ({}/{})'.format(destination, label, count, need))
        return label

    def verify_qr(self, token):
        t_start = time.time()
        while time.time() - t_start < self.get_parameter('qr_timeout').value:
            self.check_cancel()
            if self.last_qr is not None and self.last_qr[1] >= t_start:
                data = self.last_qr[0]
                if data in self.used_tokens:
                    raise MissionFailed('QR_REUSED', '이미 사용한 QR')
                if data != token:
                    raise MissionFailed('QR_INVALID', '다른 배송의 QR')
                self.used_tokens.add(data)
                return
            time.sleep(0.1)
        raise MissionFailed('QR_TIMEOUT', 'QR 인식 시간 초과')

    # ---------- 전체 흐름 ----------
    def run_mission(self, req):
        left_start = False
        delivered = False
        try:
            self.set_state(S.STATE_DETECTING, '', '물품 {} 탐색'.format(req.item_class))
            self.set_arm(SetArmPose.Request.POSE_LOOK)
            target, conf = self.detect_item(req.item_class)

            self.set_state(S.STATE_PICKING, '', '신뢰도 {:.2f}, 위치 ({:.2f}, {:.2f}, {:.2f})'.format(
                conf, target.point.x, target.point.y, target.point.z))
            goal = Pick.Goal()
            goal.target = target
            goal.item_class = req.item_class
            result = self.call_action(self.pick_client, goal, 'pick')
            if not result.success:
                raise MissionFailed('PICK_FAILED', result.reason)

            self.set_arm(SetArmPose.Request.POSE_CARRY)
            self.set_state(S.STATE_MOVING, '', '목적지 {}'.format(req.destination))
            goal = GoTo.Goal()
            goal.destination = req.destination
            left_start = True
            result = self.call_action(self.goto_client, goal, 'go_to')
            if not result.success:
                raise MissionFailed('NAV_FAILED', result.reason)

            self.set_state(S.STATE_VERIFYING, '', '표지판·QR 확인')
            self.set_arm(SetArmPose.Request.POSE_LOOK)
            self.verify_sign(req.destination)
            self.verify_qr(req.qr_token)

            self.set_state(S.STATE_PLACING, '', '하역')
            goal = Place.Goal()
            goal.use_default = True
            result = self.call_action(self.place_client, goal, 'place')
            if not result.success:
                raise MissionFailed('PLACE_FAILED', result.reason)
            delivered = True   # 하역까지 끝나면 이후 문제는 '배송 완료'로 기록

            self.set_state(S.STATE_RETURNING, '', '출발지로 복귀')
            self.set_arm(SetArmPose.Request.POSE_HOME)
            goal = GoTo.Goal()
            goal.destination = GoTo.Goal.DEST_START
            result = self.call_action(self.goto_client, goal, 'go_to')
            if not result.success:
                raise MissionFailed('RETURN_FAILED', result.reason)
            left_start = False

            self.set_state(S.STATE_DONE, '', '배송 완료')
        except MissionFailed as e:
            if delivered:
                # 하역은 끝났으므로 배송 자체는 성공. 복귀 중 문제만 오류 코드로 남김
                code = 'RETURN_CANCELED' if e.code == 'CANCELED' else e.code
                self.set_state(S.STATE_DONE, code, '배송 완료, 복귀 중 문제: ' + e.message)
                self.try_return_home()
            elif e.code == 'CANCELED':
                self.set_state(S.STATE_CANCELED, e.code, e.message)
            else:
                self.set_state(S.STATE_FAILED, e.code, e.message)
            if left_start:
                self.try_return_home()
        except Exception as e:  # 예상하지 못한 오류도 기록
            self.set_state(S.STATE_FAILED, 'INTERNAL_ERROR', str(e))

    def try_return_home(self):
        """실패·취소 후 출발지 복귀 시도 (실패해도 상태만 기록)."""
        self.cancel_event.clear()
        try:
            goal = GoTo.Goal()
            goal.destination = GoTo.Goal.DEST_START
            result = self.call_action(self.goto_client, goal, 'go_to')
            msg = '출발지 복귀 {}'.format('성공' if result.success else '실패')
        except MissionFailed as e:
            msg = '출발지 복귀 실패: {}'.format(e.message)
        self.get_logger().info(msg)


def main():
    rclpy.init()
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
