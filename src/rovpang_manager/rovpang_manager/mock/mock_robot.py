#!/usr/bin/env python3
"""팀원 2 기능을 흉내 내는 가짜 로봇 노드 (실제 로봇은 움직이지 않음)

/rovpang/pick, /rovpang/place, /rovpang/go_to 액션과 /rovpang/set_arm_pose 서비스를 제공하고,
delay 초 뒤에 성공을 돌려준다. fail_on 파라미터로 특정 기능을 실패시킬 수 있다.
  fail_on: '' (모두 성공) / 'pick' / 'place' / 'go_to' / 'arm'
           / 'arm_away' (출발지가 아닌 곳에서만 팔 자세 실패 -> 복귀 전 멈춤 시험)
  실행 중에 ros2 param set /mock_robot fail_on none 으로 고장을 '고칠' 수 있다.
  pick_fail_times: 처음 N번의 pick만 실패 (1이면 첫 집기 실패 -> 재시도 성공 시험)
  pick_fail_reason: pick 이 실패할 때 돌려줄 이유. OUT_OF_RANGE 로 두면 '팔 반경 밖' 시험 (재시도 안 함)
  팔 자세는 SetArmPose 에 정의된 이름만 받는다. 모르는 이름이면 실패를 돌려준다.
  serve: 흉내 낼 기능 목록. 실제 노드가 있는 기능은 빼서 겹치지 않게 한다
         예) ['pick', 'place', 'arm'] -> go_to 는 실제 goto_server 사용
"""
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from rovpang_interfaces.action import GoTo, Pick, Place
from rovpang_interfaces.srv import SetArmPose


class MockRobot(Node):
    def __init__(self):
        super().__init__('mock_robot')
        self.declare_parameter('delay', 2.0)
        self.declare_parameter('fail_on', '')
        self.declare_parameter('pick_fail_times', 0)
        self.declare_parameter('pick_fail_reason', 'mock: 그리퍼 파지 실패')
        self.declare_parameter('serve', ['pick', 'place', 'go_to', 'arm'])
        # SetArmPose 의 POSE_* 상수에서 받을 수 있는 자세 이름을 모은다
        self.known_poses = {getattr(SetArmPose.Request, n) for n in dir(SetArmPose.Request)
                            if n.startswith('POSE_')}
        self.pick_count = 0
        self.location = GoTo.Goal.DEST_START   # 가짜 로봇의 현재 위치
        cb = ReentrantCallbackGroup()
        common = dict(callback_group=cb,
                      goal_callback=lambda g: GoalResponse.ACCEPT,
                      cancel_callback=lambda g: CancelResponse.ACCEPT)
        serve = set(self.get_parameter('serve').value)
        if 'pick' in serve:
            ActionServer(self, Pick, '/rovpang/pick', self.exec_pick, **common)
        if 'place' in serve:
            ActionServer(self, Place, '/rovpang/place', self.exec_place, **common)
        if 'go_to' in serve:
            ActionServer(self, GoTo, '/rovpang/go_to', self.exec_goto, **common)
        if 'arm' in serve:
            self.create_service(SetArmPose, '/rovpang/set_arm_pose', self.on_arm, callback_group=cb)
        self.get_logger().info('가짜 로봇 준비 완료 (흉내: {}, fail_on="{}")'.format(
            ', '.join(sorted(serve)), self.get_parameter('fail_on').value))

    def should_fail(self, name):
        return self.get_parameter('fail_on').value == name

    def run_steps(self, goal_handle, feedback, stages):
        """stages: [(피드백 설정 함수), ...] 를 delay 동안 나눠 실행. 취소되면 False."""
        delay = self.get_parameter('delay').value
        for setter in stages:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return False
            setter(feedback)
            goal_handle.publish_feedback(feedback)
            time.sleep(delay / len(stages))
        return True

    def exec_pick(self, goal_handle):
        g = goal_handle.request
        self.get_logger().info('[pick] {} at ({:.2f}, {:.2f}, {:.2f}) frame={}'.format(
            g.item_class, g.target.point.x, g.target.point.y, g.target.point.z, g.target.header.frame_id))
        fb = Pick.Feedback()
        stages = [lambda f, s=s: setattr(f, 'stage', s) for s in ('APPROACH', 'GRASP', 'LIFT')]
        result = Pick.Result()
        if not self.run_steps(goal_handle, fb, stages):
            result.success, result.reason = False, 'canceled'
            return result
        result.attempts = 1
        self.pick_count += 1
        if self.should_fail('pick') or self.pick_count <= self.get_parameter('pick_fail_times').value:
            result.success, result.reason = False, self.get_parameter('pick_fail_reason').value
            goal_handle.abort()
        else:
            result.success, result.reason = True, ''
            goal_handle.succeed()
        return result

    def exec_place(self, goal_handle):
        self.get_logger().info('[place] use_default={}'.format(goal_handle.request.use_default))
        fb = Place.Feedback()
        stages = [lambda f, s=s: setattr(f, 'stage', s) for s in ('ALIGN', 'RELEASE', 'RETRACT')]
        result = Place.Result()
        if not self.run_steps(goal_handle, fb, stages):
            result.success, result.reason = False, 'canceled'
            return result
        if self.should_fail('place'):
            result.success, result.reason = False, 'mock: 하역 위치 정렬 실패'
            goal_handle.abort()
        else:
            result.success, result.reason = True, ''
            goal_handle.succeed()
        return result

    def exec_goto(self, goal_handle):
        dest = goal_handle.request.destination
        self.get_logger().info('[go_to] {}'.format(dest))
        fb = GoTo.Feedback()
        stages = [lambda f, d=d: setattr(f, 'distance_remaining', d) for d in (2.0, 1.0, 0.0)]
        result = GoTo.Result()
        if not self.run_steps(goal_handle, fb, stages):
            result.success, result.reason = False, 'canceled'
            return result
        if self.should_fail('go_to'):
            result.success, result.reason = False, 'mock: 경로 막힘'
            goal_handle.abort()
        else:
            result.success, result.reason = True, ''
            self.location = dest
            goal_handle.succeed()
        return result

    def on_arm(self, request, response):
        self.get_logger().info('[set_arm_pose] {}'.format(request.pose))
        time.sleep(0.5)
        away = self.location != GoTo.Goal.DEST_START
        if request.pose not in self.known_poses:
            self.get_logger().warn('모르는 팔 자세: {} (가능: {})'.format(
                request.pose, ', '.join(sorted(self.known_poses))))
            response.success, response.message = False, 'mock: 모르는 자세 {}'.format(request.pose)
        elif self.should_fail('arm') or (self.should_fail('arm_away') and away):
            response.success, response.message = False, 'mock: 팔 자세 실패'
        else:
            response.success, response.message = True, 'ok'
        return response


def main():
    rclpy.init()
    node = MockRobot()
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
