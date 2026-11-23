import math
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rovpang_interfaces.action import GoTo


class GoToServer(Node):
    def __init__(self):
        super().__init__('rovpang_go_to_server')
        self.cb_group = ReentrantCallbackGroup()

        self.declare_parameter('waypoints.START', [0.0, 0.0, 0.0])
        self.declare_parameter('waypoints.A', [1.5, 0.8, 1.57])
        self.declare_parameter('waypoints.B', [2.2, -0.6, -1.57])

        self._valid_dests = {
            GoTo.Goal.DEST_START,
            GoTo.Goal.DEST_A,
            GoTo.Goal.DEST_B,
        }

        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose', callback_group=self.cb_group
        )
        self._action_server = ActionServer(
            self,
            GoTo,
            '/rovpang/go_to',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )
        self.get_logger().info('RovPang GoTo Action Server ready (/rovpang/go_to)')

    def goal_callback(self, goal_request):
        if goal_request.destination not in self._valid_dests:
            self.get_logger().warn(f'Unknown destination: {goal_request.destination}')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Cancel request received')
        return CancelResponse.ACCEPT

    def _make_pose(self, dest: str) -> PoseStamped:
        coords = self.get_parameter(f'waypoints.{dest}').get_parameter_value().double_array_value
        x, y, yaw = float(coords[0]), float(coords[1]), float(coords[2])

        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def execute_callback(self, goal_handle):
        dest = goal_handle.request.destination
        result = GoTo.Result()

        if not self._nav_client.wait_for_server(timeout_sec=5.0):
            result.success = False
            result.reason = 'NAV2_SERVER_UNAVAILABLE'
            goal_handle.abort()
            return result

        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self._make_pose(dest)

        def nav_feedback_cb(feedback_msg):
            fb = GoTo.Feedback()
            fb.distance_remaining = float(feedback_msg.feedback.distance_remaining)
            goal_handle.publish_feedback(fb)

        send_future = self._nav_client.send_goal_async(nav_goal, feedback_callback=nav_feedback_cb)
        while not send_future.done():
            time.sleep(0.05)

        nav_goal_handle = send_future.result()
        if not nav_goal_handle.accepted:
            result.success = False
            result.reason = 'NAV2_GOAL_REJECTED'
            goal_handle.abort()
            return result

        result_future = nav_goal_handle.get_result_async()

        while not result_future.done():
            if goal_handle.is_cancel_requested:
                # 11/19 주의사항: Nav2 목표를 먼저 취소해 정지시킨 뒤 canceled 종료
                cancel_future = nav_goal_handle.cancel_goal_async()
                while not cancel_future.done():
                    time.sleep(0.05)
                result.success = False
                result.reason = 'CANCELED'
                goal_handle.canceled()
                return result
            time.sleep(0.1)

        nav_status = result_future.result().status
        if nav_status == GoalStatus.STATUS_SUCCEEDED:
            result.success = True
            result.reason = ''
            goal_handle.succeed()
        else:
            result.success = False
            result.reason = f'NAV_FAILED (status={nav_status})'
            goal_handle.abort()

        return result


def main(args=None):
    rclpy.init(args=args)
    node = GoToServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
