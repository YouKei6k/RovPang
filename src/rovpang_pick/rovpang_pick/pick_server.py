import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rovpang_interfaces.action import Pick
from rovpang_pick.calibration import HandEyeCalibrator
from rovpang_pick.grasp_verifier import GraspVerifier
# Import according to the path of the existing ArmController implementation
from rovpang_arm.arm_controller import ArmController


class PickServer(Node):
    def __init__(self):
        super().__init__('rovpang_pick_server')
        self.cb_group = ReentrantCallbackGroup()

        self.arm = ArmController(self)
        self.calibrator = HandEyeCalibrator(self, target_frame='arm_base_link')
        self.verifier = GraspVerifier(closed_empty_threshold=550, open_threshold=250)

        self._action_server = ActionServer(
            self,
            Pick,
            '/rovpang/pick',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )
        self.get_logger().info('RovPang Pick Action Server ready (/rovpang/pick)')

    def goal_callback(self, goal_request):
        self.get_logger().info(f'Pick goal received: item={goal_request.item_class}')
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _publish_stage(self, goal_handle, stage_name: str):
        fb = Pick.Feedback()
        fb.stage = stage_name
        goal_handle.publish_feedback(fb)

    def execute_callback(self, goal_handle):
        target = goal_handle.request.target
        result = Pick.Result()
        max_attempts = 2  # 1 initial attempt + 1 retry on failure

        # 1. Camera coordinates -> arm-based coordinate transformation
        ok, (x, y, z) = self.calibrator.camera_to_arm(target)
        if not ok:
            result.success = False
            result.reason = 'TF_TRANSFORM_FAILED'
            result.attempts = 0
            goal_handle.abort()
            return result

        for attempt in range(1, max_attempts + 1):
            result.attempts = attempt
            self.get_logger().info(f'Pick attempt {attempt}/{max_attempts} -> ({x:.3f}, {y:.3f}, {z:.3f})')

            # 2. Top of the item (APPROACH)
            self._publish_stage(goal_handle, 'APPROACH')
            if not self.arm.send_pose_target(x, y, z + 0.06):
                result.success = False
                result.reason = 'IK_UNREACHABLE'
                goal_handle.abort()
                return result

            # 3. Open gripper -> Lower -> Close (GRASP)
            self._publish_stage(goal_handle, 'GRASP')
            self.arm.open_gripper()
            time.sleep(0.5)

            if not self.arm.send_pose_target(x, y, z):
                result.success = False
                result.reason = 'IK_UNREACHABLE'
                goal_handle.abort()
                return result

            self.arm.close_gripper()
            time.sleep(0.6)

            # 4. (LIFT)
            self._publish_stage(goal_handle, 'LIFT')
            self.arm.send_pose_target(x, y, z + 0.08)
            time.sleep(0.5)

            # 5. First Equipment Check (Gripper Servo Position)
            # Connect a method to the ArmController that reads the current gripper position
            gripper_pos = self.arm.get_gripper_position()
            if self.verifier.is_grasped(gripper_pos):
                result.success = True
                result.reason = ''
                goal_handle.succeed()
                return result

            self.get_logger().warn(f'Grasp verification failed on attempt {attempt} (pos={gripper_pos})')
            self.arm.open_gripper()
            time.sleep(0.4)

        result.success = False
        result.reason = 'PICK_FAILED'
        goal_handle.abort()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PickServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
