import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rovpang_interfaces.action import Place
from rovpang_pick.calibration import HandEyeCalibrator
from rovpang_arm.arm_controller import ArmController


class PlaceServer(Node):
    def __init__(self):
        super().__init__('rovpang_place_server')
        self.cb_group = ReentrantCallbackGroup()

        # Fixed base coordinates for the lifting platform (arm-based coordinate system, adjusted after measurement)
        self.declare_parameter('default_place_xyz', [0.20, 0.0, 0.05])

        self.arm = ArmController(self)
        self.calibrator = HandEyeCalibrator(self, target_frame='arm_base_link')

        self._action_server = ActionServer(
            self,
            Place,
            '/rovpang/place',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.cb_group,
        )
        self.get_logger().info('RovPang Place Action Server ready (/rovpang/place)')

    def goal_callback(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        return CancelResponse.ACCEPT

    def _publish_stage(self, goal_handle, stage_name: str):
        fb = Place.Feedback()
        fb.stage = stage_name
        goal_handle.publish_feedback(fb)

    def execute_callback(self, goal_handle):
        req = goal_handle.request
        result = Place.Result()

        if req.use_default:
            xyz = self.get_parameter('default_place_xyz').get_parameter_value().double_array_value
            x, y, z = float(xyz[0]), float(xyz[1]), float(xyz[2])
        else:
            ok, (x, y, z) = self.calibrator.camera_to_arm(req.target)
            if not ok:
                result.success = False
                result.reason = 'TF_TRANSFORM_FAILED'
                goal_handle.abort()
                return result

        # 1. Approach above the loading platform
        self._publish_stage(goal_handle, 'APPROACH')
        if not self.arm.send_pose_target(x, y, z + 0.06):
            result.success = False
            result.reason = 'PLACE_FAILED'
            goal_handle.abort()
            return result

        # 2. Descend and gripper open
        self._publish_stage(goal_handle, 'RELEASE')
        if not self.arm.send_pose_target(x, y, z):
            result.success = False
            result.reason = 'PLACE_FAILED'
            goal_handle.abort()
            return result

        self.arm.open_gripper()
        time.sleep(0.5)

        # 3. Arm retraction (camera view clearance -> F-12 assist with re-detection of the unloading area)
        self._publish_stage(goal_handle, 'RETRACT')
        self.arm.send_pose_target(x, y, z + 0.08)
        time.sleep(0.4)

        result.success = True
        result.reason = ''
        goal_handle.succeed()
        return result


def main(args=None):
    rclpy.init(args=args)
    node = PlaceServer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
