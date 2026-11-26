# ===================================================
# arm_controller.py
# ===================================================

import rclpy as rp
from rclpy.node import Node
from geometry_msgs.msg import Pose
from kinematics_msgs.srv import SetRobotPose, GetRobotPose
from ros_robot_controller_msgs.msg import ServosPosition, ServoPosition

class ArmController(Node):
	def __init__(self):
		super().__init__('arm_controller')

		# Service Clients
		self.pose_target_cli = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
		self.get_cur_pose_cli = self.create_client(GetRobotPose, '/kinematics/get_current_pose')

		# wait for services
		for name, cli in [('set_pose_target', self.pose_target_cli), ('get_current_pose', self.get_cur_pose_cli)]:
			if not cli.wait_for_service(timeout_sec=1.0):
				self.get_logger().warn(f"Service '{name}' not available yet. Waiting...")

				if not cli.wait_for_service(timeout_sec=4.0):
					self.get_logger().error(f"Service '{name}' not available. Continuing but calls will fail.")

		# Publisher
		self.arm_pose_pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 10)

		self.cur_pos = Pose()

	def _publish_positions(self, pulses, duration: float):
		if len(pulses) != 5:
			self.get_logger().error(f"Need 5 pulses, got {len(pulses)}")
			return
		pos_list = []
		for i in range(5):
			p = int(pulses[i])
                        if not 0 <= p <= 1000:
                                raise ValueError(f"ID{i + 1} pulse {p} outside 0..1000")
			sp = ServoPosition()
			sp.id = i + 1
			sp.position = p
			pos_list.append(sp)

		msg = ServosPosition()
		msg.duration = float(duration)
                msg.position_unit = 'pulse'
		msg.position = pos_list
		self.arm_pose_pub.publish(msg)

	def call_set_pose_target(self, position: list, pitch: float, timeout: float = 2.0):
		req = SetRobotPose.Request()
		req.position = [float(position[0]), float(position[1]), float(position[2])]
		req.pitch = float(pitch)
		req.pitch_range = [-180.0, 180.0]
		req.resolution = 1.0

		future = self.pose_target_cli.call_async(req)
		rp.spin_until_future_complete(self, future, timeout_sec=timeout)

		if not future.done():
			self.get_logger().error('IK call timeout')
			return None

		resp = future.result()
		if not resp or not getattr(resp, 'success', False):
			self.get_logger().error('IK failed')
			return None
		if not hasattr(resp, 'pulse') or len(resp.pulse) < 5:
			self.get_logger().error('IK returned invalid pulses')
			return None
		return resp

	def call_get_current_pose(self, timeout: float = 2.0):
		req = GetRobotPose.Request()
		future = self.get_cur_pose_cli.call_async(req)
		rp.spin_until_future_complete(self, future, timeout_sec=timeout)

		if not future.done():
			self.get_logger().error('Get current pose timeout')
			return None

		resp = future.result()
		if not resp or not hasattr(resp, 'pose'):
			self.get_logger().error('Get current pose failed/invalid')
			return None
		return resp

	def send_pose_target(self, position: list, pitch: float, duration: float = 2.0):
		resp = self.call_set_pose_target(position, pitch)
		if not resp:
			return
		self._publish_positions(resp.pulse, duration)

	def send_joint_pulse_target(self, j1, j2, j3, j4, j5, duration: float = 2.0):
		self._publish_positions([j1, j2, j3, j4, j5], duration)

	def open_gripper(self, toggle: bool, duration: float, open_pulse: int = None, close_pulse: int = None):
		default_open = 800
		default_close = 350

		p_open = int(open_pulse if open_pulse is not None else default_open)
		p_close = int(close_pulse if close_pulse is not None else default_close)
		p = p_open if toggle else p_close
		if not 0 <= p <= 1000:
                        raise ValueError(f"Gripper pulse {p} outside 0..1000")

		grip = ServoPosition()
		grip.id = 10
		grip.position = p

		msg = ServosPosition()
		msg.duration = float(duration)
		msg.position_unit = 'pulse'
                msg.position = [grip]
		self.arm_pose_pub.publish(msg)

	def get_current_pose(self):
		resp = self.call_get_current_pose()
		if resp:
			self.cur_pos = resp.pose
		return self.cur_pos
