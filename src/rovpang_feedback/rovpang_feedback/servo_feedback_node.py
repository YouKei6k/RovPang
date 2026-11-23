#!/usr/bin/env python3
"""Read real JetRover bus-servo positions without commanding motor movement.

Publishes a complete sweep of IDs 1-5 and 10 as ServoStateList to
/rovpang/servo_states. Invalid, missing or timed-out samples are discarded;
this node never substitutes a default position for a hardware reading.
"""

import time

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState
from servo_controller_msgs.msg import ServoState, ServoStateList


class ServoFeedbackNode(Node):
    SERVO_IDS = (1, 2, 3, 4, 5, 10)
    SERVICE = '/ros_robot_controller/bus_servo/get_state'
    TOPIC = '/rovpang/servo_states'
    REQUEST_TIMEOUT_SEC = 2.0
    MAX_SWEEP_SEC = 8.0
    RETRY_DELAY_SEC = 1.0
    IDLE_AFTER_SWEEP_SEC = 0.5

    def __init__(self):
        super().__init__('rovpang_servo_feedback')
        self.client = self.create_client(GetBusServoState, self.SERVICE)
        self.publisher = self.create_publisher(ServoStateList, self.TOPIC, 10)

        self._index = 0
        self._values = {}
        self._sweep_started = None
        self._future = None
        self._pending_id = None
        self._request_started = 0.0
        self._next_attempt = 0.0
        self._last_warning = 0.0
        self._sweep_count = 0
        self.timer = self.create_timer(0.1, self._tick)

        self.get_logger().info(
            f'Read-only feedback enabled: {self.SERVICE} -> {self.TOPIC}; '
            f'servo IDs {list(self.SERVO_IDS)}'
        )

    def _warn(self, message):
        now = time.monotonic()
        if now - self._last_warning >= 5.0 or self._last_warning == 0.0:
            self.get_logger().warn(message)
            self._last_warning = now

    def _discard_sweep(self, reason):
        self._warn(f'Discarding incomplete feedback sweep: {reason}')
        self._future = None
        self._pending_id = None
        self._index = 0
        self._values = {}
        self._sweep_started = None
        self._next_attempt = time.monotonic() + self.RETRY_DELAY_SEC

    @staticmethod
    def _read_position(response, requested_id):
        if response is None or not response.success:
            return None
        for state in response.state:
            ids = list(state.present_id)
            positions = list(state.position)
            if len(ids) != len(positions):
                continue
            for servo_id, position in zip(ids, positions):
                position = int(position)
                if int(servo_id) == requested_id and 0 <= position <= 1000:
                    return position
        return None

    def _publish_complete_sweep(self):
        if any(servo_id not in self._values for servo_id in self.SERVO_IDS):
            self._discard_sweep('some servo IDs were not read')
            return

        msg = ServoStateList()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_footprint'

        for servo_id in self.SERVO_IDS:
            state = ServoState()
            state.id = servo_id
            state.position = self._values[servo_id]
            # Goal, error and voltage are not measured by this node.
            # Their message defaults are not valid hardware measurements.
            msg.servo_state.append(state)

        self.publisher.publish(msg)
        self._sweep_count += 1
        if self._sweep_count == 1 or self._sweep_count % 10 == 0:
            positions = [self._values[i] for i in self.SERVO_IDS]
            self.get_logger().info(f'Hardware positions {list(self.SERVO_IDS)}: {positions}')

        self._index = 0
        self._values = {}
        self._sweep_started = None
        self._next_attempt = time.monotonic() + self.IDLE_AFTER_SWEEP_SEC

    def _tick(self):
        now = time.monotonic()

        if self._future is not None:
            if self._future.done():
                future = self._future
                requested_id = self._pending_id
                self._future = None
                self._pending_id = None
                try:
                    position = self._read_position(future.result(), requested_id)
                except Exception as exc:
                    self._discard_sweep(f'service request for ID {requested_id}: {exc}')
                    return
                if position is None:
                    self._discard_sweep(f'invalid response for servo ID {requested_id}')
                    return
                self._values[requested_id] = position
                self._index += 1
                if self._index == len(self.SERVO_IDS):
                    self._publish_complete_sweep()
                return

            if now - self._request_started > self.REQUEST_TIMEOUT_SEC:
                # Drop the unanswered request so it cannot pollute a later sweep.
                try:
                    self.client.remove_pending_request(self._future)
                except Exception:
                    pass
                self._discard_sweep(f'timeout reading servo ID {self._pending_id}')
            return

        if now < self._next_attempt:
            return

        if not self.client.service_is_ready():
            self._warn(f'Waiting for service {self.SERVICE}')
            self._next_attempt = now + self.RETRY_DELAY_SEC
            return

        if self._sweep_started is None:
            self._sweep_started = now
        elif now - self._sweep_started > self.MAX_SWEEP_SEC:
            self._discard_sweep('sweep exceeded maximum age')
            return

        servo_id = self.SERVO_IDS[self._index]
        request = GetBusServoState.Request()
        request.cmd = [GetBusServoCmd(id=servo_id, get_id=1, get_position=1)]
        try:
            self._future = self.client.call_async(request)
        except Exception as exc:
            self._discard_sweep(f'could not request servo ID {servo_id}: {exc}')
            return
        self._pending_id = servo_id
        self._request_started = now


def main(args=None):
    rclpy.init(args=args)
    node = ServoFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
