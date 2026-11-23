#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RovPang JetRover arm IK test (ROS 2 Humble).

Dry-run by default. This file uses the existing ArmController as the ROS 2
service/publisher adapter, and deliberately avoids its unverified motor helpers.
No movement command is published unless --move is set and MOVE is confirmed.
"""

import argparse
import math
import time

import rclpy
from servo_controller_msgs.msg import ServoStateList
from ros_robot_controller_msgs.msg import GetBusServoCmd, ServoPosition, ServosPosition
from ros_robot_controller_msgs.srv import GetBusServoState
from kinematics_msgs.srv import SetRobotPose
from rovpang_arm.arm_controller import ArmController

FEEDBACK_TOPIC = '/rovpang/servo_states'
COMMAND_TOPIC = '/ros_robot_controller/bus_servo/set_position'
MAX_CARTESIAN_STEP_M = 0.003   # 3 mm maximum for first move
MAX_PULSE_STEP = 5             # 5 pulses maximum per joint for first move
FEEDBACK_TOLERANCE = 3         # measured and IK reference agreement
MAX_FEEDBACK_AGE_SEC = 4.0    # refuse old ROS feedback
MIN_COMMAND_PULSE = 30        # conservative first-test margin, not a calibrated joint limit
MAX_COMMAND_PULSE = 970


def get_live_pulses(node, timeout=6.0, since=None):
    """Return fresh ID 1..5 feedback, or None. No motor commands."""
    latest = {'pulses': None, 'time': None}

    def on_state(msg):
        stamp = msg.header.stamp
        stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
        age_sec = (node.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if stamp_ns == 0 or not -1.0 <= age_sec <= MAX_FEEDBACK_AGE_SEC:
            return
        states = {int(s.id): int(s.position) for s in msg.servo_state}
        if all(i in states for i in range(1, 6)):
            latest['pulses'] = [states[i] for i in range(1, 6)]
            latest['time'] = time.monotonic()

    sub = node.create_subscription(ServoStateList, FEEDBACK_TOPIC, on_state, 10)
    try:
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            if latest['pulses'] is not None and (since is None or latest['time'] >= since):
                return latest['pulses']
        return None
    finally:
        node.destroy_subscription(sub)



def get_hardware_pulses(node, timeout_per_id=2.5):
    """Read IDs 1..5 directly, independently of the ROS feedback topic.

    Uses the already-running manufacturer's read-only GetBusServoState service.
    Never publishes a motor command.
    """
    client = node.create_client(GetBusServoState, '/ros_robot_controller/bus_servo/get_state')
    try:
        if not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('Direct bus-servo hardware state service is unavailable.')
        values = []
        for servo_id in range(1, 6):
            request = GetBusServoState.Request()
            request.cmd = [GetBusServoCmd(id=servo_id, get_id=1, get_position=1)]
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_per_id)
            if not future.done():
                raise RuntimeError('Direct hardware reading timed out for servo ID{}.'.format(servo_id))
            try:
                response = future.result()
            except Exception as exc:
                raise RuntimeError('Direct hardware read exception for servo ID{}: {}'.format(servo_id, exc)) from exc
            if response is None or not response.success:
                raise RuntimeError('Direct hardware read failed for servo ID{}.'.format(servo_id))
            reading = None
            for state in response.state:
                if len(state.present_id) != len(state.position):
                    continue
                for found_id, position in zip(state.present_id, state.position):
                    if int(found_id) == servo_id and 0 <= int(position) <= 1000:
                        reading = int(position)
                        break
                if reading is not None:
                    break
            if reading is None:
                raise RuntimeError('Invalid hardware response for servo ID{}.'.format(servo_id))
            values.append(reading)
        return values
    finally:
        node.destroy_client(client)


def compare_feedback(topic_values, hardware_values, label):
    """Fail closed if ROS topic and independently read hardware disagree."""
    errors = [abs(a - b) for a, b in zip(topic_values, hardware_values)]
    if len(topic_values) != 5 or len(hardware_values) != 5 or any(e > FEEDBACK_TOLERANCE for e in errors):
        raise RuntimeError('{}: topic {} and hardware {} differ too much.'.format(
            label, topic_values, hardware_values))


def read_current_pose(node):
    response = node.call_get_current_pose(timeout=5.0)
    if response is None or not bool(getattr(response, 'success', False)) \
            or not bool(getattr(response, 'solution', False)):
        raise RuntimeError('Failed to read current XYZ: check the IK node and servo feedback.')
    p = response.pose.position
    xyz = [float(p.x), float(p.y), float(p.z)]
    if not all(math.isfinite(v) for v in xyz):
        raise RuntimeError('Current XYZ contains invalid values.')
    return xyz


def parse_args():
    parser = argparse.ArgumentParser(description='RovPang XYZ arm test (dry-run by default)')
    parser.add_argument('--status', action='store_true', help='Show current XYZ and servo feedback')
    parser.add_argument('--x', type=float)
    parser.add_argument('--y', type=float)
    parser.add_argument('--z', type=float)
    parser.add_argument('--dx', type=float)
    parser.add_argument('--dy', type=float)
    parser.add_argument('--dz', type=float)
    parser.add_argument('--pitch', type=float, default=None, help='Target pitch (degrees); does not automatically preserve current pitch')
    parser.add_argument('--move', action='store_true', help='Enable real movement after safety checks and explicit confirmation')
    return parser.parse_args()


def target_from_args(args, current):
    absolute = [args.x, args.y, args.z]
    relative = [args.dx, args.dy, args.dz]
    has_abs = any(v is not None for v in absolute)
    has_rel = any(v is not None for v in relative)
    if has_abs and has_rel:
        raise ValueError('Do not mix absolute coordinates (--x --y --z) with relative coordinates (--dx, etc.).')
    if has_abs:
        if any(v is None for v in absolute):
            raise ValueError('Absolute coordinates require all three options: --x, --y, and --z.')
        return [float(v) for v in absolute]
    if has_rel:
        return [current[i] + float(relative[i] or 0.0) for i in range(3)]
    return None


def compute_narrow_pitch_ik(node, target, pitch, timeout=15.0):
    """Request IK with a narrow pitch window rather than wrapper's +/-180 range."""
    request = SetRobotPose.Request()
    request.position = [float(v) for v in target]
    request.pitch = float(pitch)
    request.pitch_range = [max(-180.0, pitch - 2.0), min(180.0, pitch + 2.0)]
    request.resolution = 1.0
    future = node.pose_target_cli.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError('IK service request timed out.')
    response = future.result()
    if response is None:
        raise RuntimeError('No response from the IK service.')
    return response


def validate_plan(current, target, actual, response):
    issues = []
    target_pulses = list(getattr(response, 'pulse', []))
    ik_current = list(getattr(response, 'current_pulse', []))
    if not getattr(response, 'success', False) or len(target_pulses) != 5 or len(ik_current) != 5:
        return ['IK solution or five current servo values are missing.'], None
    if not all(math.isfinite(float(v)) for v in target_pulses + ik_current):
        return ['IK servo values contain NaN or infinity.'], None
    targets = [int(round(float(v))) for v in target_pulses]
    ik_currents = [int(round(float(v))) for v in ik_current]
    dist = math.dist(current, target)
    if dist > MAX_CARTESIAN_STEP_M + 1e-9:
        issues.append('Target is {:.1f} mm from current XYZ, exceeding the 3 mm limit.'.format(dist * 1000))
    for i, (now, ik_now, goal) in enumerate(zip(actual, ik_currents, targets), start=1):
        if not MIN_COMMAND_PULSE <= goal <= MAX_COMMAND_PULSE:
            issues.append('ID{}: target {} outside conservative first-test {}..{} pulse margin.'.format(
                i, goal, MIN_COMMAND_PULSE, MAX_COMMAND_PULSE))
        if abs(now - ik_now) > FEEDBACK_TOLERANCE:
            issues.append('ID{}: live feedback {} differs from IK reference value {}.'.format(i, now, ik_now))
        if abs(goal - now) > MAX_PULSE_STEP:
            issues.append('ID{}: pulse change {} exceeds the first-test limit of 5.'.format(i, abs(goal - now)))
    return issues, targets


def publish_verified_target(node, pulses, duration_sec=2.0):
    # Deliberately do not invoke send_pose_target(): the repository version has
    # a _publisher_positions / _publish_positions naming mismatch and clamps 65535.
    message = ServosPosition()
    message.duration = float(duration_sec)
    message.position = []
    for servo_id, pulse in enumerate(pulses, start=1):
        servo = ServoPosition()
        servo.id = int(servo_id)
        servo.position = int(pulse)
        message.position.append(servo)
    node.arm_pose_pub.publish(message)


def main():
    args = parse_args()
    rclpy.init()
    arm = None
    try:
        arm = ArmController()
        print('[1/3] Checking live servo feedback...')
        actual = get_live_pulses(arm)
        if actual is None:
            raise RuntimeError('No live feedback for servo IDs 1..5 received on topic {}.'.format(FEEDBACK_TOPIC))
        print('Feedback topic servo ID1..5: {}'.format(actual))
        direct = get_hardware_pulses(arm)
        print('Direct hardware servo ID1..5: {}'.format(direct))
        compare_feedback(actual, direct, 'Initial feedback verification')

        print('[2/3] Reading current XYZ...')
        current = read_current_pose(arm)
        print('Current XYZ (m): x={:.4f}, y={:.4f}, z={:.4f}'.format(*current))

        target = target_from_args(args, current)
        if target is None:
            if args.move:
                raise ValueError('--move requires a target XYZ position or relative dx/dy/dz values.')
            print('Status check complete. No motor command was sent.')
            return
        if args.pitch is None or not math.isfinite(args.pitch) or not -180 <= args.pitch <= 180:
            raise ValueError('XYZ testing requires an explicit --pitch value between -180 and 180 degrees.')
        if not all(math.isfinite(v) for v in target):
            raise ValueError('XYZ coordinates must be finite numbers.')

        print('[3/3] Computing IK...')
        print('Target XYZ (m): x={:.4f}, y={:.4f}, z={:.4f}; pitch={:.1f} deg'.format(
            target[0], target[1], target[2], args.pitch))
        print('Note: --pitch does not automatically preserve the current wrist orientation.')
        response = compute_narrow_pitch_ik(arm, target, args.pitch, timeout=15.0)
        print("IK success:", response.success)
        print("IK target pulses:", list(response.pulse))
        print("IK current pulses:", list(response.current_pulse))
        if response is None:
            raise RuntimeError('IK service call failed or returned no solution.')
        issues, pulses = validate_plan(current, target, direct, response)
        if pulses is None:
            raise RuntimeError('; '.join(issues))
        print('Target servo ID1..5: {}'.format(pulses))
        print('Servo pulse deltas: {}'.format([pulses[i] - direct[i] for i in range(5)]))
        print('Straight-line distance: {:.1f} mm'.format(math.dist(current, target) * 1000))
        if issues:
            print('Safety checks: movement blocked')
            for issue in issues:
                print(' - ' + issue)
        else:
            print('Basic numerical checks passed (collision safety is NOT guaranteed).')

        if not args.move:
            print('DRY RUN: No motor command was sent.')
            return
        if issues:
            raise RuntimeError('Safety checks failed: no movement will be performed.')
        if arm.arm_pose_pub.get_subscription_count() == 0:
            raise RuntimeError('No subscriber is connected to command topic {}.'.format(COMMAND_TOPIC))

        print('\nWARNING: This test does NOT verify collisions, pinch points, or mechanical joint limits.')
        print('Clear the area and keep an emergency power cutoff available.')
        approval = input('Type MOVE to send one movement command: ').strip()
        if approval != 'MOVE':
            print('Cancelled. No motor command was sent.')
            return

        # Reconfirm fresh hardware feedback AFTER human approval; reject if
        # anything moved while the operator was deciding.
        after_approval = time.monotonic()
        now = get_live_pulses(arm, timeout=5.0, since=after_approval)
        if now is None:
            raise RuntimeError('Aborted: no fresh servo feedback after confirmation.')
        current_hw = get_hardware_pulses(arm)
        print('Pre-move direct hardware servo ID1..5: {}'.format(current_hw))
        compare_feedback(now, current_hw, 'Pre-move feedback verification')
        if any(abs(current_hw[i] - direct[i]) > 3 for i in range(5)):
            raise RuntimeError('Aborted: joint positions changed while waiting for confirmation.')
        if arm.arm_pose_pub.get_subscription_count() == 0:
            raise RuntimeError('Servo command topic subscriber disconnected.')
        publish_verified_target(arm, pulses, duration_sec=2.0)
        # Short spin for outgoing transport. Publishing != confirmed arrival.
        rclpy.spin_once(arm, timeout_sec=0.25)
        print('One command published. Checking physical servo positions after movement...')
        # A command being published does not mean it arrived or completed.
        end = time.monotonic() + 2.3
        while time.monotonic() < end:
            rclpy.spin_once(arm, timeout_sec=0.2)
        try:
            measured = get_hardware_pulses(arm)
            print('Post-move direct hardware servo ID1..5: {}'.format(measured))
            print('Measured change from pre-move: {}'.format([
                measured[i] - current_hw[i] for i in range(5)]))
            print('Requested servo ID1..5: {}'.format(pulses))
            print('Check physical clearance and actual movement yourself. No automatic retries.')
        except RuntimeError as exc:
            print('WARNING: command was sent, but post-move reading failed: {}.'.format(exc))
    except (RuntimeError, ValueError, KeyboardInterrupt) as exc:
        print('Stopped: {}'.format(exc))
    finally:
        if arm is not None:
            arm.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
