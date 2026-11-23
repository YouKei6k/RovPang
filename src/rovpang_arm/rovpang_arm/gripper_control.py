#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RovPang gripper control and guarded open-close-open demo for JetRover.

Uses the manufacturer's /servo_controller input so ServoManager receives the
command, and independently checks actual servo ID 10 through the manufacturer's
read-only GetBusServoState service. Never sends a motor command by default.

IMPORTANT:
  * Open/closed pulse positions and mechanical safe range MUST be calibrated
    on the physical gripper. This program never assumes which direction opens.
  * --cycle controls the gripper ONLY. It is not a collision-aware arm Pick.
  * Published motion is not an emergency-stoppable or force-limited command.
"""

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import GetBusServoCmd
from servo_controller_msgs.msg import ServoPosition, ServosPosition

SERVO_ID = 10
COMMAND_TOPIC = '/servo_controller'
STATE_SERVICE = '/ros_robot_controller/bus_servo/get_state'
HARDWARE_LOW = 0
HARDWARE_HIGH = 1000
MAX_JOG = 5               # Maximum pulse shift for one exploratory command
STEP_PULSES = 5           # Maximum pulse shift per sequence command
MAX_LEG_TRAVEL = 80       # Upper bound of any continuous gripper leg
POSITION_TOLERANCE = 2   # Measurement tolerance for pulse feedback
STATE_READ_TIMEOUT = 2.5
MOTION_TIMEOUT = 3.0
DURATION_SECONDS = 0.8


class Gripper(Node):
    def __init__(self):
        super().__init__('rovpang_gripper_control')
        self.state_client = self.create_client(GetBusServoState, STATE_SERVICE)
        self.command_pub = self.create_publisher(ServosPosition, COMMAND_TOPIC, 10)

    def read_position(self):
        """Read actual bus-servo ID10; fail closed if unreadable."""
        if not self.state_client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError('Hardware read service unavailable: ' + STATE_SERVICE)
        req = GetBusServoState.Request()
        req.cmd = [GetBusServoCmd(id=SERVO_ID, get_id=1, get_position=1)]
        fut = self.state_client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=STATE_READ_TIMEOUT)
        if not fut.done():
            try:
                self.state_client.remove_pending_request(fut)
            except Exception:
                pass
            raise RuntimeError('Hardware position request timed out for servo ID10.')
        try:
            response = fut.result()
        except Exception as exc:
            raise RuntimeError('Hardware feedback error: ' + str(exc)) from exc
        if response is None or not response.success:
            raise RuntimeError('Hardware feedback request failed for servo ID10.')
        for state in response.state:
            if len(state.present_id) != len(state.position):
                continue
            for servo_id, position in zip(state.present_id, state.position):
                if int(servo_id) == SERVO_ID and HARDWARE_LOW <= int(position) <= HARDWARE_HIGH:
                    return int(position)
        raise RuntimeError('Servo ID10 position not present in hardware response.')

    def wait_command_subscriber(self, timeout=3.0):
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            if self.command_pub.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.15)
        return False

    def send_command(self, pulse):
        """Send a single gripper target through ServoManager (not direct driver)."""
        if not self.wait_command_subscriber():
            raise RuntimeError('No subscriber on ' + COMMAND_TOPIC + '; controller not ready.')
        message = ServosPosition()
        message.duration = float(DURATION_SECONDS)
        servo = ServoPosition()
        servo.id = SERVO_ID
        servo.position = int(pulse)
        message.position = [servo]
        self.command_pub.publish(message)
        rclpy.spin_once(self, timeout_sec=0.15)

    def wait_position(self, before, target):
        """Read hardware after command. Never auto-retry a failed move."""
        end = time.monotonic() + MOTION_TIMEOUT
        observed = before
        moved = False
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.15)
            observed = self.read_position()
            moved = moved or observed != before
            if abs(observed - target) <= POSITION_TOLERANCE:
                # Don't claim a real motion from merely sitting within tolerance.
                if before == target or moved:
                    return observed
        raise RuntimeError(
            'Move NOT confirmed: before={}, target={}, last_observed={}. '
            'No automatic retry; stop sequence and inspect physical gripper.'.format(
                before, target, observed)
        )

    def move_one(self, target, safe_low, safe_high):
        current = self.read_position()
        if not safe_low <= current <= safe_high:
            raise RuntimeError(
                'Current position {} outside verified range {}..{}.'.format(
                    current, safe_low, safe_high)
            )
        if not safe_low <= target <= safe_high:
            raise RuntimeError(
                'Target {} outside verified range {}..{}.'.format(
                    target, safe_low, safe_high)
            )
        if abs(target - current) > STEP_PULSES:
            raise RuntimeError('Refusing single command larger than {} pulses.'.format(STEP_PULSES))
        if target == current:
            print('ID10 already at target {} (no command).'.format(target))
            return current
        print('ID10 commanded: {} -> {}'.format(current, target), flush=True)
        self.send_command(target)
        measured = self.wait_position(current, target)
        print('ID10 hardware observed: {} (requested {}).'.format(measured, target), flush=True)
        return measured

    def move_incrementally(self, desired, safe_low, safe_high):
        current = self.read_position()
        if not safe_low <= current <= safe_high:
            raise RuntimeError('Current servo ID10 outside calibrated range; aborting.')
        if not safe_low <= desired <= safe_high:
            raise RuntimeError('Destination outside calibrated range; aborting.')
        if abs(desired - current) > MAX_LEG_TRAVEL:
            raise RuntimeError(
                'Leg is {} pulses, exceeding maximum {}. No command sent.'.format(
                    abs(desired - current), MAX_LEG_TRAVEL)
            )
        if abs(desired - current) <= POSITION_TOLERANCE:
            print('ID10 already near target ({} -> {}); no command.'.format(current, desired))
            return current

        # Each loop uses NEW hardware feedback, not commanded position as truth.
        # Stop on a stale/no-motion response; never increase power automatically.
        max_steps = math.ceil(MAX_LEG_TRAVEL / STEP_PULSES) + 3
        for _ in range(max_steps):
            current = self.read_position()
            if abs(desired - current) <= POSITION_TOLERANCE:
                return current
            next_target = current + (STEP_PULSES if desired > current else -STEP_PULSES)
            if desired > current:
                next_target = min(next_target, desired)
            else:
                next_target = max(next_target, desired)
            self.move_one(next_target, safe_low, safe_high)
        raise RuntimeError('Too many gripper increments; stopped without retry.')


def parse_args():
    p = argparse.ArgumentParser(
        description='JetRover gripper ID10: read, small jog, or calibrated cycle.'
    )
    g = p.add_mutually_exclusive_group(required=True)
    g.add_argument('--status', action='store_true', help='Read actual gripper position; no motion')
    g.add_argument('--jog', type=int, metavar='DELTA', help='One small relative pulse shift (-5..+5)')
    g.add_argument('--open', action='store_true', help='Move to calibrated open-pulse')
    g.add_argument('--close', action='store_true', help='Move to calibrated close-pulse')
    g.add_argument('--cycle', action='store_true', help='Open -> close -> open (empty gripper only)')
    p.add_argument('--open-pulse', type=int, help='Verified physical open position for ID10')
    p.add_argument('--close-pulse', type=int, help='Verified physical closed position for ID10')
    p.add_argument('--safe-min', type=int, help='Verified safe lower pulse limit for ID10')
    p.add_argument('--safe-max', type=int, help='Verified safe upper pulse limit for ID10')
    p.add_argument('--execute', action='store_true', help='Enable real motion after confirmation')
    return p.parse_args()


def build_plan(args, current):
    if args.status:
        return []
    if args.jog is not None:
        if args.jog == 0 or abs(args.jog) > MAX_JOG:
            raise ValueError('For calibration, --jog must be within -5..-1 or +1..+5.')
        goal = current + args.jog
        if goal < 50 or goal > 950:
            raise ValueError('Exploratory jog blocked near nominal 0/1000 pulse endpoints.')
        return [('JOG', goal)], 50, 950

    if args.safe_min is None or args.safe_max is None:
        raise ValueError('Open/close/cycle require measured --safe-min and --safe-max.')
    low, high = args.safe_min, args.safe_max
    if not (50 <= low < high <= 950):
        raise ValueError('Calibrated safe range must satisfy 50 <= min < max <= 950.')
    if not low <= current <= high:
        raise ValueError('Current ID10 position is outside the specified safe range.')
    if args.open_pulse is None or not low <= args.open_pulse <= high:
        raise ValueError('--open-pulse must be given and within calibrated safe range.')
    if args.close_pulse is None or not low <= args.close_pulse <= high:
        raise ValueError('--close-pulse must be given and within calibrated safe range.')
    if args.open_pulse == args.close_pulse:
        raise ValueError('Open and close pulses must differ.')
    if abs(args.open_pulse - args.close_pulse) > MAX_LEG_TRAVEL:
        raise ValueError('Open-close travel exceeds maximum continuous gripper leg (80 pulses).')
    if args.open:
        stages = [('OPEN', args.open_pulse)]
    elif args.close:
        stages = [('CLOSE', args.close_pulse)]
    else:
        stages = [('OPEN', args.open_pulse), ('CLOSE', args.close_pulse), ('OPEN', args.open_pulse)]
    if abs(stages[0][1] - current) > MAX_LEG_TRAVEL:
        raise ValueError('First motion leg exceeds maximum continuous travel (80 pulses).')
    return stages, low, high


def main():
    args = parse_args()
    rclpy.init()
    node = None
    try:
        node = Gripper()
        current = node.read_position()
        print('Actual gripper hardware ID10 position: {}'.format(current))
        if args.status:
            print('STATUS ONLY: no motor command sent.')
            return

        stages, low, high = build_plan(args, current)
        print('Plan: {}'.format(' -> '.join('{}({})'.format(name, pulse) for name, pulse in stages)))
        print('Operator-provided numeric bounds: {}..{} (NOT verified as mechanical limits)'.format(low, high))
        print('NOTE: pulse direction and physical clearance are NOT inferred by software.')
        if not args.execute:
            print('DRY RUN: no motor command sent. Use --execute after physical review.')
            return

        if not node.wait_command_subscriber():
            raise RuntimeError('No subscriber on {}. Start controller.launch.py.'.format(COMMAND_TOPIC))
        print('WARNING: Real gripper motion. Remove hands from pinch points; '
              'have emergency power cutoff ready.')
        print('Do NOT use --cycle with an object in the gripper or an obstructed workspace.')
        if input('Type MOVE to approve this full gripper plan: ').strip() != 'MOVE':
            print('Cancelled. No command sent.')
            return

        # Direct re-read after confirmation prevents stale position use.
        fresh = node.read_position()
        if abs(fresh - current) > POSITION_TOLERANCE:
            raise RuntimeError('Gripper position changed before execution; aborted.')
        for name, target in stages:
            print('\n[{}] Target ID10={}'.format(name, target), flush=True)
            if args.jog is not None:
                node.move_one(target, low, high)
            else:
                node.move_incrementally(target, low, high)
            print('[{}] Verified hardware ID10={}'.format(name, node.read_position()))
        print('\nSequence finished. Physical grasp/release was NOT independently verified.')
    except (ValueError, RuntimeError, KeyboardInterrupt) as exc:
        print('STOPPED: {}'.format(exc), file=sys.stderr)
        print('No automatic retries. Ctrl+C does not stop a command already in flight.', file=sys.stderr)
        sys.exit(1)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
