#!/usr/bin/env python3
"""Read one ROS 2 depth image and measure the center (or selected) depth patch.

Usage:
    python3 depth_check.py
    python3 depth_check.py --u 320 --v 240 --radius 5
"""
import argparse
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def main():
    parser = argparse.ArgumentParser(description='Read-only ROS 2 depth distance check')
    parser.add_argument('--topic', default='/depth_cam/depth/image_raw')
    parser.add_argument('--u', type=int, default=None, help='Depth image pixel x (default: center)')
    parser.add_argument('--v', type=int, default=None, help='Depth image pixel y (default: center)')
    parser.add_argument('--radius', type=int, default=5, help='Patch radius in pixels')
    parser.add_argument('--timeout', type=float, default=10.0, help='Wait for depth image seconds')
    args = parser.parse_args()
    if args.radius < 0 or args.timeout <= 0:
        parser.error('--radius must be >= 0 and --timeout must be positive')

    rclpy.init()
    node = Node('rovpang_depth_check')
    latest = []
    subscription = node.create_subscription(Image, args.topic, latest.append, qos_profile_sensor_data)
    try:
        end = time.monotonic() + args.timeout
        while rclpy.ok() and not latest and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
        if not latest:
            raise RuntimeError('No depth frames received. Check camera launch, topic name, and ROS_DOMAIN_ID.')
        msg = latest[0]
        enc = msg.encoding.upper()
        if enc in ('16UC1', 'MONO16'):
            dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
            multiplier = 0.001  # expected millimeters -> meters; verify against tape measure
            unit_label = 'raw units (typically millimeters)'
        elif enc == '32FC1':
            dtype = np.dtype('>f4' if msg.is_bigendian else '<f4')
            multiplier = 1.0  # expected meters
            unit_label = 'raw units (typically meters)'
        else:
            raise ValueError(f'Unsupported depth encoding: {msg.encoding!r}. Check camera driver.')

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        if raw.size != msg.step * msg.height or msg.step < msg.width * dtype.itemsize:
            raise ValueError('Invalid image data size or row stride')
        pixels = np.ndarray((msg.height, msg.width), dtype=dtype,
                            buffer=raw, strides=(msg.step, dtype.itemsize))
        u = args.u if args.u is not None else msg.width // 2
        v = args.v if args.v is not None else msg.height // 2
        if not (0 <= u < msg.width and 0 <= v < msg.height):
            raise ValueError(f'Pixel ({u}, {v}) outside {msg.width}x{msg.height}')
        r = args.radius
        patch = pixels[max(0, v-r):min(msg.height, v+r+1),
                       max(0, u-r):min(msg.width, u+r+1)].astype(np.float64)
        valid = np.isfinite(patch) & (patch > 0)
        valid_values = patch[valid]
        print(f'Topic: {args.topic}')
        print(f'Image: {msg.width}x{msg.height}, encoding={msg.encoding}, step={msg.step}, frame_id={msg.header.frame_id}')
        print(f'Selected depth pixel: (u={u}, v={v}), radius={r}')
        print(f'Center raw: {pixels[v, u]} ({unit_label})')
        print(f'Valid pixels in patch: {int(valid.sum())}/{patch.size} ({100.0 * valid.mean():.1f}%)')
        if valid_values.size:
            median_raw = float(np.median(valid_values))
            print(f'Patch median raw: {median_raw:.3f}')
            print(f'Estimated optical-axis depth Z: {median_raw * multiplier:.3f} m ({median_raw * multiplier * 100:.1f} cm)')
            print('NOTE: Confirm scale against a measured target; this is not yet an RGB-aligned 3D coordinate.')
        else:
            print('No valid depth at this patch. Aim at a textured, nonreflective target and try again.')
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
