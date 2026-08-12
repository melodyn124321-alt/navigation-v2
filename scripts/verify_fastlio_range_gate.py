#!/usr/bin/env python3
"""Verify that Fast-LIO's body-frame output respects its radial range gate."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class RangeGateVerifier(Node):
    def __init__(self, topic: str, frames: int, minimum_range: float) -> None:
        super().__init__('fastlio_range_gate_verifier')
        self.target_frames = frames
        self.minimum_range = minimum_range
        self.frames = 0
        self.points = 0
        self.below_gate = 0
        self.minimum_seen = math.inf
        self.create_subscription(
            PointCloud2, topic, self.on_cloud, qos_profile_sensor_data)

    def on_cloud(self, message: PointCloud2) -> None:
        if self.frames >= self.target_frames:
            return
        for x, y, z in point_cloud2.read_points(
                message, field_names=('x', 'y', 'z'), skip_nans=True):
            distance = math.sqrt(float(x) ** 2 + float(y) ** 2 + float(z) ** 2)
            self.points += 1
            self.minimum_seen = min(self.minimum_seen, distance)
            if distance + 1e-6 < self.minimum_range:
                self.below_gate += 1
        self.frames += 1


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/cloud_registered_body')
    parser.add_argument('--frames', type=int, default=10)
    parser.add_argument('--minimum-range', type=float, default=0.70)
    parser.add_argument('--timeout', type=float, default=15.0)
    args = parser.parse_args()

    rclpy.init()
    node = RangeGateVerifier(args.topic, args.frames, args.minimum_range)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and node.frames < node.target_frames and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(
        f'topic={args.topic} frames={node.frames}/{node.target_frames} '
        f'points={node.points} minimum_seen={node.minimum_seen:.3f}m '
        f'points_below_{args.minimum_range:.3f}m={node.below_gate}')
    if node.frames < node.target_frames or node.points == 0:
        print('RESULT: FAIL no complete Fast-LIO point-cloud sample')
        return 2
    if node.below_gate:
        print('RESULT: FAIL points below the configured range gate remain')
        return 3
    print('RESULT: PASS Fast-LIO output respects the radial range gate')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
