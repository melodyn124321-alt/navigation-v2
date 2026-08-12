#!/usr/bin/env python3
"""Measure the direct navigation pose without relying on RViz rendering."""

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class NavTfSampler(Node):
    def __init__(self, topic, map_frame, base_frame):
        super().__init__("nav_tf_stability_verifier")
        self.map_frame = map_frame
        self.base_frame = base_frame
        self.samples = []
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(TFMessage, topic, self.on_tf, qos)

    def on_tf(self, message):
        for transform in message.transforms:
            if (
                    transform.header.frame_id == self.map_frame
                    and transform.child_frame_id == self.base_frame):
                translation = transform.transform.translation
                self.samples.append((
                    time.monotonic(),
                    float(translation.x),
                    float(translation.y),
                    yaw_from_quaternion(transform.transform.rotation),
                ))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/nav_tf")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--duration", type=float, default=8.0)
    parser.add_argument("--min-samples", type=int, default=20)
    parser.add_argument("--max-position-span", type=float, default=0.015)
    parser.add_argument("--max-yaw-span", type=float, default=0.02)
    args = parser.parse_args()

    rclpy.init()
    node = NavTfSampler(args.topic, args.map_frame, args.base_frame)
    started = time.monotonic()
    deadline = started + max(0.1, args.duration)
    try:
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        samples = node.samples
        node.destroy_node()
        rclpy.shutdown()

    if not samples:
        print("NAV_TF_STABILITY FAIL samples=0 reason=no_data")
        return 1

    xs = [sample[1] for sample in samples]
    ys = [sample[2] for sample in samples]
    reference_yaw = samples[0][3]
    relative_yaws = [
        math.atan2(
            math.sin(sample[3] - reference_yaw),
            math.cos(sample[3] - reference_yaw),
        )
        for sample in samples
    ]
    x_span = max(xs) - min(xs)
    y_span = max(ys) - min(ys)
    position_span = math.hypot(x_span, y_span)
    yaw_span = max(relative_yaws) - min(relative_yaws)
    elapsed = max(1.0e-6, samples[-1][0] - samples[0][0])
    rate = max(0, len(samples) - 1) / elapsed
    passed = (
        len(samples) >= args.min_samples
        and position_span <= args.max_position_span
        and yaw_span <= args.max_yaw_span
    )
    print(
        f"NAV_TF_STABILITY {'PASS' if passed else 'FAIL'} "
        f"samples={len(samples)} rate={rate:.2f}Hz "
        f"start=({xs[0]:.4f},{ys[0]:.4f},"
        f"{math.degrees(samples[0][3]):+.2f}deg) "
        f"end=({xs[-1]:.4f},{ys[-1]:.4f},"
        f"{math.degrees(samples[-1][3]):+.2f}deg) "
        f"x_span={x_span:.4f}m y_span={y_span:.4f}m "
        f"position_span={position_span:.4f}m/"
        f"{args.max_position_span:.4f}m "
        f"yaw_span={math.degrees(yaw_span):.2f}deg/"
        f"{math.degrees(args.max_yaw_span):.2f}deg"
    )
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
