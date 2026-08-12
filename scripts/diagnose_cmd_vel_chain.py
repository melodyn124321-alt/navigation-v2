#!/usr/bin/env python3
"""Measure the resident Nav2 velocity chain while the final gate is disarmed.

The default probe is an all-zero Twist.  ``--angular-z`` may exercise the
rotation path, but must only be used after the physical motion gate has been
explicitly disarmed; the final /cmd_vel output is included in the report.
"""

import argparse
import math
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TopicStats:
    def __init__(self):
        self.count = 0
        self.nonzero_count = 0
        self.first = None
        self.last = None
        self.gaps = []
        self.max_abs_angular = 0.0

    def add(self, now, message):
        if self.last is not None:
            self.gaps.append(now - self.last)
        else:
            self.first = now
        self.last = now
        self.count += 1
        magnitude = max(
            abs(float(message.linear.x)),
            abs(float(message.linear.y)),
            abs(float(message.angular.z)),
        )
        if magnitude > 0.001:
            self.nonzero_count += 1
        self.max_abs_angular = max(
            self.max_abs_angular, abs(float(message.angular.z)))

    def summary(self):
        duration = (
            self.last - self.first
            if self.first is not None and self.last is not None else 0.0
        )
        rate = (self.count - 1) / duration if duration > 0.0 else 0.0
        max_gap = max(self.gaps) if self.gaps else math.inf
        over_030 = sum(gap > 0.30 for gap in self.gaps)
        return (
            f"count={self.count} duration={duration:.3f}s "
            f"rate={rate:.3f}Hz max_gap={max_gap:.3f}s "
            f"gaps_over_0.30s={over_030} "
            f"nonzero={self.nonzero_count} "
            f"max_abs_angular={self.max_abs_angular:.3f}rad/s"
        )


class CmdVelChainProbe(Node):
    TOPICS = (
        "/cmd_vel_nav",
        "/cmd_vel_nav_smoothed",
        "/cmd_vel_nav_collision_safe",
        "/cmd_vel",
    )

    def __init__(self, publish_hz, angular_z):
        super().__init__("cmd_vel_chain_zero_probe")
        self.stats = {topic: TopicStats() for topic in self.TOPICS}
        for topic in self.TOPICS:
            self.create_subscription(
                Twist,
                topic,
                lambda msg, topic=topic: self.stats[topic].add(
                    time.monotonic(), msg),
                100,
            )
        self.publisher = self.create_publisher(Twist, "/cmd_vel_nav", 10)
        self.timer = self.create_timer(1.0 / publish_hz, self.publish_command)
        self.publish_count = 0
        self.angular_z = angular_z

    def publish_command(self):
        command = Twist()
        command.angular.z = self.angular_z
        self.publisher.publish(command)
        self.publish_count += 1


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=15.0)
    parser.add_argument("--warmup", type=float, default=8.0)
    parser.add_argument("--publish-hz", type=float, default=10.0)
    parser.add_argument("--angular-z", type=float, default=0.0)
    args = parser.parse_args()
    if args.duration <= 0.0 or args.warmup < 0.0 or args.publish_hz <= 0.0:
        parser.error("duration/publish-hz must be positive and warmup nonnegative")

    rclpy.init()
    node = CmdVelChainProbe(args.publish_hz, args.angular_z)
    started = time.monotonic()
    measuring = False
    try:
        while rclpy.ok() and time.monotonic() - started < args.warmup + args.duration:
            rclpy.spin_once(node, timeout_sec=0.05)
            if not measuring and time.monotonic() - started >= args.warmup:
                node.stats = {topic: TopicStats() for topic in node.TOPICS}
                node.publish_count = 0
                measuring = True
    finally:
        node.publisher.publish(Twist())
        print(
            f"probe_publish count={node.publish_count} "
            f"requested_rate={args.publish_hz:.3f}Hz "
            f"angular_z={args.angular_z:.3f}rad/s"
        )
        for topic in node.TOPICS:
            print(f"{topic}: {node.stats[topic].summary()}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
