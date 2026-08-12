#!/usr/bin/env python3
"""Probe Collision Monitor output while the final motion gate is disarmed."""

import argparse
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class CollisionProbe(Node):
    def __init__(self, linear_x, angular_z, duration):
        super().__init__("collision_command_probe")
        self.linear_x = linear_x
        self.angular_z = angular_z
        self.deadline = time.monotonic() + duration
        self.outputs = []
        self.publisher = self.create_publisher(
            Twist, "/cmd_vel_nav_smoothed", 10)
        self.create_subscription(
            Twist, "/cmd_vel_nav_collision_safe", self.on_output, 10)
        self.create_timer(0.05, self.publish_command)

    def publish_command(self):
        message = Twist()
        if time.monotonic() < self.deadline:
            message.linear.x = self.linear_x
            message.angular.z = self.angular_z
        self.publisher.publish(message)

    def on_output(self, message):
        self.outputs.append((
            time.monotonic(), float(message.linear.x),
            float(message.angular.z)))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--linear", type=float, required=True)
    parser.add_argument("--angular", type=float, required=True)
    parser.add_argument("--duration", type=float, default=3.0)
    args = parser.parse_args()

    rclpy.init()
    node = CollisionProbe(args.linear, args.angular, args.duration)
    started = time.monotonic()
    finish = started + args.duration + 0.5
    while rclpy.ok() and time.monotonic() < finish:
        rclpy.spin_once(node, timeout_sec=0.05)
    steady = [
        (x, z) for stamp, x, z in node.outputs
        if stamp >= started + 0.75 and stamp <= started + args.duration
    ]
    nonzero = [
        (x, z) for x, z in steady
        if abs(x) > 1.0e-4 or abs(z) > 1.0e-4
    ]
    last = steady[-1] if steady else (0.0, 0.0)
    print(
        "COLLISION_COMMAND_PROBE "
        f"requested=({args.linear:+.3f},{args.angular:+.3f}) "
        f"samples={len(steady)} nonzero={len(nonzero)} "
        f"last_safe=({last[0]:+.3f},{last[1]:+.3f})")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if steady else 1)


if __name__ == "__main__":
    main()
