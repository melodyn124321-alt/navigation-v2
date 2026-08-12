#!/usr/bin/env python3
"""Reliably publish one HN RViz goal to the Seeed-local goal bridge."""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def parse_args():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, required=True)
    parser.add_argument("--qw", type=float, required=True)
    parser.add_argument("--frame", default="map")
    parser.add_argument("--discovery-timeout", type=float, default=10.0)
    return parser.parse_args()


def main():
    args = parse_args()
    values = (args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw)
    if not all(math.isfinite(value) for value in values):
        print("SEEED_GOAL_RELAY_REJECTED non-finite pose", file=sys.stderr)
        return 2
    norm = math.sqrt(
        args.qx ** 2 + args.qy ** 2 + args.qz ** 2 + args.qw ** 2)
    if norm < 0.5 or args.frame != "map":
        print("SEEED_GOAL_RELAY_REJECTED invalid frame/orientation", file=sys.stderr)
        return 2

    rclpy.init()
    node = Node("seeed_ssh_goal_pose_relay")
    qos = QoSProfile(
        depth=10,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.VOLATILE,
    )
    publisher = node.create_publisher(PoseStamped, "/goal_pose_relay", qos)
    try:
        deadline = time.monotonic() + max(2.0, args.discovery_timeout)
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            if publisher.get_subscription_count() >= 1:
                break
        if publisher.get_subscription_count() < 1:
            print(
                "SEEED_GOAL_RELAY_FAILED bridge subscription unavailable",
                file=sys.stderr,
            )
            return 3

        message = PoseStamped()
        message.header.frame_id = "map"
        message.pose.position.x = args.x
        message.pose.position.y = args.y
        message.pose.position.z = args.z
        message.pose.orientation.x = args.qx / norm
        message.pose.orientation.y = args.qy / norm
        message.pose.orientation.z = args.qz / norm
        message.pose.orientation.w = args.qw / norm
        for _ in range(20):
            message.header.stamp = node.get_clock().now().to_msg()
            publisher.publish(message)
            rclpy.spin_once(node, timeout_sec=0.1)
        print(
            f"SEEED_GOAL_RELAY_OK target=({args.x:.3f},{args.y:.3f}) "
            f"subscribers={publisher.get_subscription_count()}")
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
