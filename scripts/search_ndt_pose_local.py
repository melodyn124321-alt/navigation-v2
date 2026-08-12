#!/usr/bin/env python3
"""Search a bounded initial-pose grid and retain the best live NDT score."""

import argparse
import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class SearchNode(Node):
    def __init__(self):
        super().__init__("search_ndt_pose_local")
        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped, "/initialpose_relay", 10)
        qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.samples = []
        self.subscription = self.create_subscription(
            Odometry, "/relocalization_odom", self.on_odom, qos)

    def on_odom(self, message):
        self.samples.append((
            time.monotonic(), float(message.pose.covariance[0])))

    def publish_pose(self, x, y, yaw):
        message = PoseWithCovarianceStamped()
        message.header.frame_id = "map"
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = x
        message.pose.pose.position.y = y
        message.pose.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.pose.orientation.w = math.cos(yaw / 2.0)
        message.pose.covariance[0] = 0.04
        message.pose.covariance[7] = 0.04
        message.pose.covariance[35] = 0.0685
        for _ in range(3):
            self.publisher.publish(message)
            rclpy.spin_once(self, timeout_sec=0.08)


def values(center, radius, step):
    count = int(round(radius / step))
    return [center + index * step for index in range(-count, count + 1)]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--yaw-deg", type=float, required=True)
    parser.add_argument("--xy-radius", type=float, default=0.10)
    parser.add_argument("--xy-step", type=float, default=0.10)
    parser.add_argument("--yaw-radius-deg", type=float, default=3.0)
    parser.add_argument("--yaw-step-deg", type=float, default=3.0)
    parser.add_argument("--settle", type=float, default=0.55)
    parser.add_argument("--sample", type=float, default=1.25)
    args = parser.parse_args()

    rclpy.init()
    node = SearchNode()
    results = []
    try:
        # Allow DDS discovery before the first pose is evaluated.
        deadline = time.monotonic() + 1.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        for yaw_deg in values(
                args.yaw_deg, args.yaw_radius_deg, args.yaw_step_deg):
            for y in values(args.y, args.xy_radius, args.xy_step):
                for x in values(args.x, args.xy_radius, args.xy_step):
                    node.samples.clear()
                    node.publish_pose(x, y, math.radians(yaw_deg))
                    started = time.monotonic()
                    deadline = started + args.settle + args.sample
                    while time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.10)
                    scores = [
                        score for stamp, score in node.samples
                        if stamp >= started + args.settle
                        and math.isfinite(score)
                    ]
                    score = statistics.median(scores) if scores else math.inf
                    results.append((score, x, y, yaw_deg, len(scores)))
                    print(
                        f"NDT_GRID score={score:.4f} samples={len(scores)} "
                        f"x={x:.3f} y={y:.3f} yaw_deg={yaw_deg:+.1f}",
                        flush=True)
        results.sort()
        best = results[0]
        print(
            f"NDT_GRID_BEST score={best[0]:.4f} samples={best[4]} "
            f"x={best[1]:.3f} y={best[2]:.3f} yaw_deg={best[3]:+.1f}")
        node.publish_pose(best[1], best[2], math.radians(best[3]))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
