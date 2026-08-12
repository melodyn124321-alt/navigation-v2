#!/usr/bin/env python3
"""Inject a close obstacle and verify trajectory-aware collision limiting."""

import time
import argparse

import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header
from std_msgs.msg import Bool


class DynamicAvoidanceVerifier(Node):
    def __init__(self, direct_cloud=False, obstacle_x=0.42):
        super().__init__("nav2_dynamic_avoidance_verifier")
        self.test_enable_pub = self.create_publisher(
            Bool, "/nav_obstacle_test_enable", 10)
        self.command_pub = self.create_publisher(
            Twist, "/cmd_vel_nav_smoothed", 10)
        self.direct_cloud = direct_cloud
        self.obstacle_x = obstacle_x
        cloud_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.cloud_pub = self.create_publisher(
            PointCloud2, "/nav_obstacle_cloud", cloud_qos)
        self.clear_clients = [
            self.create_client(
                ClearEntireCostmap,
                "/local_costmap/clear_entirely_local_costmap"),
            self.create_client(
                ClearEntireCostmap,
                "/global_costmap/clear_entirely_global_costmap"),
        ]
        self.collision_subscription = self.create_subscription(
            Twist, "/cmd_vel_nav_collision_safe", self.on_collision_output, 10)
        self.base_subscription = self.create_subscription(
            Twist, "/cmd_vel", self.on_base_output, 10)
        self.collision_outputs = []
        self.base_outputs = []
        self.started = time.monotonic()
        self.warmup_sec = 5.0
        self.create_timer(0.02, self.publish_test_data)

    def on_collision_output(self, msg):
        self.collision_outputs.append(
            (time.monotonic(), msg.linear.x, msg.angular.z))

    def on_base_output(self, msg):
        self.base_outputs.append(
            (time.monotonic(), msg.linear.x, msg.angular.z))

    def publish_test_data(self):
        if time.monotonic() - self.started < self.warmup_sec:
            return
        self.test_enable_pub.publish(Bool(data=True))
        if self.direct_cloud:
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "base_link"
            points = [
                (self.obstacle_x, y, 0.30)
                for y in (-0.12, -0.08, -0.04, 0.0, 0.04, 0.08, 0.12)
            ]
            self.cloud_pub.publish(
                point_cloud2.create_cloud_xyz32(header, points))
        command = Twist()
        command.linear.x = 0.05
        self.command_pub.publish(command)

    def result(self, safe_linear_limit):
        steady_start = self.started + self.warmup_sec + 0.50
        collision_samples = [
            (x, z) for stamp, x, z in self.collision_outputs
            if stamp >= steady_start
        ]
        base_samples = [
            (x, z) for stamp, x, z in self.base_outputs
            if stamp >= steady_start
        ]
        collision_linear_max = max(
            (abs(x) for x, _z in collision_samples), default=float("inf"))
        collision_angular_max = max(
            (abs(z) for _x, z in collision_samples), default=float("inf"))
        base_max = max(
            (abs(x) + abs(z) for x, z in base_samples), default=float("inf"))
        passed = (
            len(collision_samples) >= 1
            and collision_linear_max <= safe_linear_limit + 5.0e-4
            and collision_angular_max < 1.0e-4
            and len(base_samples) >= 10
            and base_max < 1.0e-4
        )
        return (
            passed, collision_linear_max, collision_angular_max,
            base_max, len(collision_samples), len(base_samples))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--warmup-sec", type=float, default=5.0)
    parser.add_argument("--test-sec", type=float, default=8.0)
    parser.add_argument("--direct-cloud", action="store_true")
    parser.add_argument("--obstacle-x", type=float, default=0.42)
    parser.add_argument("--footprint-front-x", type=float, default=0.38)
    parser.add_argument("--time-before-collision", type=float, default=2.5)
    args = parser.parse_args()
    rclpy.init()
    node = DynamicAvoidanceVerifier(
        direct_cloud=args.direct_cloud, obstacle_x=args.obstacle_x)
    node.warmup_sec = args.warmup_sec
    deadline = time.monotonic() + node.warmup_sec + args.test_sec
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    clearance = max(0.0, args.obstacle_x - args.footprint_front_x)
    safe_linear_limit = clearance / max(args.time_before_collision, 0.01)
    (
        passed, collision_linear_max, collision_angular_max,
        base_max, collision_count, base_count,
    ) = node.result(safe_linear_limit)
    for _index in range(5):
        node.test_enable_pub.publish(Bool(data=False))
        rclpy.spin_once(node, timeout_sec=0.10)
    cleared = []
    for client in node.clear_clients:
        if not client.wait_for_service(timeout_sec=1.0):
            continue
        future = client.call_async(ClearEntireCostmap.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        if future.done() and future.result() is not None:
            cleared.append(client.srv_name)
    if cleared:
        print("DYNAMIC_TEST_CLEANUP cleared=" + ",".join(cleared))
        cleanup_deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < cleanup_deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
    print(
        "DYNAMIC_AVOIDANCE_TEST "
        f"{'PASS' if passed else 'FAIL'} "
        f"command_subscribers={node.count_subscribers('/cmd_vel_nav_smoothed')} "
        f"collision_publishers={node.count_publishers('/cmd_vel_nav_collision_safe')} "
        f"steady_collision_samples={collision_count} "
        f"safe_linear_limit={safe_linear_limit:.6f} "
        f"collision_linear_max={collision_linear_max:.6f} "
        f"collision_angular_max={collision_angular_max:.6f} "
        f"steady_base_samples={base_count} base_max={base_max:.6f}")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if passed else 1)


if __name__ == "__main__":
    main()
