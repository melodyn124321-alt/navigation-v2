#!/usr/bin/env python3
"""Verify injected obstacle points land at their requested base_link pose."""

import struct
import time

import numpy as np
import rclpy
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_nav_obstacle_transform")
        self.result = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            PointCloud2, "/nav_obstacle_cloud", self.on_cloud, qos)
        self.publisher = self.create_publisher(
            Bool, "/nav_obstacle_test_enable", 10)
        self.clear_clients = [
            self.create_client(
                ClearEntireCostmap,
                "/local_costmap/clear_entirely_local_costmap"),
            self.create_client(
                ClearEntireCostmap,
                "/global_costmap/clear_entirely_global_costmap"),
        ]
        self.create_timer(0.02, lambda: self.publisher.publish(Bool(data=True)))

    @staticmethod
    def field_offset(message, name):
        for field in message.fields:
            if field.name == name:
                return field.offset
        raise RuntimeError(f"missing field {name}")

    def on_cloud(self, message):
        if message.header.frame_id != "base_link" or message.width < 5:
            return
        byte_order = ">" if message.is_bigendian else "<"
        offsets = [self.field_offset(message, axis) for axis in "xyz"]
        base_points = []
        for index in range(message.width - 5, message.width):
            start = index * message.point_step
            base_points.append(np.array([
                struct.unpack_from(byte_order + "f", message.data, start + offset)[0]
                for offset in offsets
            ]))
        expected_y = np.array([-0.08, -0.04, 0.0, 0.04, 0.08])
        errors = [
            # Keep this synchronized with the republisher's injected point at
            # x=0.42 m, just outside the 0.38 m chassis footprint.
            abs(point[0] - 0.42) + abs(point[1] - y) + abs(point[2] - 0.30)
            for point, y in zip(base_points, expected_y)
        ]
        if max(errors) < 1.0e-3:
            self.result = base_points


def main():
    rclpy.init()
    node = Verifier()
    deadline = time.monotonic() + 8.0
    while rclpy.ok() and node.result is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
    # Stop injection reliably before clearing its marked cells. Repeating the
    # false sample also covers a late DDS match during cold-start verification.
    for _index in range(5):
        node.publisher.publish(Bool(data=False))
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
        print("OBSTACLE_TEST_CLEANUP cleared=" + ",".join(cleared))
        cleanup_deadline = time.monotonic() + 2.0
        while rclpy.ok() and time.monotonic() < cleanup_deadline:
            rclpy.spin_once(node, timeout_sec=0.10)
    if node.result is None:
        print("OBSTACLE_TRANSFORM FAIL injected points were not found")
        result = 1
    else:
        for point in node.result:
            print("base_point=(%.4f, %.4f, %.4f)" % tuple(point))
        print("OBSTACLE_TRANSFORM PASS output cloud is in base_link")
        result = 0
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(result)


if __name__ == "__main__":
    main()
