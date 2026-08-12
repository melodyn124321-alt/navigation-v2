#!/usr/bin/env python3
"""Inspect one live navigation obstacle cloud in base_link coordinates."""

import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class Analyzer(Node):
    def __init__(self):
        super().__init__("nav_obstacle_cloud_analyzer")
        self.message = None
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PointCloud2, "/nav_obstacle_cloud", self.on_cloud, qos)

    def on_cloud(self, message):
        self.message = message


def field_offset(message, name):
    for field in message.fields:
        if field.name == name and field.datatype == 7:
            return int(field.offset)
    raise RuntimeError(f"missing FLOAT32 field {name}")


def describe(name, mask, x, y, z):
    count = int(np.count_nonzero(mask))
    if count == 0:
        print(f"{name}: count=0")
        return
    print(
        f"{name}: count={count} "
        f"x=[{np.min(x[mask]):.3f},{np.max(x[mask]):.3f}] "
        f"y=[{np.min(y[mask]):.3f},{np.max(y[mask]):.3f}] "
        f"z_p05/p50/p95="
        f"{np.quantile(z[mask], 0.05):.3f}/"
        f"{np.quantile(z[mask], 0.50):.3f}/"
        f"{np.quantile(z[mask], 0.95):.3f}m"
    )


def main():
    rclpy.init()
    node = Analyzer()
    deadline = time.monotonic() + 20.0
    while rclpy.ok() and node.message is None and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.20)
    message = node.message
    node.destroy_node()
    rclpy.shutdown()
    if message is None:
        raise RuntimeError("no /nav_obstacle_cloud message within 20s")

    count = int(message.width) * int(message.height)
    step = int(message.point_step)
    endian = ">" if message.is_bigendian else "<"
    arrays = []
    for axis in ("x", "y", "z"):
        arrays.append(np.ndarray(
            shape=(count,), dtype=endian + "f4", buffer=message.data,
            offset=field_offset(message, axis), strides=(step,)))
    x, y, z = arrays
    finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    height = finite & (z >= 0.08) & (z <= 1.60)
    front = height & (x >= 0.30) & (x <= 0.90) & (np.abs(y) <= 0.38)
    rotation = height & ((x * x + y * y) <= 0.60 * 0.60)
    near_ground = finite & (z >= -0.05) & (z <= 0.20)
    print(
        f"frame={message.header.frame_id} total={count} "
        f"finite={int(np.count_nonzero(finite))}"
    )
    describe("collision_height", height, x, y, z)
    describe("front_stop_zone", front, x, y, z)
    describe("rotation_stop_zone", rotation, x, y, z)
    describe("near_ground_-0.05_to_0.20", near_ground, x, y, z)


if __name__ == "__main__":
    main()
