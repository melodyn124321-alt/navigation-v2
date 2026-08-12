#!/usr/bin/env python3
"""Print one Fast-LIO body-cloud geometry summary without publishing data."""

import math
import struct

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class CloudGeometryInspector(Node):
    def __init__(self):
        super().__init__("cloud_geometry_inspector")
        topic = self.declare_parameter(
            "topic", "/cloud_registered_body").value
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscription = self.create_subscription(
            PointCloud2, topic, self.on_cloud, qos)
        self.done = False

    @staticmethod
    def percentile(values, ratio):
        index = round((len(values) - 1) * ratio)
        return values[max(0, min(index, len(values) - 1))]

    def on_cloud(self, msg):
        if self.done:
            return
        offsets = {
            field.name: field.offset
            for field in msg.fields
            if field.name in ("x", "y", "z") and field.datatype == 7
        }
        if len(offsets) != 3:
            self.get_logger().error("cloud has no FLOAT32 x/y/z fields")
            rclpy.shutdown()
            return

        order = ">" if msg.is_bigendian else "<"
        axes = {"x": [], "y": [], "z": []}
        count = int(msg.width) * int(msg.height)
        stride = max(1, count // 20000)
        for index in range(0, count, stride):
            row, column = divmod(index, int(msg.width))
            start = row * int(msg.row_step) + column * int(msg.point_step)
            try:
                xyz = {
                    name: struct.unpack_from(
                        order + "f", msg.data, start + offset)[0]
                    for name, offset in offsets.items()
                }
            except (IndexError, struct.error):
                continue
            if all(math.isfinite(value) for value in xyz.values()):
                for name, value in xyz.items():
                    axes[name].append(value)

        print(
            f"frame={msg.header.frame_id} source_points={count} "
            f"sampled_points={len(axes['z'])}")
        for name in ("x", "y", "z"):
            values = sorted(axes[name])
            if not values:
                continue
            print(
                f"{name}: min={values[0]:.3f} "
                f"p01={self.percentile(values, 0.01):.3f} "
                f"p05={self.percentile(values, 0.05):.3f} "
                f"p50={self.percentile(values, 0.50):.3f} "
                f"p95={self.percentile(values, 0.95):.3f} "
                f"p99={self.percentile(values, 0.99):.3f} "
                f"max={values[-1]:.3f}")
        z_bins = {}
        for value in axes["z"]:
            if -1.0 <= value <= 1.0:
                center = round(value / 0.05) * 0.05
                z_bins[center] = z_bins.get(center, 0) + 1
        densest = sorted(z_bins.items(), key=lambda item: item[1], reverse=True)[:8]
        print("densest_z_5cm_bins=" + ", ".join(
            f"{center:+.2f}:{count}" for center, count in densest), flush=True)
        self.done = True


def main():
    rclpy.init()
    node = CloudGeometryInspector()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
