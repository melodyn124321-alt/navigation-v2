#!/usr/bin/env python3
"""Summarize live obstacle cloud and Nav2 costmap contents."""

import time

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


class RuntimeInspector(Node):
    def __init__(self):
        super().__init__("nav2_dynamic_runtime_inspector")
        # The obstacle republisher uses sensor-data (best-effort) QoS. A
        # best-effort reader is compatible with both best-effort and reliable
        # publishers and therefore works across cold/warm DDS starts.
        cloud_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.cloud_samples = 0
        self.cloud_points = 0
        self.local = None
        self.global_map = None
        self.static_map = None
        self.create_subscription(
            PointCloud2, "/nav_obstacle_cloud", self.on_cloud, cloud_qos)
        self.create_subscription(
            OccupancyGrid, "/local_costmap/costmap", self.on_local, map_qos)
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_global, map_qos)
        self.create_subscription(
            OccupancyGrid, "/map", self.on_static, map_qos)

    def on_cloud(self, msg):
        self.cloud_samples += 1
        self.cloud_points = int(msg.width) * int(msg.height)

    def on_local(self, msg):
        self.local = msg

    def on_global(self, msg):
        self.global_map = msg

    def on_static(self, msg):
        self.static_map = msg

    @staticmethod
    def map_summary(msg):
        data = msg.data
        width = int(msg.info.width)
        height = int(msg.info.height)
        center = (height // 2) * width + (width // 2)
        return (
            len(data),
            sum(value >= 99 for value in data),
            sum(1 <= value < 99 for value in data),
            sum(value < 0 for value in data),
            data[center] if 0 <= center < len(data) else -999,
        )


def main():
    rclpy.init()
    node = RuntimeInspector()
    deadline = time.monotonic() + 15.0
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        if (node.cloud_samples >= 10 and node.local is not None
                and node.global_map is not None and node.static_map is not None):
            break

    static_occupied = 0
    global_occupied = 0
    if node.static_map is not None:
        static_occupied = node.map_summary(node.static_map)[1]
    if node.global_map is not None:
        global_occupied = node.map_summary(node.global_map)[1]
    preserved_ratio = (
        global_occupied / static_occupied if static_occupied > 0 else 0.0)
    ok = (
        node.cloud_samples >= 10
        and node.local is not None
        and node.global_map is not None
        and node.static_map is not None
        and static_occupied > 0
        and preserved_ratio >= 0.90
    )
    print(f"cloud_samples={node.cloud_samples} cloud_points={node.cloud_points}")
    if node.local is not None:
        print("local_cells=%d occupied=%d inflated=%d unknown=%d center=%d" % node.map_summary(node.local))
    else:
        print("local_costmap=missing")
    if node.global_map is not None:
        print("global_cells=%d occupied=%d inflated=%d unknown=%d center=%d" % node.map_summary(node.global_map))
    else:
        print("global_costmap=missing")
    if node.static_map is not None:
        print("static_cells=%d occupied=%d inflated=%d unknown=%d center=%d" % node.map_summary(node.static_map))
    else:
        print("static_map=missing")
    print(
        f"static_obstacles={static_occupied} "
        f"global_lethal={global_occupied} "
        f"static_preserved_ratio={preserved_ratio:.3f} required>=0.900")
    print(f"NAV2_DYNAMIC_RUNTIME {'PASS' if ok else 'FAIL'}")
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
