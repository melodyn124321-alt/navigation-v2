#!/usr/bin/env python3
"""Publish a low-bandwidth FAST-LIO cloud intended only for remote RViz."""

import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2


class FastlioVizCloudThrottle(Node):
    def __init__(self):
        super().__init__("fastlio_viz_cloud_throttle")
        self.input_topic = self.declare_parameter(
            "input_topic", "/cloud_registered").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/cloud_registered_viz").value
        self.max_rate_hz = max(
            0.1, float(self.declare_parameter("max_rate_hz", 2.0).value))
        self.point_stride = max(
            1, int(self.declare_parameter("point_stride", 2).value))
        output_reliability = str(self.declare_parameter(
            "output_reliability", "best_effort").value).strip().lower()
        output_durability = str(self.declare_parameter(
            "output_durability", "volatile").value).strip().lower()
        if output_reliability not in ("best_effort", "reliable"):
            raise ValueError(
                "output_reliability must be best_effort or reliable")
        if output_durability not in ("volatile", "transient_local"):
            raise ValueError(
                "output_durability must be volatile or transient_local")
        self.period = 1.0 / self.max_rate_hz
        self.next_publish = 0.0
        self.received = 0
        self.published = 0
        self.last_input_points = 0
        self.last_output_points = 0

        input_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        output_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=(ReliabilityPolicy.RELIABLE
                         if output_reliability == "reliable"
                         else ReliabilityPolicy.BEST_EFFORT),
            durability=(DurabilityPolicy.TRANSIENT_LOCAL
                        if output_durability == "transient_local"
                        else DurabilityPolicy.VOLATILE),
        )
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, output_qos)
        self.subscription = self.create_subscription(
            PointCloud2, self.input_topic, self.on_cloud, input_qos)
        self.create_timer(10.0, self.report)
        self.get_logger().info(
            f"throttling {self.input_topic} -> {self.output_topic} at "
            f"{self.max_rate_hz:.1f} Hz, point_stride={self.point_stride}, "
            f"output QoS={output_reliability}/{output_durability} depth=1")

    @staticmethod
    def point_offset(message, flat_index):
        row = flat_index // message.width
        column = flat_index % message.width
        return row * message.row_step + column * message.point_step

    def decimate(self, message):
        total_points = int(message.width) * int(message.height)
        if self.point_stride == 1 or total_points <= 1:
            return message

        selected = range(0, total_points, self.point_stride)
        data = bytearray()
        source = memoryview(message.data)
        for index in selected:
            offset = self.point_offset(message, index)
            data.extend(source[offset:offset + message.point_step])

        output = PointCloud2()
        output.header = message.header
        output.height = 1
        output.width = len(data) // message.point_step
        output.fields = message.fields
        output.is_bigendian = message.is_bigendian
        output.point_step = message.point_step
        output.row_step = output.width * output.point_step
        output.data = data
        output.is_dense = message.is_dense
        return output

    def on_cloud(self, message):
        self.received += 1
        self.last_input_points = int(message.width) * int(message.height)
        now = time.monotonic()
        if now < self.next_publish:
            return
        self.next_publish = now + self.period
        output = self.decimate(message)
        self.last_output_points = int(output.width) * int(output.height)
        self.publisher.publish(output)
        self.published += 1

    def report(self):
        self.get_logger().info(
            f"health received={self.received} published={self.published} "
            f"points={self.last_input_points}->{self.last_output_points}")


def main():
    rclpy.init()
    node = FastlioVizCloudThrottle()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
