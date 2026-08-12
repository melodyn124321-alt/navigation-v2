#!/usr/bin/env python3
"""Republish the rolling local costmap in map frame for remote HN RViz."""

import copy
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


class LocalCostmapRepublisher(Node):
    def __init__(self):
        super().__init__("hn_local_costmap_republisher")
        source_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        output_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.publisher = self.create_publisher(
            OccupancyGrid, "/hn_local_costmap", output_qos)
        self.status_publisher = self.create_publisher(
            String, "/hn_local_costmap_status", output_qos)
        self.create_subscription(
            OccupancyGrid,
            "/local_costmap/costmap",
            self.on_costmap,
            source_qos,
        )
        self.received = 0
        self.published = 0
        self.last_error = "waiting for /local_costmap/costmap"
        self.create_timer(1.0, self.publish_status)

    def on_costmap(self, message):
        self.received += 1
        source_frame = message.header.frame_id or "odom"
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", source_frame, Time(), timeout=Duration(seconds=0.1))
        except TransformException as error:
            self.last_error = f"TF map<-{source_frame}: {error}"
            return

        output = copy.deepcopy(message)
        output.header.frame_id = "map"
        output.header.stamp = self.get_clock().now().to_msg()
        translation = transform.transform.translation
        transform_yaw = yaw_from_quaternion(transform.transform.rotation)
        origin = message.info.origin
        origin_yaw = yaw_from_quaternion(origin.orientation)
        cosine = math.cos(transform_yaw)
        sine = math.sin(transform_yaw)
        output.info.origin.position.x = (
            translation.x
            + cosine * origin.position.x
            - sine * origin.position.y
        )
        output.info.origin.position.y = (
            translation.y
            + sine * origin.position.x
            + cosine * origin.position.y
        )
        output.info.origin.position.z = translation.z + origin.position.z
        output_yaw = transform_yaw + origin_yaw
        output.info.origin.orientation.x = 0.0
        output.info.origin.orientation.y = 0.0
        output.info.origin.orientation.z = math.sin(output_yaw * 0.5)
        output.info.origin.orientation.w = math.cos(output_yaw * 0.5)
        self.publisher.publish(output)
        self.published += 1
        self.last_error = "none"

    def publish_status(self):
        status = String()
        state = "READY" if self.published > 0 else "WAITING"
        status.data = (
            f"{state} received={self.received} published={self.published} "
            f"error={self.last_error}"
        )
        self.status_publisher.publish(status)


def main():
    rclpy.init()
    node = LocalCostmapRepublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
