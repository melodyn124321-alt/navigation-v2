#!/usr/bin/env python3
"""Publish a visible alarm when a navigation goal is blocked for too long."""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


class NavObstacleBlockAlarm(Node):
    def __init__(self):
        super().__init__("nav_obstacle_block_alarm")
        self.cloud_topic = self.declare_parameter(
            "cloud_topic", "/nav_obstacle_cloud").value
        self.goal_status_topic = self.declare_parameter(
            "goal_status_topic", "/aligned_goal_status").value
        self.stop_radius = float(self.declare_parameter(
            "stop_radius", 1.08).value)
        self.min_height = float(self.declare_parameter(
            "min_height", 0.10).value)
        self.max_height = float(self.declare_parameter(
            "max_height", 1.60).value)
        self.min_points = max(1, int(self.declare_parameter(
            "min_points", 3).value))
        self.alarm_timeout = max(1.0, float(self.declare_parameter(
            "alarm_timeout_sec", 60.0).value))
        self.clear_hold = max(0.0, float(self.declare_parameter(
            "clear_hold_sec", 0.50).value))
        self.cloud_timeout = max(0.1, float(self.declare_parameter(
            "cloud_timeout_sec", 1.00).value))

        sensor_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            PointCloud2, self.cloud_topic, self.on_cloud, sensor_qos)
        self.create_subscription(
            String, self.goal_status_topic, self.on_goal_status, latched_qos)
        self.alarm_pub = self.create_publisher(
            Bool, "/nav_obstacle_alarm", latched_qos)
        self.status_pub = self.create_publisher(
            String, "/nav_obstacle_alarm_status", latched_qos)
        self.marker_pub = self.create_publisher(
            MarkerArray, "/nav_obstacle_alarm_markers", latched_qos)

        self.goal_active = False
        self.obstacle_points = 0
        self.last_cloud_time = None
        self.blocked_since = None
        self.clear_since = None
        self.alarm_active = False
        self.last_logged_second = -1
        self.create_timer(0.20, self.on_timer)
        self.publish_alarm(False)
        self.publish_status("IDLE waiting for an active navigation goal")
        self.get_logger().info(
            f"block alarm ready: radius={self.stop_radius:.2f}m, "
            f"points>={self.min_points}, timeout={self.alarm_timeout:.1f}s")

    @staticmethod
    def field_offset(msg, name):
        for field in msg.fields:
            if field.name == name and field.datatype == 7:
                return int(field.offset)
        return None

    def on_goal_status(self, msg):
        text = msg.data.strip().upper()
        active = text.startswith("ACTIVE")
        if active != self.goal_active:
            self.goal_active = active
            if not active:
                self.reset_block("navigation goal is no longer active")

    def on_cloud(self, msg):
        self.last_cloud_time = self.get_clock().now()
        offsets = [self.field_offset(msg, axis) for axis in ("x", "y", "z")]
        if any(offset is None for offset in offsets):
            self.obstacle_points = 0
            self.get_logger().error(
                "PointCloud2 requires FLOAT32 x/y/z fields",
                throttle_duration_sec=5.0)
            return
        count = int(msg.width) * int(msg.height)
        step = int(msg.point_step)
        if count <= 0 or step <= 0 or len(msg.data) < count * step:
            self.obstacle_points = 0
            return
        endian = ">" if msg.is_bigendian else "<"
        values = []
        for offset in offsets:
            values.append(np.ndarray(
                shape=(count,), dtype=endian + "f4", buffer=msg.data,
                offset=offset, strides=(step,)))
        x, y, z = values
        inside = (
            np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
            & ((x * x + y * y) <= self.stop_radius * self.stop_radius)
            & (z >= self.min_height) & (z <= self.max_height)
        )
        self.obstacle_points = int(np.count_nonzero(inside))

    def elapsed(self, start):
        if start is None:
            return 0.0
        return (self.get_clock().now() - start).nanoseconds / 1e9

    def cloud_fresh(self):
        return (
            self.last_cloud_time is not None
            and self.elapsed(self.last_cloud_time) <= self.cloud_timeout
        )

    def publish_alarm(self, active):
        self.alarm_pub.publish(Bool(data=active))

    def publish_status(self, text):
        self.status_pub.publish(String(data=text))

    def reset_block(self, reason):
        was_blocked = self.blocked_since is not None
        was_alarm = self.alarm_active
        self.blocked_since = None
        self.clear_since = None
        self.alarm_active = False
        self.last_logged_second = -1
        self.publish_alarm(False)
        self.delete_markers()
        if was_blocked or was_alarm:
            self.get_logger().info(f"obstacle block cleared: {reason}")

    def delete_markers(self):
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[marker]))

    def publish_marker(self, blocked_for, alarm):
        now = self.get_clock().now().to_msg()
        color = (1.0, 0.05, 0.05) if alarm else (1.0, 0.65, 0.0)
        ring = Marker()
        ring.header.frame_id = "base_link"
        ring.header.stamp = now
        ring.ns = "obstacle_alarm"
        ring.id = 0
        ring.type = Marker.CYLINDER
        ring.action = Marker.ADD
        ring.pose.orientation.w = 1.0
        ring.pose.position.z = 0.03
        ring.scale.x = 2.0 * self.stop_radius
        ring.scale.y = 2.0 * self.stop_radius
        ring.scale.z = 0.04
        ring.color.r, ring.color.g, ring.color.b = color
        ring.color.a = 0.45 if alarm else 0.25

        text = Marker()
        text.header.frame_id = "base_link"
        text.header.stamp = now
        text.ns = "obstacle_alarm"
        text.id = 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.orientation.w = 1.0
        text.pose.position.z = 1.25
        text.scale.z = 0.22
        text.color.r, text.color.g, text.color.b = color
        text.color.a = 1.0
        if alarm:
            text.text = f"OBSTACLE ALARM  {blocked_for:.0f}s"
        else:
            remaining = max(0.0, self.alarm_timeout - blocked_for)
            text.text = f"OBSTACLE STOP  alarm in {remaining:.0f}s"
        self.marker_pub.publish(MarkerArray(markers=[ring, text]))

    def on_timer(self):
        if not self.goal_active:
            self.publish_status("IDLE no active navigation goal")
            return
        if not self.cloud_fresh():
            self.reset_block("obstacle cloud is stale; motion gate remains fail-safe")
            self.publish_status("WAITING stale obstacle cloud; motion is inhibited")
            return

        blocked = self.obstacle_points >= self.min_points
        if blocked:
            self.clear_since = None
            if self.blocked_since is None:
                self.blocked_since = self.get_clock().now()
                self.get_logger().warn(
                    f"obstacle stop started with {self.obstacle_points} points")
            duration = self.elapsed(self.blocked_since)
            if duration >= self.alarm_timeout and not self.alarm_active:
                self.alarm_active = True
                self.publish_alarm(True)
                self.get_logger().error(
                    f"OBSTACLE ALARM: blocked continuously for {duration:.1f}s")
            whole_second = int(math.floor(duration))
            if whole_second != self.last_logged_second:
                self.last_logged_second = whole_second
                self.publish_status(
                    f"{'ALARM' if self.alarm_active else 'BLOCKED'} "
                    f"duration={duration:.1f}s points={self.obstacle_points} "
                    f"timeout={self.alarm_timeout:.1f}s")
            self.publish_marker(duration, self.alarm_active)
            return

        if self.blocked_since is None:
            self.publish_status("CLEAR active goal; stop zone is clear")
            return
        if self.clear_since is None:
            self.clear_since = self.get_clock().now()
        clear_for = self.elapsed(self.clear_since)
        if clear_for >= self.clear_hold:
            self.reset_block(f"clear for {clear_for:.2f}s")
            self.publish_status("CLEAR obstacle removed; navigation may resume")


def main():
    rclpy.init()
    node = NavObstacleBlockAlarm()
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
