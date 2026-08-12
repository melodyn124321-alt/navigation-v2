#!/usr/bin/env python3
"""Publish a direct localization transform for Nav2.

The global TF tree remains map->odom->base_link for RViz and diagnostics.
Nav2 can use a separate child frame such as nav_base_link, avoiding duplicate
parents for the real base_link frame while still giving costmaps a short TF
chain.
"""

import math

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_msgs.msg import TFMessage


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def multiply_quaternions(q, offset):
    ox, oy, oz, ow = offset
    return (
        q.w * ox + q.x * ow + q.y * oz - q.z * oy,
        q.w * oy - q.x * oz + q.y * ow + q.z * ox,
        q.w * oz + q.x * oy - q.y * ox + q.z * ow,
        q.w * ow - q.x * ox - q.y * oy - q.z * oz,
    )


def rotate_vector(q, x, y, z):
    # Quaternion-vector rotation without adding a numpy dependency.
    tx = 2.0 * (q.y * z - q.z * y)
    ty = 2.0 * (q.z * x - q.x * z)
    tz = 2.0 * (q.x * y - q.y * x)
    return (
        x + q.w * tx + q.y * tz - q.z * ty,
        y + q.w * ty + q.z * tx - q.x * tz,
        z + q.w * tz + q.x * ty - q.y * tx,
    )


class NavTfDirectRelay(Node):
    def __init__(self):
        super().__init__("nav_tf_direct_relay")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        self.tf_topic = self.declare_parameter("tf_topic", "/nav_tf").value
        reliability = str(self.declare_parameter(
            "tf_reliability", "reliable").value).lower()
        self.roll_offset = float(self.declare_parameter(
            "localization_to_base_roll_rad", 0.0).value)
        self.pitch_offset = float(self.declare_parameter(
            "localization_to_base_pitch_rad", 0.0).value)
        self.yaw_offset = float(self.declare_parameter(
            "localization_to_base_yaw_rad", 0.0).value)
        self.offset_x = float(self.declare_parameter(
            "localization_to_base_x_m", 0.0).value)
        self.offset_y = float(self.declare_parameter(
            "localization_to_base_y_m", 0.0).value)
        self.offset_z = float(self.declare_parameter(
            "localization_to_base_z_m", 0.0).value)
        self.rotation_offset = quaternion_from_rpy(
            self.roll_offset, self.pitch_offset, self.yaw_offset)
        tf_reliability = (
            ReliabilityPolicy.BEST_EFFORT
            if reliability in ("best_effort", "besteffort", "best-effort")
            else ReliabilityPolicy.RELIABLE
        )
        tf_qos = QoSProfile(depth=20, reliability=tf_reliability)
        pose_qos = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(TFMessage, self.tf_topic, tf_qos)
        self.last_pose = None
        self.seen_pose = False
        self.publish_count = 0
        self.create_subscription(
            PoseStamped, "/relocalization_pose", self.on_pose, pose_qos)
        self.create_subscription(
            PoseStamped, "/initialpose", self.on_initialpose, QoSProfile(depth=10))
        self.create_timer(0.02, self.publish_tf)
        self.create_timer(2.0, self.log_health)
        self.get_logger().info(
            f"publishing {self.map_frame}->{self.base_frame} on {self.tf_topic}, "
            f"reliability={tf_reliability.name}, "
            f"rpy offset=({self.roll_offset:.3f},"
            f"{self.pitch_offset:.3f},{self.yaw_offset:.3f})")

    def on_pose(self, msg):
        self.last_pose = msg
        if not self.seen_pose:
            self.seen_pose = True
            self.get_logger().info(
                f"first /relocalization_pose: x={msg.pose.position.x:.3f}, "
                f"y={msg.pose.position.y:.3f}")

    def on_initialpose(self, msg):
        self.get_logger().info(
            "received RViz /initialpose; interpret the arrow as the physical "
            "base_link heading. The NDT localizer converts it internally.")

    def publish_tf(self):
        if self.last_pose is None:
            return
        pose = self.last_pose.pose
        qx, qy, qz, qw = multiply_quaternions(
            pose.orientation, self.rotation_offset)
        dx, dy, dz = rotate_vector(
            pose.orientation, self.offset_x, self.offset_y, self.offset_z)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = self.map_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = pose.position.x + dx
        tf.transform.translation.y = pose.position.y + dy
        tf.transform.translation.z = pose.position.z + dz
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.pub.publish(TFMessage(transforms=[tf]))
        self.publish_count += 1

    def log_health(self):
        if self.last_pose is None:
            self.get_logger().warn(
                "waiting for /relocalization_pose before publishing TF")
            return
        self.get_logger().info(
            f"healthy: published {self.publish_count} TF messages on {self.tf_topic}")


def main():
    rclpy.init()
    node = NavTfDirectRelay()
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
