#!/usr/bin/env python3
import math

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


def quat_to_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    xx, yy, zz = x * x * s, y * y * s, z * z * s
    xy, xz, yz = x * y * s, x * z * s, y * z * s
    wx, wy, wz = w * x * s, w * y * s, w * z * s
    return np.array([
        [1.0 - yy - zz, xy - wz, xz + wy],
        [xy + wz, 1.0 - xx - zz, yz - wx],
        [xz - wy, yz + wx, 1.0 - xx - yy],
    ])


def matrix_to_quat(rot):
    trace = float(np.trace(rot))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (rot[2, 1] - rot[1, 2]) / s
        y = (rot[0, 2] - rot[2, 0]) / s
        z = (rot[1, 0] - rot[0, 1]) / s
    else:
        i = int(np.argmax([rot[0, 0], rot[1, 1], rot[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + rot[0, 0] - rot[1, 1] - rot[2, 2]) * 2.0
            w = (rot[2, 1] - rot[1, 2]) / s
            x = 0.25 * s
            y = (rot[0, 1] + rot[1, 0]) / s
            z = (rot[0, 2] + rot[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 + rot[1, 1] - rot[0, 0] - rot[2, 2]) * 2.0
            w = (rot[0, 2] - rot[2, 0]) / s
            x = (rot[0, 1] + rot[1, 0]) / s
            y = 0.25 * s
            z = (rot[1, 2] + rot[2, 1]) / s
        else:
            s = math.sqrt(1.0 + rot[2, 2] - rot[0, 0] - rot[1, 1]) * 2.0
            w = (rot[1, 0] - rot[0, 1]) / s
            x = (rot[0, 2] + rot[2, 0]) / s
            y = (rot[1, 2] + rot[2, 1]) / s
            z = 0.25 * s
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    return x / norm, y / norm, z / norm, w / norm


def rpy_to_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def pose_to_matrix(msg):
    mat = np.eye(4)
    mat[:3, :3] = quat_to_matrix(msg.pose.orientation)
    mat[0, 3] = msg.pose.position.x
    mat[1, 3] = msg.pose.position.y
    mat[2, 3] = msg.pose.position.z
    return mat


def transform_to_matrix(msg):
    mat = np.eye(4)
    mat[:3, :3] = quat_to_matrix(msg.transform.rotation)
    mat[0, 3] = msg.transform.translation.x
    mat[1, 3] = msg.transform.translation.y
    mat[2, 3] = msg.transform.translation.z
    return mat


class MapOdomTfBridge(Node):
    def __init__(self):
        super().__init__("map_odom_tf_bridge")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.odom_frame = self.declare_parameter("odom_frame", "odom").value
        self.base_frame = self.declare_parameter("base_frame", "base_link").value
        # NDT aligns Fast-LIO's lidar/body frame. Apply the calibrated
        # localization->base transform before deriving map->odom.
        self.localization_to_base_roll_rad = float(
            self.declare_parameter(
                "localization_to_base_roll_rad", -0.274595872).value)
        self.localization_to_base_pitch_rad = float(
            self.declare_parameter(
                "localization_to_base_pitch_rad", 0.010270063).value)
        self.localization_to_base_yaw_rad = float(
            self.declare_parameter(
                "localization_to_base_yaw_rad", -0.002893145).value)
        self.localization_to_base_x_m = float(
            self.declare_parameter(
                "localization_to_base_x_m", -0.897418044).value)
        self.localization_to_base_y_m = float(
            self.declare_parameter(
                "localization_to_base_y_m", -0.139088063).value)
        self.localization_to_base_z_m = float(
            self.declare_parameter(
                "localization_to_base_z_m", -0.678155856).value)
        # Nav2 queries TF at the current ROS time.  Future-dated transforms
        # can make a freshly created Nav2 buffer intermittently report that
        # the map frame does not exist.
        self.future_stamp_sec = float(
            self.declare_parameter("future_stamp_sec", 0.0).value)
        self.pose_topic = self.declare_parameter(
            "pose_topic", "/relocalization_pose").value
        self.publish_rate = float(
            self.declare_parameter("publish_rate", 10.0).value)
        self.last_pose = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_pub = TransformBroadcaster(self)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.sub = self.create_subscription(
            PoseStamped, self.pose_topic, self.on_pose, qos)
        self.timer = self.create_timer(
            1.0 / max(1.0, self.publish_rate), self.publish_map_odom)
        self.get_logger().info(
            f"bridging {self.pose_topic} and "
            f"{self.odom_frame}->{self.base_frame} into "
            f"{self.map_frame}->{self.odom_frame}, "
            f"future_stamp_sec={self.future_stamp_sec:.2f}, "
            f"publish_rate={self.publish_rate:.1f} Hz, "
            "localization_to_base_rpy_rad=("
            f"{self.localization_to_base_roll_rad:.4f},"
            f"{self.localization_to_base_pitch_rad:.4f},"
            f"{self.localization_to_base_yaw_rad:.4f})")

    def on_pose(self, msg):
        self.last_pose = msg

    def publish_map_odom(self):
        if self.last_pose is None:
            return
        try:
            odom_t_base = self.tf_buffer.lookup_transform(
                self.odom_frame,
                self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.02),
            )
        except TransformException as exc:
            self.get_logger().warn(
                f"waiting for TF {self.odom_frame}->{self.base_frame}: {exc}",
                throttle_duration_sec=2.0)
            return

        mat_map_localization = pose_to_matrix(self.last_pose)
        mat_localization_base = np.eye(4)
        mat_localization_base[:3, :3] = rpy_to_matrix(
            self.localization_to_base_roll_rad,
            self.localization_to_base_pitch_rad,
            self.localization_to_base_yaw_rad,
        )
        mat_localization_base[0, 3] = self.localization_to_base_x_m
        mat_localization_base[1, 3] = self.localization_to_base_y_m
        mat_localization_base[2, 3] = self.localization_to_base_z_m
        mat_map_base = mat_map_localization @ mat_localization_base
        mat_odom_base = transform_to_matrix(odom_t_base)
        mat_map_odom = mat_map_base @ np.linalg.inv(mat_odom_base)
        quat = matrix_to_quat(mat_map_odom[:3, :3])

        out = TransformStamped()
        out.header.stamp = (
            self.get_clock().now()
            + Duration(seconds=self.future_stamp_sec)
        ).to_msg()
        out.header.frame_id = self.map_frame
        out.child_frame_id = self.odom_frame
        out.transform.translation.x = float(mat_map_odom[0, 3])
        out.transform.translation.y = float(mat_map_odom[1, 3])
        out.transform.translation.z = float(mat_map_odom[2, 3])
        out.transform.rotation.x = quat[0]
        out.transform.rotation.y = quat[1]
        out.transform.rotation.z = quat[2]
        out.transform.rotation.w = quat[3]
        self.tf_pub.sendTransform(out)


def main():
    rclpy.init()
    node = MapOdomTfBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
