#!/usr/bin/env python3
"""Publish a chassis-aligned view of FAST-LIO's LiDAR/body odometry.

FAST-LIO's /Odometry pose is camera_init -> body, where body follows the
MID360/IMU axes.  The vehicle uses a fixed base_link -> LiDAR mounting
transform.  This node applies its inverse and publishes camera_init ->
base_link as /Odometry_chassis without changing FAST-LIO's estimator output
or TF tree.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum(value * value for value in quaternion))
    if norm < 1.0e-12:
        raise ValueError("zero-length quaternion")
    return tuple(value / norm for value in quaternion)


def quaternion_multiply(left, right):
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def quaternion_conjugate(quaternion):
    x, y, z, w = quaternion
    return (-x, -y, -z, w)


def rotate_vector(quaternion, vector):
    vector_quaternion = (vector[0], vector[1], vector[2], 0.0)
    rotated = quaternion_multiply(
        quaternion_multiply(quaternion, vector_quaternion),
        quaternion_conjugate(quaternion),
    )
    return rotated[:3]


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def compose_world_base(
    world_lidar_position,
    world_lidar_quaternion,
    base_lidar_position,
    base_lidar_quaternion,
):
    """Return T_world_base = T_world_lidar * inverse(T_base_lidar)."""
    world_lidar_quaternion = normalize_quaternion(world_lidar_quaternion)
    base_lidar_quaternion = normalize_quaternion(base_lidar_quaternion)
    lidar_base_quaternion = quaternion_conjugate(base_lidar_quaternion)
    lidar_base_position = rotate_vector(
        lidar_base_quaternion,
        tuple(-value for value in base_lidar_position),
    )
    world_base_quaternion = normalize_quaternion(
        quaternion_multiply(world_lidar_quaternion, lidar_base_quaternion)
    )
    rotated_offset = rotate_vector(
        world_lidar_quaternion, lidar_base_position)
    world_base_position = tuple(
        value + offset
        for value, offset in zip(world_lidar_position, rotated_offset)
    )
    return world_base_position, world_base_quaternion


class FastlioChassisOdometry(Node):
    def __init__(self):
        super().__init__("fastlio_chassis_odometry")
        self.input_topic = self.declare_parameter(
            "input_topic", "/Odometry").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/Odometry_chassis").value
        self.output_child_frame = self.declare_parameter(
            "output_child_frame", "base_link").value

        self.base_lidar_position = (
            float(self.declare_parameter(
                "base_to_lidar_x_m", 0.100).value),
            float(self.declare_parameter(
                "base_to_lidar_y_m", -0.110).value),
            float(self.declare_parameter(
                "base_to_lidar_z_m", 0.749).value),
        )
        roll = math.radians(float(self.declare_parameter(
                "base_to_lidar_roll_deg", -45.937).value))
        pitch = math.radians(float(self.declare_parameter(
            "base_to_lidar_pitch_deg", -0.4065).value))
        yaw = math.radians(float(self.declare_parameter(
            "base_to_lidar_yaw_deg", -90.0).value))
        self.base_lidar_quaternion = quaternion_from_rpy(roll, pitch, yaw)

        qos = QoSProfile(depth=50)
        qos.reliability = ReliabilityPolicy.RELIABLE
        self.publisher = self.create_publisher(
            Odometry, self.output_topic, qos)
        self.subscription = self.create_subscription(
            Odometry, self.input_topic, self.on_odometry, qos)
        self.get_logger().info(
            f"converting {self.input_topic} LiDAR/body pose to "
            f"{self.output_topic} chassis pose; "
            "base_link->lidar xyz=("
            f"{self.base_lidar_position[0]:.3f},"
            f"{self.base_lidar_position[1]:.3f},"
            f"{self.base_lidar_position[2]:.3f}) m "
            f"rpy=({math.degrees(roll):.3f},{math.degrees(pitch):.4f},"
            f"{math.degrees(yaw):.3f}) deg; TF is not published")

    def on_odometry(self, message):
        source_position = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            float(message.pose.pose.position.z),
        )
        source_quaternion = (
            float(message.pose.pose.orientation.x),
            float(message.pose.pose.orientation.y),
            float(message.pose.pose.orientation.z),
            float(message.pose.pose.orientation.w),
        )
        try:
            position, quaternion = compose_world_base(
                source_position,
                source_quaternion,
                self.base_lidar_position,
                self.base_lidar_quaternion,
            )
        except ValueError as error:
            self.get_logger().error(
                f"rejecting invalid FAST-LIO orientation: {error}",
                throttle_duration_sec=2.0)
            return

        output = Odometry()
        output.header = message.header
        output.child_frame_id = self.output_child_frame
        output.pose = message.pose
        output.pose.pose.position.x = position[0]
        output.pose.pose.position.y = position[1]
        output.pose.pose.position.z = position[2]
        output.pose.pose.orientation.x = quaternion[0]
        output.pose.pose.orientation.y = quaternion[1]
        output.pose.pose.orientation.z = quaternion[2]
        output.pose.pose.orientation.w = quaternion[3]
        # FAST-LIO currently leaves twist at zero. Preserve it and its
        # covariance rather than inventing chassis-frame velocities.
        output.twist = message.twist
        self.publisher.publish(output)


def main():
    rclpy.init()
    node = FastlioChassisOdometry()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
