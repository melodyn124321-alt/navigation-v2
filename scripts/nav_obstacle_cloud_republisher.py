#!/usr/bin/env python3
"""Expose the Fast-LIO body cloud with its calibrated chassis mounting pose."""

import math
import struct

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


class NavObstacleCloudRepublisher(Node):
    def __init__(self):
        super().__init__("nav_obstacle_cloud_republisher")
        self.input_topic = self.declare_parameter(
            "input_topic", "/cloud_registered_body").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/nav_obstacle_cloud").value
        self.viz_output_topic = self.declare_parameter(
            "viz_output_topic", "/nav_obstacle_cloud_viz").value
        self.output_frame = self.declare_parameter(
            "output_frame", "base_link").value
        self.point_stride = max(1, int(self.declare_parameter(
            "point_stride", 4).value))
        self.lidar_x = float(self.declare_parameter(
            "lidar_x_m", 0.10).value)
        self.lidar_y = float(self.declare_parameter(
            "lidar_y_m", -0.11).value)
        self.lidar_z = float(self.declare_parameter(
            "lidar_z_m", 0.749).value)
        self.lidar_roll = float(self.declare_parameter(
            "lidar_roll_rad", math.radians(-45.937)).value)
        self.lidar_pitch = float(self.declare_parameter(
            "lidar_pitch_rad", math.radians(-0.4065)).value)
        self.lidar_yaw = float(self.declare_parameter(
            "lidar_yaw_rad", -math.pi / 2.0).value)
        self.base_rotation = self.rpy_matrix(
            self.lidar_roll, self.lidar_pitch, self.lidar_yaw)
        self.tf_buffer = Buffer(cache_time=Duration(seconds=5.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.self_min_x = float(self.declare_parameter(
            "self_min_x", -0.38).value)
        self.self_max_x = float(self.declare_parameter(
            "self_max_x", 0.38).value)
        self.self_half_width = float(self.declare_parameter(
            "self_half_width", 0.27).value)
        self.self_min_z = float(self.declare_parameter(
            "self_min_z", -0.03).value)
        self.self_max_z = float(self.declare_parameter(
            "self_max_z", 0.375).value)
        self.test_timeout = float(self.declare_parameter(
            "test_timeout_sec", 0.50).value)
        # The local/global costmap TF buffers receive odom/map transforms from
        # separate DDS participants.  A cloud stamped at the exact publish
        # instant can therefore arrive a few milliseconds ahead of the newest
        # transform and remain trapped in the message filter.  Backdating by a
        # small bounded interval keeps the sample live while making the
        # matching transform already available.
        self.output_stamp_delay = max(0.0, float(self.declare_parameter(
            "output_stamp_delay_sec", 0.15).value))
        # Keep the clearing cloud unchanged, but expose a second lightweight
        # cloud containing exactly the height/range window Nav2 can mark as an
        # obstacle.  RViz can then distinguish physical returns from inflated
        # costmap safety margins without rendering floor points as obstacles.
        self.viz_min_height = float(self.declare_parameter(
            "viz_min_height", 0.08).value)
        self.viz_max_height = float(self.declare_parameter(
            "viz_max_height", 1.60).value)
        self.viz_min_range = max(0.0, float(self.declare_parameter(
            "viz_min_range", 0.05).value))
        self.viz_max_range = max(self.viz_min_range, float(
            self.declare_parameter("viz_max_range", 5.0).value))

        # Every consumer uses sensor-data BEST_EFFORT QoS.  A depth-1 writer
        # prevents old multi-kilobyte clouds from blocking the next live scan.
        output_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(
            PointCloud2, self.output_topic, output_qos)
        self.viz_publisher = self.create_publisher(
            PointCloud2, self.viz_output_topic, output_qos)
        self.status_publisher = self.create_publisher(
            String, "/nav_obstacle_cloud_status", 1)
        input_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.subscription = self.create_subscription(
            PointCloud2,
            self.input_topic,
            self.on_cloud,
            input_qos,
        )
        self.create_subscription(
            Bool, "/nav_obstacle_test_enable", self.on_test_enable, 10)
        self.received = 0
        self.published = 0
        self.input_points = 0
        self.output_points = 0
        self.viz_points = 0
        self.last_cloud_time = None
        self.test_enabled_until = None
        self.create_timer(0.20, self.publish_status)
        self.get_logger().info(
            f"republishing {self.input_topic} as {self.output_topic} "
            f"in frame {self.output_frame}, stride={self.point_stride}, "
            f"lidar_in_base=({self.lidar_x:.3f},{self.lidar_y:.3f},"
            f"{self.lidar_z:.3f},rpy=({self.lidar_roll:.4f},"
            f"{self.lidar_pitch:.4f},{self.lidar_yaw:.4f})), "
            f"stamp_delay={self.output_stamp_delay:.3f}s, "
            f"viz=z[{self.viz_min_height:.2f},{self.viz_max_height:.2f}] "
            f"range=[{self.viz_min_range:.2f},{self.viz_max_range:.2f}], "
            f"self_box=x[{self.self_min_x:.2f},{self.self_max_x:.2f}] "
            f"y+/-{self.self_half_width:.2f}")

    @staticmethod
    def rpy_matrix(roll, pitch, yaw):
        """Return Rz(yaw) @ Ry(pitch) @ Rx(roll), matching ROS static TF."""
        cr, sr = math.cos(roll), math.sin(roll)
        cp, sp = math.cos(pitch), math.sin(pitch)
        cy, sy = math.cos(yaw), math.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr],
        ], dtype=np.float64)

    @staticmethod
    def quaternion_matrix(quaternion):
        """Return a 3-D rotation matrix for geometry_msgs/Quaternion."""
        x = float(quaternion.x)
        y = float(quaternion.y)
        z = float(quaternion.z)
        w = float(quaternion.w)
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm <= 1.0e-12:
            return np.eye(3, dtype=np.float64)
        x, y, z, w = x / norm, y / norm, z / norm, w / norm
        return np.array([
            [1.0 - 2.0 * (y * y + z * z),
             2.0 * (x * y - z * w),
             2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w),
             1.0 - 2.0 * (x * x + z * z),
             2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w),
             2.0 * (y * z + x * w),
             1.0 - 2.0 * (x * x + y * y)],
        ], dtype=np.float64)

    def on_test_enable(self, msg):
        if msg.data:
            self.test_enabled_until = (
                self.get_clock().now()
                + Duration(seconds=self.test_timeout)
            )
        else:
            self.test_enabled_until = None

    def test_injection_active(self):
        return (
            self.test_enabled_until is not None
            and self.get_clock().now() <= self.test_enabled_until
        )

    def base_to_lidar(self, x_base, y_base, z_base):
        """Convert a desired base-frame test point into the LiDAR frame."""
        delta = np.array([
            x_base - self.lidar_x,
            y_base - self.lidar_y,
            z_base - self.lidar_z,
        ])
        return tuple(self.base_rotation.T @ delta)

    @staticmethod
    def field_offset(msg, name):
        for field in msg.fields:
            if field.name == name:
                # sensor_msgs/PointField.FLOAT32
                if field.datatype != 7:
                    return None
                return field.offset
        return None

    def on_cloud(self, msg):
        self.received += 1
        if self.received == 1:
            self.get_logger().info(
                f"first cloud received: points={msg.width * msg.height} "
                f"source_frame={msg.header.frame_id}")
        x_offset = self.field_offset(msg, "x")
        y_offset = self.field_offset(msg, "y")
        z_offset = self.field_offset(msg, "z")
        if x_offset is None or y_offset is None or z_offset is None:
            self.get_logger().error(
                "PointCloud2 must contain FLOAT32 x/y/z fields",
                throttle_duration_sec=5.0)
            return

        byte_order = ">" if msg.is_bigendian else "<"
        source_count = int(msg.width) * int(msg.height)
        point_step = int(msg.point_step)
        contiguous = (
            int(msg.row_step) == int(msg.width) * point_step
            and len(msg.data) >= source_count * point_step
        )
        if contiguous and source_count > 0:
            endian = ">" if msg.is_bigendian else "<"
            indices = np.arange(0, source_count, self.point_stride, dtype=np.int64)
            x_all = np.ndarray(
                shape=(source_count,), dtype=endian + "f4", buffer=msg.data,
                offset=x_offset, strides=(point_step,))
            y_all = np.ndarray(
                shape=(source_count,), dtype=endian + "f4", buffer=msg.data,
                offset=y_offset, strides=(point_step,))
            z_all = np.ndarray(
                shape=(source_count,), dtype=endian + "f4", buffer=msg.data,
                offset=z_offset, strides=(point_step,))
            x_raw = x_all[indices]
            y_raw = y_all[indices]
            z_raw = z_all[indices]
            finite = np.isfinite(x_raw) & np.isfinite(y_raw) & np.isfinite(z_raw)
            raw_points = np.vstack((x_raw, y_raw, z_raw))
            base_points = self.base_rotation @ raw_points
            x_base = self.lidar_x + base_points[0]
            y_base = self.lidar_y + base_points[1]
            z_base = self.lidar_z + base_points[2]
            chassis_return = (
                (x_base >= self.self_min_x)
                & (x_base <= self.self_max_x)
                & (np.abs(y_base) <= self.self_half_width)
                & (z_base >= self.self_min_z)
                & (z_base <= self.self_max_z)
            )
            selected_mask = finite & ~chassis_return
            selected = indices[selected_mask]
            records = np.frombuffer(
                msg.data, dtype=np.dtype((np.void, point_step)), count=source_count)
            output_data = bytearray(records[selected].tobytes())
            output_count = int(selected.size)
            if output_count:
                x_out = np.ndarray(
                    shape=(output_count,), dtype=endian + "f4",
                    buffer=output_data, offset=x_offset, strides=(point_step,))
                y_out = np.ndarray(
                    shape=(output_count,), dtype=endian + "f4",
                    buffer=output_data, offset=y_offset, strides=(point_step,))
                z_out = np.ndarray(
                    shape=(output_count,), dtype=endian + "f4",
                    buffer=output_data, offset=z_offset, strides=(point_step,))
                x_out[:] = x_base[selected_mask]
                y_out[:] = y_base[selected_mask]
                z_out[:] = z_base[selected_mask]
        else:
            kept = []
            for index in range(0, source_count, self.point_stride):
                row, column = divmod(index, int(msg.width))
                start = row * int(msg.row_step) + column * point_step
                try:
                    x_raw = struct.unpack_from(
                        byte_order + "f", msg.data, start + x_offset)[0]
                    y_raw = struct.unpack_from(
                        byte_order + "f", msg.data, start + y_offset)[0]
                    z_raw = struct.unpack_from(
                        byte_order + "f", msg.data, start + z_offset)[0]
                except (struct.error, IndexError):
                    continue
                if not (
                    math.isfinite(x_raw)
                    and math.isfinite(y_raw)
                    and math.isfinite(z_raw)
                ):
                    continue
                base_point = self.base_rotation @ np.array(
                    [x_raw, y_raw, z_raw])
                x_base = self.lidar_x + base_point[0]
                y_base = self.lidar_y + base_point[1]
                z_base = self.lidar_z + base_point[2]
                is_chassis_return = (
                    self.self_min_x <= x_base <= self.self_max_x
                    and abs(y_base) <= self.self_half_width
                    and self.self_min_z <= z_base <= self.self_max_z
                )
                if not is_chassis_return:
                    record = bytearray(msg.data[start:start + point_step])
                    struct.pack_into(
                        byte_order + "f", record, x_offset, float(x_base))
                    struct.pack_into(
                        byte_order + "f", record, y_offset, float(y_base))
                    struct.pack_into(
                        byte_order + "f", record, z_offset, float(z_base))
                    kept.append(bytes(record))
            output_data = b"".join(kept)
            output_count = len(kept)

        if self.test_injection_active():
            test_points = []
            # Inject 0.04 m beyond the 0.38 m chassis nose and inside the
            # projected footprint at the verifier's 0.05 m/s command. Define
            # it in base_link so LiDAR mounting changes cannot move the test.
            for y_base in (-0.08, -0.04, 0.00, 0.04, 0.08):
                point = bytearray(point_step)
                struct.pack_into(byte_order + "f", point, x_offset, 0.42)
                struct.pack_into(byte_order + "f", point, y_offset, y_base)
                struct.pack_into(byte_order + "f", point, z_offset, 0.30)
                test_points.append(bytes(point))
            output_data += b"".join(test_points)
            output_count += len(test_points)

        out = PointCloud2()
        # Fast-LIO's body cloud stamp on this installation can lag wall/odom
        # time by several seconds.  Nav2 then drops an otherwise current cloud
        # as older than its TF cache, so stamp the freshly received sample now.
        out.header.stamp = (
            self.get_clock().now()
            - Duration(seconds=self.output_stamp_delay)
        ).to_msg()
        out.header.frame_id = self.output_frame
        out.height = 1
        out.width = output_count
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = point_step * output_count
        out.data = output_data
        out.is_dense = True
        self.publisher.publish(out)

        viz_data = b""
        viz_count = 0
        if output_count:
            output_records = np.frombuffer(
                output_data, dtype=np.dtype((np.void, point_step)),
                count=output_count)
            x_viz = np.ndarray(
                shape=(output_count,), dtype=byte_order + "f4",
                buffer=output_data, offset=x_offset, strides=(point_step,))
            y_viz = np.ndarray(
                shape=(output_count,), dtype=byte_order + "f4",
                buffer=output_data, offset=y_offset, strides=(point_step,))
            z_viz = np.ndarray(
                shape=(output_count,), dtype=byte_order + "f4",
                buffer=output_data, offset=z_offset, strides=(point_step,))
            range_squared = x_viz * x_viz + y_viz * y_viz
            viz_mask = (
                np.isfinite(x_viz) & np.isfinite(y_viz) & np.isfinite(z_viz)
                & (z_viz >= self.viz_min_height)
                & (z_viz <= self.viz_max_height)
                & (range_squared >= self.viz_min_range * self.viz_min_range)
                & (range_squared <= self.viz_max_range * self.viz_max_range)
            )
            viz_data = bytearray(output_records[viz_mask].tobytes())
            viz_count = int(np.count_nonzero(viz_mask))

        viz = PointCloud2()
        viz.header = out.header
        # Transform the display-only cloud to map on the Seeed host. HN RViz
        # then needs no historical cross-Wi-Fi map->base_link transform, which
        # prevents both stale pixels and MessageFilter queue drops. The
        # navigation cloud above remains in base_link with its real timestamp.
        if viz_count:
            try:
                transform = self.tf_buffer.lookup_transform(
                    "map", self.output_frame, Time())
                translation = transform.transform.translation
                map_rotation = self.quaternion_matrix(
                    transform.transform.rotation)
                x_map = np.ndarray(
                    shape=(viz_count,), dtype=byte_order + "f4",
                    buffer=viz_data, offset=x_offset, strides=(point_step,))
                y_map = np.ndarray(
                    shape=(viz_count,), dtype=byte_order + "f4",
                    buffer=viz_data, offset=y_offset, strides=(point_step,))
                z_map = np.ndarray(
                    shape=(viz_count,), dtype=byte_order + "f4",
                    buffer=viz_data, offset=z_offset, strides=(point_step,))
                mapped = map_rotation @ np.vstack((x_map, y_map, z_map))
                x_map[:] = mapped[0] + float(translation.x)
                y_map[:] = mapped[1] + float(translation.y)
                z_map[:] = mapped[2] + float(translation.z)
                viz.header.frame_id = "map"
                viz.header.stamp = self.get_clock().now().to_msg()
            except TransformException:
                # Do not publish a base-frame visualization that RViz could
                # place using an obsolete remote transform. Navigation output
                # is independent and continues normally.
                viz_count = 0
                viz_data = b""
        viz.height = 1
        viz.width = viz_count
        viz.fields = out.fields
        viz.is_bigendian = out.is_bigendian
        viz.point_step = out.point_step
        viz.row_step = point_step * viz_count
        viz.data = viz_data
        viz.is_dense = True
        self.viz_publisher.publish(viz)
        self.published += 1
        self.input_points = source_count
        self.output_points = output_count
        self.viz_points = viz_count
        self.last_cloud_time = self.get_clock().now()

    def publish_status(self):
        if self.last_cloud_time is None:
            status = "waiting for /cloud_registered_body"
        else:
            age = (
                self.get_clock().now() - self.last_cloud_time
            ).nanoseconds / 1e9
            status = (
                f"ok received={self.received} published={self.published} "
                f"age={age:.3f}s points={self.input_points}->{self.output_points} "
                f"viz={self.viz_points} viz_frame=map "
                f"frame={self.output_frame} "
                f"test_injection={self.test_injection_active()}")
        self.status_publisher.publish(String(data=status))


def main():
    rclpy.init()
    node = NavObstacleCloudRepublisher()
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
