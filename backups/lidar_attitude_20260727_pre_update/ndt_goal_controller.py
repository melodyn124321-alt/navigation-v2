#!/usr/bin/env python3
"""Small NDT-based goal follower for safe short-range robot motion.

This is intentionally simpler than Nav2: RViz publishes /goal_pose in map,
NDT publishes /relocalization_pose, and this node outputs a conservative twist.
The existing nav_motion_safety_gate remains the last authority before /cmd_vel.
"""

import copy
import math

import rclpy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


class NdtGoalController(Node):
    def __init__(self):
        super().__init__("ndt_goal_controller")
        self.map_frame = self.declare_parameter("map_frame", "map").value
        self.output_topic = self.declare_parameter(
            "output_topic", "/cmd_vel_nav_smoothed").value
        self.localization_to_base_yaw = float(self.declare_parameter(
            "localization_to_base_yaw_rad", -0.01491409).value)
        self.localization_to_base_x = float(self.declare_parameter(
            "localization_to_base_x_m", -0.5105783).value)
        self.localization_to_base_y = float(self.declare_parameter(
            "localization_to_base_y_m", 0.0822838).value)
        self.max_linear = float(self.declare_parameter("max_linear", 0.08).value)
        self.max_angular = float(self.declare_parameter("max_angular", 0.18).value)
        self.goal_tolerance = float(self.declare_parameter(
            "goal_tolerance", 0.08).value)
        self.goal_label_offset_x = float(self.declare_parameter(
            "goal_label_offset_x", 0.30).value)
        self.goal_label_offset_y = float(self.declare_parameter(
            "goal_label_offset_y", 0.30).value)
        self.align_final_heading = bool(self.declare_parameter(
            "align_final_heading", False).value)
        self.yaw_tolerance = float(self.declare_parameter(
            "yaw_tolerance", 0.08).value)
        self.final_yaw_stable_time = float(self.declare_parameter(
            "final_yaw_stable_time", 0.60).value)
        self.final_yaw_timeout = float(self.declare_parameter(
            "final_yaw_timeout", 45.0).value)
        self.max_final_angular = float(self.declare_parameter(
            "max_final_angular", 0.20).value)
        self.k_final_angular = float(self.declare_parameter(
            "k_final_angular", 0.60).value)
        self.final_yaw_filter_alpha = float(self.declare_parameter(
            "final_yaw_filter_alpha", 0.25).value)
        self.rotate_first_angle = float(self.declare_parameter(
            "rotate_first_angle", 0.45).value)
        self.localization_timeout = float(self.declare_parameter(
            "localization_timeout", 0.50).value)
        self.k_linear = float(self.declare_parameter("k_linear", 0.35).value)
        self.k_angular = float(self.declare_parameter("k_angular", 0.9).value)

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            PoseStamped, "/relocalization_pose", self.on_pose, sensor_qos)
        self.create_subscription(PoseStamped, "/goal_pose", self.on_goal, 10)
        self.cmd_pub = self.create_publisher(Twist, self.output_topic, 10)
        self.path_pub = self.create_publisher(Path, "/ndt_goal_path", 10)
        self.status_pub = self.create_publisher(String, "/ndt_goal_status", 10)
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(
            MarkerArray, "/ndt_goal_markers", marker_qos)

        self.pose = None
        self.pose_time = None
        self.goal = None
        self.goal_number = 0
        self.position_reached = False
        self.final_align_start = None
        self.final_yaw_stable_start = None
        self.filtered_final_yaw_error = None
        self.last_final_yaw_error = None
        self.unwrapped_final_yaw_error = None
        self.last_status = None
        self.create_timer(0.05, self.tick)
        self.create_timer(1.0, self.publish_status_heartbeat)
        self.clear_goal_markers()
        self.get_logger().info(
            f"ready: /goal_pose -> {self.output_topic}, "
            f"limits=({self.max_linear:.2f} m/s, {self.max_angular:.2f} rad/s)")

    def on_pose(self, msg):
        self.pose = msg
        self.pose_time = self.get_clock().now()

    def on_goal(self, msg):
        if msg.header.frame_id and msg.header.frame_id != self.map_frame:
            self.set_status(
                f"ignored goal frame={msg.header.frame_id}; expected {self.map_frame}")
            return
        self.goal = msg
        self.goal_number += 1
        self.position_reached = False
        self.final_align_start = None
        self.final_yaw_stable_start = None
        self.filtered_final_yaw_error = None
        self.last_final_yaw_error = None
        self.unwrapped_final_yaw_error = None
        self.publish_path()
        self.publish_goal_markers("ACTIVE")
        self.set_status(
            f"goal accepted x={msg.pose.position.x:.2f} y={msg.pose.position.y:.2f}")

    def elapsed_pose(self):
        if self.pose_time is None:
            return math.inf
        return (self.get_clock().now() - self.pose_time).nanoseconds / 1e9

    def base_yaw(self):
        return wrap(yaw_from_quat(self.pose.pose.orientation) + self.localization_to_base_yaw)

    def base_xy(self):
        yaw = yaw_from_quat(self.pose.pose.orientation)
        dx = (
            math.cos(yaw) * self.localization_to_base_x
            - math.sin(yaw) * self.localization_to_base_y)
        dy = (
            math.sin(yaw) * self.localization_to_base_x
            + math.cos(yaw) * self.localization_to_base_y)
        return self.pose.pose.position.x + dx, self.pose.pose.position.y + dy

    def set_status(self, text):
        self.status_pub.publish(String(data=text))
        if text != self.last_status:
            self.get_logger().info(text)
        self.last_status = text

    def publish_status_heartbeat(self):
        if self.last_status is not None:
            text = self.last_status
        elif self.pose is None:
            text = "waiting for localization"
        else:
            text = "ready: waiting for /goal_pose"
        self.status_pub.publish(String(data=text))

    def publish_path(self):
        if self.pose is None or self.goal is None:
            return
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.map_frame
        base_pose = copy.deepcopy(self.pose)
        base_pose.pose.position.x, base_pose.pose.position.y = self.base_xy()
        path.poses = [base_pose, self.goal]
        self.path_pub.publish(path)

    def clear_goal_markers(self):
        stamp = self.get_clock().now().to_msg()
        markers = MarkerArray()
        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)
        self.marker_pub.publish(markers)

    def publish_goal_markers(self, state):
        if self.goal is None:
            return
        stamp = self.get_clock().now().to_msg()
        if state == "ACTIVE":
            marker_color = (1.0, 0.1, 0.8)
        elif state == "ALIGNING":
            marker_color = (1.0, 0.85, 0.0)
        elif state == "REACHED":
            marker_color = (0.1, 1.0, 0.1)
        else:
            marker_color = (1.0, 0.35, 0.0)
        markers = MarkerArray()

        clear = Marker()
        clear.header.frame_id = self.map_frame
        clear.header.stamp = stamp
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        body = Marker()
        body.header.frame_id = self.map_frame
        body.header.stamp = stamp
        body.ns = "ndt_goal"
        body.id = 1
        body.type = Marker.CYLINDER
        body.action = Marker.ADD
        body.pose = copy.deepcopy(self.goal.pose)
        body.pose.position.z = 0.12
        body.scale.x = 0.32
        body.scale.y = 0.32
        body.scale.z = 0.24
        body.color.r, body.color.g, body.color.b = marker_color
        body.color.a = 0.95
        markers.markers.append(body)

        arrow = Marker()
        arrow.header = body.header
        arrow.ns = "ndt_goal"
        arrow.id = 2
        arrow.type = Marker.ARROW
        arrow.action = Marker.ADD
        arrow.pose = copy.deepcopy(self.goal.pose)
        arrow.pose.position.z = 0.32
        arrow.scale.x = 0.65
        arrow.scale.y = 0.12
        arrow.scale.z = 0.12
        arrow.color.r = body.color.r
        arrow.color.g = body.color.g
        arrow.color.b = body.color.b
        arrow.color.a = 1.0
        markers.markers.append(arrow)

        ring = Marker()
        ring.header = body.header
        ring.ns = "ndt_goal"
        ring.id = 3
        ring.type = Marker.LINE_STRIP
        ring.action = Marker.ADD
        ring.pose.orientation.w = 1.0
        ring.scale.x = 0.035
        ring.color.r = body.color.r
        ring.color.g = body.color.g
        ring.color.b = body.color.b
        ring.color.a = 0.9
        for index in range(49):
            angle = 2.0 * math.pi * index / 48.0
            ring.points.append(Point(
                x=self.goal.pose.position.x + self.goal_tolerance * math.cos(angle),
                y=self.goal.pose.position.y + self.goal_tolerance * math.sin(angle),
                z=0.03,
            ))
        markers.markers.append(ring)

        text = Marker()
        text.header = body.header
        text.ns = "ndt_goal"
        text.id = 4
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose.position.x = (
            self.goal.pose.position.x + self.goal_label_offset_x)
        text.pose.position.y = (
            self.goal.pose.position.y + self.goal_label_offset_y)
        text.pose.position.z = 0.55
        text.pose.orientation.w = 1.0
        text.scale.z = 0.22
        text.color.r = body.color.r
        text.color.g = body.color.g
        text.color.b = body.color.b
        text.color.a = 1.0
        text.text = f"GOAL #{self.goal_number} {state}"
        markers.markers.append(text)

        leader = Marker()
        leader.header = body.header
        leader.ns = "ndt_goal"
        leader.id = 5
        leader.type = Marker.LINE_STRIP
        leader.action = Marker.ADD
        leader.pose.orientation.w = 1.0
        leader.scale.x = 0.025
        leader.color.r = body.color.r
        leader.color.g = body.color.g
        leader.color.b = body.color.b
        leader.color.a = 0.8
        leader.points = [
            Point(
                x=self.goal.pose.position.x,
                y=self.goal.pose.position.y,
                z=0.24,
            ),
            Point(
                x=text.pose.position.x,
                y=text.pose.position.y,
                z=0.45,
            ),
        ]
        markers.markers.append(leader)

        self.marker_pub.publish(markers)

    def stop(self, reason):
        self.cmd_pub.publish(Twist())
        self.set_status(reason)

    def elapsed_since(self, stamp):
        if stamp is None:
            return 0.0
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def finish_goal(self, reason, marker_state="REACHED"):
        self.publish_goal_markers(marker_state)
        self.goal = None
        self.position_reached = False
        self.final_align_start = None
        self.final_yaw_stable_start = None
        self.filtered_final_yaw_error = None
        self.last_final_yaw_error = None
        self.unwrapped_final_yaw_error = None
        self.stop(reason)

    def align_goal_heading(self, final_yaw_error):
        now = self.get_clock().now()
        if self.final_align_start is None:
            self.final_align_start = now
            self.filtered_final_yaw_error = final_yaw_error
            self.last_final_yaw_error = final_yaw_error
            self.unwrapped_final_yaw_error = final_yaw_error
            self.publish_goal_markers("ALIGNING")

        if self.elapsed_since(self.final_align_start) > self.final_yaw_timeout:
            self.finish_goal(
                f"goal stopped: final heading timeout error={final_yaw_error:.2f}",
                "HEADING TIMEOUT",
            )
            return

        # Keep the error continuous across the -pi/+pi boundary. Without this,
        # an almost 180-degree target can alternate the commanded turn direction.
        raw_delta = wrap(final_yaw_error - self.last_final_yaw_error)
        self.unwrapped_final_yaw_error += raw_delta
        self.last_final_yaw_error = final_yaw_error
        self.filtered_final_yaw_error += self.final_yaw_filter_alpha * (
            self.unwrapped_final_yaw_error - self.filtered_final_yaw_error)

        if abs(self.filtered_final_yaw_error) <= self.yaw_tolerance:
            self.cmd_pub.publish(Twist())
            if self.final_yaw_stable_start is None:
                self.final_yaw_stable_start = now
            stable_for = self.elapsed_since(self.final_yaw_stable_start)
            if stable_for >= self.final_yaw_stable_time:
                self.finish_goal(
                    "goal reached: position 0.08 m and heading 0.08 rad")
                return
            self.set_status(
                f"stabilizing final heading error={self.filtered_final_yaw_error:.2f} "
                f"stable={stable_for:.2f}/{self.final_yaw_stable_time:.2f}s")
            return

        self.final_yaw_stable_start = None
        cmd = Twist()
        cmd.angular.z = max(
            -self.max_final_angular,
            min(
                self.max_final_angular,
                self.k_final_angular * self.filtered_final_yaw_error,
            ),
        )
        self.cmd_pub.publish(cmd)
        self.set_status(
            f"aligning final heading error={self.filtered_final_yaw_error:.2f}")

    def tick(self):
        if self.goal is None:
            self.cmd_pub.publish(Twist())
            return
        if self.pose is None or self.elapsed_pose() > self.localization_timeout:
            self.stop("blocked: stale localization")
            return

        base_x, base_y = self.base_xy()
        dx = self.goal.pose.position.x - base_x
        dy = self.goal.pose.position.y - base_y
        distance = math.hypot(dx, dy)
        target_yaw = math.atan2(dy, dx)
        heading_error = wrap(target_yaw - self.base_yaw())
        goal_yaw = yaw_from_quat(self.goal.pose.orientation)
        final_yaw_error = wrap(goal_yaw - self.base_yaw())

        if not self.position_reached and distance <= self.goal_tolerance:
            self.position_reached = True
            if not self.align_final_heading:
                self.finish_goal(
                    f"goal reached: position tolerance {self.goal_tolerance:.2f} m")
                return

        if self.position_reached:
            self.align_goal_heading(final_yaw_error)
            return

        cmd = Twist()
        if abs(heading_error) > self.rotate_first_angle:
            cmd.angular.z = max(
                -self.max_angular,
                min(self.max_angular, self.k_angular * heading_error),
            )
            self.set_status(
                f"rotating to target distance={distance:.2f} heading_error={heading_error:.2f}")
        else:
            linear_limit = self.max_linear * max(0.20, 1.0 - abs(heading_error))
            cmd.linear.x = max(0.0, min(linear_limit, self.k_linear * distance))
            cmd.angular.z = max(
                -self.max_angular,
                min(self.max_angular, self.k_angular * heading_error),
            )
            self.set_status(
                f"tracking distance={distance:.2f} heading_error={heading_error:.2f}")
        self.cmd_pub.publish(cmd)
        self.publish_path()


def main():
    rclpy.init()
    node = NdtGoalController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
