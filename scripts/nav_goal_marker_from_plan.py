#!/usr/bin/env python3
"""Publish a persistent RViz marker for the actual Nav2 plan endpoint."""

import copy
import math

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Point, Pose, PoseStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray


STATUS_NAMES = {
    GoalStatus.STATUS_UNKNOWN: "GOAL",
    GoalStatus.STATUS_ACCEPTED: "GOAL ACCEPTED",
    GoalStatus.STATUS_EXECUTING: "GOAL ACTIVE",
    GoalStatus.STATUS_CANCELING: "GOAL CANCELING",
    GoalStatus.STATUS_SUCCEEDED: "GOAL REACHED",
    GoalStatus.STATUS_CANCELED: "GOAL CANCELED",
    GoalStatus.STATUS_ABORTED: "GOAL ABORTED",
}


class NavGoalMarker(Node):
    def __init__(self):
        super().__init__("nav_goal_marker")
        self.goal_tolerance = float(self.declare_parameter("goal_tolerance", 0.05).value)
        self.text_offset = float(self.declare_parameter("text_offset", 1.10).value)
        self.pulse_period = float(self.declare_parameter("pulse_period", 0.40).value)
        marker_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.marker_pub = self.create_publisher(MarkerArray, "/nav_goal_markers", marker_qos)
        self.status_pub = self.create_publisher(String, "/nav_goal_marker_status", marker_qos)
        self.create_subscription(Path, "/plan", self.on_plan, 10)
        self.create_subscription(
            PoseStamped, "/aligned_goal_approach_pose",
            self.on_approach_pose, marker_qos)
        self.create_subscription(
            PoseStamped, "/aligned_goal_target_pose",
            self.on_target_pose, marker_qos)
        self.create_subscription(
            GoalStatusArray,
            "/aligned_navigate_to_pose/_action/status",
            self.on_status,
            10,
        )
        self.create_service(Trigger, "/clear_nav_goal_marker", self.on_clear)
        self.goal_pose = None
        self.have_requested_target = False
        self.approach_pose = None
        self.goal_frame = "map"
        self.status = GoalStatus.STATUS_UNKNOWN
        self.pulse_on = False
        self.create_timer(self.pulse_period, self.on_pulse)
        self.get_logger().info(
            f"Waiting for /plan; goal tolerance marker={self.goal_tolerance:.2f} m")

    def on_pulse(self):
        if self.goal_pose is None or self.status != GoalStatus.STATUS_SUCCEEDED:
            return
        self.pulse_on = not self.pulse_on
        self.publish_markers()

    @staticmethod
    def yaw_from_pose(pose):
        q = pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def on_plan(self, msg):
        if not msg.poses:
            return
        # A Nav2 plan endpoint is allowed to seed a marker only when the
        # requested target has not arrived.  Its orientation is the path
        # tangent and is not necessarily the operator's 2D Goal Pose yaw.
        # Overwriting the requested target here displayed a second orange
        # arrow tens of degrees away from the heading that was actually
        # executed and NDT-confirmed.
        if not self.have_requested_target:
            self.goal_pose = copy.deepcopy(msg.poses[-1].pose)
            self.goal_frame = (
                msg.header.frame_id
                or msg.poses[-1].header.frame_id
                or "map"
            )
        self.status = GoalStatus.STATUS_EXECUTING
        self.publish_markers()

    def on_approach_pose(self, msg):
        self.approach_pose = msg.pose
        self.goal_frame = msg.header.frame_id or "map"
        if self.goal_pose is not None:
            self.publish_markers()

    def on_target_pose(self, msg):
        self.goal_pose = copy.deepcopy(msg.pose)
        self.have_requested_target = True
        self.goal_frame = msg.header.frame_id or "map"
        self.status = GoalStatus.STATUS_ACCEPTED
        self.publish_markers()

    def on_status(self, msg):
        if self.goal_pose is None or not msg.status_list:
            return
        latest = max(
            msg.status_list,
            key=lambda item: (item.goal_info.stamp.sec, item.goal_info.stamp.nanosec),
        )
        self.status = latest.status
        self.publish_markers()

    def on_clear(self, _request, response):
        delete = Marker()
        delete.action = Marker.DELETEALL
        self.marker_pub.publish(MarkerArray(markers=[delete]))
        self.goal_pose = None
        self.have_requested_target = False
        self.approach_pose = None
        self.status_pub.publish(String(data="goal marker cleared"))
        response.success = True
        response.message = "Navigation goal marker cleared"
        return response

    def color(self):
        if self.status == GoalStatus.STATUS_SUCCEEDED:
            return (0.15, 1.0, 0.20)
        if self.status == GoalStatus.STATUS_ABORTED:
            return (1.0, 0.10, 0.10)
        if self.status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_CANCELING):
            return (0.65, 0.65, 0.65)
        return (0.0, 0.85, 1.0)

    def base_marker(self, marker_id, marker_type):
        marker = Marker()
        marker.header.frame_id = self.goal_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nav_goal"
        marker.id = marker_id
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.frame_locked = True
        marker.pose.orientation.w = 1.0
        return marker

    def delete_marker(self, marker_id):
        marker = self.base_marker(marker_id, Marker.SPHERE)
        marker.action = Marker.DELETE
        return marker

    def publish_markers(self):
        if self.goal_pose is None:
            return
        pose = self.goal_pose
        yaw = self.yaw_from_pose(pose)
        r, g, b = self.color()

        center = self.base_marker(0, Marker.CYLINDER)
        center.pose.position.x = pose.position.x
        center.pose.position.y = pose.position.y
        center.pose.position.z = 0.12
        reached = self.status == GoalStatus.STATUS_SUCCEEDED
        center.scale.x = 0.34 if reached else 0.16
        center.scale.y = 0.34 if reached else 0.16
        center.scale.z = 0.22 if reached else 0.10
        center.color.r, center.color.g, center.color.b, center.color.a = r, g, b, 0.9

        arrow = self.base_marker(1, Marker.ARROW)
        arrow.pose.position.x = pose.position.x
        arrow.pose.position.y = pose.position.y
        arrow.pose.orientation.x = pose.orientation.x
        arrow.pose.orientation.y = pose.orientation.y
        arrow.pose.orientation.z = pose.orientation.z
        arrow.pose.orientation.w = pose.orientation.w
        arrow.pose.position.z = 0.12
        arrow.scale.x = 0.50
        arrow.scale.y = 0.11
        arrow.scale.z = 0.11
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 0.45, 0.0, 1.0

        ring = self.base_marker(2, Marker.LINE_STRIP)
        ring.pose.position.x = pose.position.x
        ring.pose.position.y = pose.position.y
        ring.pose.position.z = 0.09
        ring.scale.x = 0.035
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = r, g, b, 1.0
        for index in range(49):
            angle = 2.0 * math.pi * index / 48.0
            ring.points.append(Point(
                x=self.goal_tolerance * math.cos(angle),
                y=self.goal_tolerance * math.sin(angle),
                z=0.0,
            ))

        # Goal text belongs on the right side of the requested heading.  The
        # HN NDT-quality label is deliberately placed on the left side, so
        # both remain readable even after the chassis reaches the target.
        text_x = pose.position.x + math.sin(yaw) * self.text_offset
        text_y = pose.position.y - math.cos(yaw) * self.text_offset
        text = self.base_marker(3, Marker.TEXT_VIEW_FACING)
        text.pose.position.x = text_x
        text.pose.position.y = text_y
        text.pose.position.z = 1.70
        text.scale.z = 0.16 if reached else 0.14
        text.color.r, text.color.g, text.color.b, text.color.a = r, g, b, 1.0
        if reached:
            text.text = "GOAL OK"
        else:
            text.text = STATUS_NAMES.get(self.status, "GOAL")

        leader = self.base_marker(4, Marker.LINE_LIST)
        leader.scale.x = 0.025
        leader.color.r, leader.color.g, leader.color.b, leader.color.a = r, g, b, 0.8
        leader.points = [
            Point(x=pose.position.x, y=pose.position.y, z=0.14),
            Point(x=text_x, y=text_y, z=1.62),
        ]

        markers = [center, arrow, ring, text, leader]
        if reached:
            pulse_scale = 0.34 if self.pulse_on else 0.25
            beacon = self.base_marker(5, Marker.SPHERE)
            beacon.pose.position.x = pose.position.x
            beacon.pose.position.y = pose.position.y
            beacon.pose.position.z = 0.62
            beacon.scale.x = pulse_scale
            beacon.scale.y = pulse_scale
            beacon.scale.z = 0.90
            beacon.color.r = 0.10
            beacon.color.g = 1.0
            beacon.color.b = 0.15
            beacon.color.a = 0.85 if self.pulse_on else 0.55
            markers.append(beacon)

            for marker_id, radius in ((6, 0.18), (7, 0.30)):
                halo = self.base_marker(marker_id, Marker.LINE_STRIP)
                halo.pose.position.x = pose.position.x
                halo.pose.position.y = pose.position.y
                halo.pose.position.z = 0.13
                halo.scale.x = 0.055
                halo.color.r = 0.10
                halo.color.g = 1.0
                halo.color.b = 0.15
                halo.color.a = 1.0
                for index in range(49):
                    angle = 2.0 * math.pi * index / 48.0
                    halo.points.append(Point(
                        x=radius * math.cos(angle),
                        y=radius * math.sin(angle),
                        z=0.0,
                    ))
                markers.append(halo)
        else:
            markers.extend(self.delete_marker(marker_id) for marker_id in (5, 6, 7))

        if self.approach_pose is not None:
            approach = self.approach_pose
            pre_arrow = self.base_marker(8, Marker.ARROW)
            pre_arrow.pose = copy.deepcopy(approach)
            pre_arrow.pose.position.z = 0.10
            pre_arrow.scale.x = 0.34
            pre_arrow.scale.y = 0.09
            pre_arrow.scale.z = 0.09
            pre_arrow.color.r = 0.10
            pre_arrow.color.g = 0.65
            pre_arrow.color.b = 1.0
            pre_arrow.color.a = 1.0

            approach_line = self.base_marker(9, Marker.LINE_STRIP)
            approach_line.scale.x = 0.06
            approach_line.color.r = 0.10
            approach_line.color.g = 0.65
            approach_line.color.b = 1.0
            approach_line.color.a = 0.95
            approach_line.points = [
                Point(x=approach.position.x, y=approach.position.y, z=0.08),
                Point(x=pose.position.x, y=pose.position.y, z=0.08),
            ]

            # The blue approach arrow and line are sufficient; a third text
            # label near the goal adds clutter in narrow rooms.
            approach_text = self.delete_marker(10)
            markers.extend([pre_arrow, approach_line, approach_text])
        else:
            markers.extend(self.delete_marker(marker_id) for marker_id in (8, 9, 10))

        self.marker_pub.publish(MarkerArray(markers=markers))
        self.status_pub.publish(String(data=(
            f"{STATUS_NAMES.get(self.status, 'GOAL')} "
            f"x={pose.position.x:.3f} y={pose.position.y:.3f} "
            f"yaw={yaw:.3f} tolerance={self.goal_tolerance:.2f}m")))


def main():
    rclpy.init()
    node = NavGoalMarker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
