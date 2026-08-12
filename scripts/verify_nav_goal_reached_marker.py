#!/usr/bin/env python3
"""Verify the prominent reached marker in an isolated ROS domain."""

import argparse
import math
import os
import subprocess
import sys
import time
from pathlib import Path as FilesystemPath

domain_id = os.environ.get("ROS_DOMAIN_ID", "")
if not domain_id.isdigit() or int(domain_id) == 0:
    print(
        "REFUSED: run this test in an isolated non-zero ROS_DOMAIN_ID, "
        "for example: use_fastdds_env.sh env ROS_DOMAIN_ID=77 /usr/bin/python3 ...",
        file=sys.stderr,
    )
    raise SystemExit(2)

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


class MarkerHarness(Node):
    def __init__(self):
        super().__init__("nav_goal_reached_marker_test")
        self.plan_pub = self.create_publisher(Path, "/plan", 10)
        self.action_status_pub = self.create_publisher(
            GoalStatusArray,
            "/aligned_navigate_to_pose/_action/status",
            10,
        )
        marker_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            MarkerArray, "/nav_goal_markers", self.on_markers, marker_qos)
        self.create_subscription(
            String, "/nav_goal_marker_status", self.on_status, marker_qos)
        self.marker_ids = set()
        self.texts = []
        self.text_positions = []
        self.status = ""
        self.create_timer(0.20, self.publish_reached_goal)

    def publish_reached_goal(self):
        now = self.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = now
        pose.header.frame_id = "map"
        pose.pose.position.x = 1.0
        pose.pose.position.y = 2.0
        pose.pose.orientation.w = 1.0
        path = Path()
        path.header = pose.header
        path.poses = [pose]
        self.plan_pub.publish(path)

        status = GoalStatus()
        status.goal_info.goal_id.uuid = [1] + [0] * 15
        status.goal_info.stamp = now
        status.status = GoalStatus.STATUS_SUCCEEDED
        array = GoalStatusArray()
        array.status_list = [status]
        self.action_status_pub.publish(array)

    def on_markers(self, msg):
        self.marker_ids = {marker.id for marker in msg.markers}
        self.texts = [marker.text for marker in msg.markers if marker.text]
        self.text_positions = [
            (marker.text, marker.pose.position.x, marker.pose.position.y)
            for marker in msg.markers if marker.text
        ]

    def on_status(self, msg):
        self.status = msg.data


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--marker",
        default=str(FilesystemPath(__file__).with_name("nav_goal_marker_from_plan.py")),
    )
    args = parser.parse_args()
    marker_process = subprocess.Popen(
        [
            sys.executable, args.marker, "--ros-args",
            "-p", "goal_tolerance:=0.05",
            "-p", "text_offset:=1.10",
            "-p", "pulse_period:=0.40",
        ],
        env=os.environ.copy(),
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
    )
    try:
        rclpy.init()
        node = MarkerHarness()
        deadline = time.monotonic() + 6.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        required_ids = {0, 1, 2, 3, 4, 5, 6, 7}
        reached_text = any(text == "GOAL OK" for text in node.texts)
        goal_label_offset_ok = any(
            text == "GOAL OK"
            and math.hypot(x - 1.0, y - 2.0) >= 1.00
            for text, x, y in node.text_positions
        )
        passed = (
            required_ids.issubset(node.marker_ids)
            and reached_text
            and goal_label_offset_ok
            and node.status.startswith("GOAL REACHED")
        )
        print(
            "GOAL_REACHED_MARKER_TEST "
            f"{'PASS' if passed else 'FAIL'} "
            f"ids={sorted(node.marker_ids)} "
            f"texts={node.texts} positions={node.text_positions} "
            f"status={node.status!r}"
        )
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0 if passed else 1)
    finally:
        marker_process.terminate()
        try:
            marker_process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            marker_process.kill()


if __name__ == "__main__":
    main()
