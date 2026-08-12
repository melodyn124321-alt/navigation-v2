#!/usr/bin/env python3
"""Request a Nav2 path without moving the chassis and summarize its direction."""

import argparse
import math
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import ComputePathThroughPoses
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


def quaternion_yaw(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z
               + quaternion.x * quaternion.y),
        1.0 - 2.0 * (quaternion.y * quaternion.y
                     + quaternion.z * quaternion.z),
    )


def make_pose(node, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


class PlanProbe(Node):
    def __init__(self):
        super().__init__("nav_plan_probe")
        self.client = ActionClient(
            self, ComputePathThroughPoses, "/compute_path_through_poses")
        self.costmap = None
        costmap_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid,
            "/global_costmap/costmap",
            self.on_costmap,
            costmap_qos,
        )

    def on_costmap(self, message):
        self.costmap = message


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("--approach-x", type=float, required=True)
    parser.add_argument("--approach-y", type=float, required=True)
    parser.add_argument("--goal-x", type=float, required=True)
    parser.add_argument("--goal-y", type=float, required=True)
    parser.add_argument("--yaw", type=float, required=True)
    parser.add_argument("--start-x", type=float)
    parser.add_argument("--start-y", type=float)
    parser.add_argument("--start-yaw", type=float)
    parser.add_argument("--planner-id", default="GridBased")
    parser.add_argument("--approach-only", action="store_true")
    parser.add_argument("--timeout", type=float, default=10.0)
    return parser.parse_args()


def grid_cost(grid, x, y):
    column = int((x - grid.info.origin.position.x) / grid.info.resolution)
    row = int((y - grid.info.origin.position.y) / grid.info.resolution)
    if column < 0 or row < 0:
        return None
    if column >= grid.info.width or row >= grid.info.height:
        return None
    return int(grid.data[row * grid.info.width + column])


def summarize(path, costmap):
    poses = path.poses
    if len(poses) < 2:
        return "PLAN_INVALID points<2", 2

    path_length = 0.0
    reverse_length = 0.0
    reverse_segments = 0
    forward_segments = 0
    heading_changes = []
    segment_headings = []

    for first, second in zip(poses, poses[1:]):
        dx = second.pose.position.x - first.pose.position.x
        dy = second.pose.position.y - first.pose.position.y
        length = math.hypot(dx, dy)
        if length < 1.0e-6:
            continue
        segment_heading = math.atan2(dy, dx)
        pose_heading = quaternion_yaw(first.pose.orientation)
        direction = math.cos(segment_heading - pose_heading)
        path_length += length
        segment_headings.append(segment_heading)
        if direction < 0.0:
            reverse_segments += 1
            reverse_length += length
        else:
            forward_segments += 1

    for first, second in zip(segment_headings, segment_headings[1:]):
        heading_changes.append(abs(math.atan2(
            math.sin(second - first), math.cos(second - first))))
    curved = sum(heading_changes) > 0.12
    path_costs = [
        grid_cost(costmap, pose.pose.position.x, pose.pose.position.y)
        for pose in poses
    ]
    outside_costmap = sum(value is None for value in path_costs)
    lethal_hits = sum(
        value is not None and value >= 99 for value in path_costs)
    known_costs = [value for value in path_costs if value is not None]
    max_cost = max(known_costs, default=-1)
    direction = (
        "FORWARD_ONLY" if reverse_segments == 0
        else "REVERSE_STRAIGHT" if not curved
        else "REVERSE_CURVED"
    )
    text = (
        f"PLAN_OK direction={direction} points={len(poses)} "
        f"start=({poses[0].pose.position.x:.3f},"
        f"{poses[0].pose.position.y:.3f}) "
        f"end=({poses[-1].pose.position.x:.3f},"
        f"{poses[-1].pose.position.y:.3f}) "
        f"length={path_length:.3f}m forward_segments={forward_segments} "
        f"reverse_segments={reverse_segments} "
        f"reverse_length={reverse_length:.3f}m "
        f"heading_change_sum={sum(heading_changes):.3f}rad "
        f"heading_change_max={max(heading_changes, default=0.0):.3f}rad "
        f"first_heading={segment_headings[0]:+.3f}rad "
        f"last_heading={segment_headings[-1]:+.3f}rad "
        f"lethal_hits={lethal_hits} max_cost={max_cost} "
        f"outside_costmap={outside_costmap}"
    )
    safe = lethal_hits == 0 and outside_costmap == 0
    return text, 0 if direction != "REVERSE_CURVED" and safe else 3


def main():
    args = parse_arguments()
    rclpy.init()
    node = PlanProbe()
    try:
        if not node.client.wait_for_server(timeout_sec=args.timeout):
            print("PLAN_FAILED compute_path_through_poses unavailable")
            return 4
        deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
        while node.costmap is None \
                and node.get_clock().now().nanoseconds < deadline:
            rclpy.spin_once(node, timeout_sec=0.2)
        if node.costmap is None:
            print("PLAN_FAILED global costmap unavailable")
            return 7
        goal = ComputePathThroughPoses.Goal()
        goal.goals = [make_pose(
            node, args.approach_x, args.approach_y, args.yaw)]
        if not args.approach_only:
            goal.goals.append(make_pose(
                node, args.goal_x, args.goal_y, args.yaw))
        goal.planner_id = args.planner_id
        explicit_start = (
            args.start_x is not None
            and args.start_y is not None
            and args.start_yaw is not None
        )
        if explicit_start:
            goal.start = make_pose(
                node, args.start_x, args.start_y, args.start_yaw)
        goal.use_start = explicit_start
        send_future = node.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(
            node, send_future, timeout_sec=args.timeout)
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("PLAN_FAILED goal rejected")
            return 5
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(
            node, result_future, timeout_sec=args.timeout)
        wrapped = result_future.result()
        if wrapped is None:
            print("PLAN_FAILED result timeout")
            return 6
        text, exit_code = summarize(wrapped.result.path, node.costmap)
        print(text)
        return exit_code
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
