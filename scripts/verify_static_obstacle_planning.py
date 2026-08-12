#!/usr/bin/env python3
"""Prove Nav2 detours around a lethal static/global-costmap obstacle."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Verifier(Node):
    def __init__(self):
        super().__init__("verify_static_obstacle_planning")
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grid = None
        self.create_subscription(
            OccupancyGrid, "/global_costmap/costmap", self.on_grid, map_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose")

    def on_grid(self, message):
        self.grid = message

    def base_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", Time(), timeout=Duration(seconds=0.1))
        except TransformException:
            return None
        translation = transform.transform.translation
        return float(translation.x), float(translation.y)


def grid_cost(grid, x, y):
    info = grid.info
    cx = int(math.floor((x - info.origin.position.x) / info.resolution))
    cy = int(math.floor((y - info.origin.position.y) / info.resolution))
    if cx < 0 or cy < 0 or cx >= info.width or cy >= info.height:
        return None
    return int(grid.data[cy * info.width + cx])


def segment_costs(grid, start, goal):
    distance = math.hypot(goal[0] - start[0], goal[1] - start[1])
    samples = max(2, int(math.ceil(distance / (grid.info.resolution * 0.5))))
    values = []
    for index in range(samples + 1):
        ratio = index / samples
        x = start[0] + ratio * (goal[0] - start[0])
        y = start[1] + ratio * (goal[1] - start[1])
        values.append((ratio, grid_cost(grid, x, y)))
    return values


def candidate_goals(grid, start):
    for radius in (1.5, 2.0, 2.5, 3.0, 4.0, 5.0):
        for degrees in range(-180, 180, 10):
            angle = math.radians(degrees)
            goal = (
                start[0] + radius * math.cos(angle),
                start[1] + radius * math.sin(angle),
            )
            target_cost = grid_cost(grid, *goal)
            if target_cost is None or target_cost < 0 or target_cost > 50:
                continue
            costs = segment_costs(grid, start, goal)
            blocked = [
                cost for ratio, cost in costs
                if 0.20 <= ratio <= 0.90 and cost is not None and cost >= 99
            ]
            if len(blocked) >= 2:
                yield goal, radius, degrees, len(blocked)


def make_goal(node, x, y):
    pose = PoseStamped()
    pose.header.frame_id = "map"
    pose.header.stamp = node.get_clock().now().to_msg()
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.orientation.w = 1.0
    goal = ComputePathToPose.Goal()
    goal.goal = pose
    goal.planner_id = "GridBased"
    goal.use_start = False
    return goal


def path_length(poses):
    return sum(
        math.hypot(
            second.pose.position.x - first.pose.position.x,
            second.pose.position.y - first.pose.position.y,
        )
        for first, second in zip(poses, poses[1:])
    )


def main():
    rclpy.init()
    node = Verifier()
    deadline = time.monotonic() + 20.0
    start = None
    try:
        while time.monotonic() < deadline and (
                node.grid is None or start is None):
            rclpy.spin_once(node, timeout_sec=0.2)
            start = node.base_pose()
        if node.grid is None or start is None:
            raise RuntimeError("global costmap or map->base_link TF unavailable")
        if not node.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/compute_path_to_pose unavailable")

        attempted = 0
        for goal, direct, degrees, blocked_cells in candidate_goals(
                node.grid, start):
            attempted += 1
            future = node.client.send_goal_async(
                make_goal(node, goal[0], goal[1]))
            rclpy.spin_until_future_complete(node, future, timeout_sec=8.0)
            handle = future.result()
            if handle is None or not handle.accepted:
                continue
            result_future = handle.get_result_async()
            rclpy.spin_until_future_complete(
                node, result_future, timeout_sec=12.0)
            wrapped = result_future.result()
            if wrapped is None or len(wrapped.result.path.poses) < 2:
                continue
            poses = wrapped.result.path.poses
            lethal_hits = sum(
                (grid_cost(
                    node.grid,
                    pose.pose.position.x,
                    pose.pose.position.y,
                ) or 0) >= 99
                for pose in poses
            )
            planned = path_length(poses)
            if lethal_hits == 0 and planned > direct + 0.05:
                print(
                    "STATIC_OBSTACLE_PLANNING PASS "
                    f"start=({start[0]:.3f},{start[1]:.3f}) "
                    f"goal=({goal[0]:.3f},{goal[1]:.3f}) "
                    f"bearing={degrees:+d}deg direct={direct:.3f}m "
                    f"straight_lethal_samples={blocked_cells} "
                    f"planned={planned:.3f}m path_points={len(poses)} "
                    "planned_lethal_hits=0 physical_motion=DISARMED")
                return
            if attempted >= 16:
                break
        raise RuntimeError(
            f"no verified obstacle-detour plan after {attempted} candidates")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except Exception as error:  # Keep a single actionable line for shell logs.
        print(f"STATIC_OBSTACLE_PLANNING FAIL {error}")
        raise SystemExit(1)
