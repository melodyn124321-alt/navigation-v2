#!/usr/bin/env python3
"""Inspect Nav2 global-costmap occupancy and connectivity for goal coordinates."""

import argparse
from collections import deque
import math

import rclpy
from nav_msgs.msg import OccupancyGrid
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Inspector(Node):
    def __init__(self, map_topic, map_frame, base_frame):
        super().__init__("inspect_nav_goal_costmap")
        self.grid = None
        self.map_frame = map_frame
        self.base_frame = base_frame
        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, map_topic, self.on_grid, map_qos)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def on_grid(self, msg):
        self.grid = msg

    def base_transform(self):
        try:
            return self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                Time(),
                timeout=Duration(seconds=0.05),
            )
        except TransformException:
            return None


def cell(grid, x, y):
    info = grid.info
    return (
        int(math.floor((x - info.origin.position.x) / info.resolution)),
        int(math.floor((y - info.origin.position.y) / info.resolution)),
    )


def value(grid, point):
    x, y = point
    if x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height:
        return None
    return grid.data[y * grid.info.width + x]


def world(grid, point):
    x, y = point
    info = grid.info
    return (
        info.origin.position.x + (x + 0.5) * info.resolution,
        info.origin.position.y + (y + 0.5) * info.resolution,
    )


def passable(cost):
    return cost is not None and (cost == -1 or cost < 99)


def nearest_passable(grid, start, radius_cells=20):
    if passable(value(grid, start)):
        return start
    sx, sy = start
    best = None
    best_d2 = None
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            point = (sx + dx, sy + dy)
            if not passable(value(grid, point)):
                continue
            d2 = dx * dx + dy * dy
            if best_d2 is None or d2 < best_d2:
                best, best_d2 = point, d2
    return best


def nearest_blocked_distance(grid, start, radius_cells=40):
    sx, sy = start
    best_d2 = None
    best = None
    best_cost = None
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            point = (sx + dx, sy + dy)
            cost = value(grid, point)
            if cost is None or cost < 99:
                continue
            d2 = dx * dx + dy * dy
            if best_d2 is None or d2 < best_d2:
                best, best_cost, best_d2 = point, cost, d2
    if best is None:
        return None
    return best, best_cost, math.sqrt(best_d2) * grid.info.resolution


def blocked_component(grid, seed, limit=100000):
    if seed is None or (value(grid, seed) or 0) < 99:
        return None
    queue = deque([seed])
    seen = {seed}
    min_x = max_x = seed[0]
    min_y = max_y = seed[1]
    while queue and len(seen) < limit:
        x, y = queue.popleft()
        min_x, max_x = min(min_x, x), max(max_x, x)
        min_y, max_y = min(min_y, y), max(max_y, y)
        for nxt in (
            (x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1),
            (x - 1, y - 1), (x - 1, y + 1),
            (x + 1, y - 1), (x + 1, y + 1),
        ):
            if nxt in seen or (value(grid, nxt) or 0) < 99:
                continue
            seen.add(nxt)
            queue.append(nxt)
    return len(seen), (min_x, min_y, max_x, max_y)


def connected(grid, start, goal):
    if start is None or goal is None:
        return False, 0
    queue = deque([start])
    seen = {start}
    width, height = grid.info.width, grid.info.height
    while queue:
        point = queue.popleft()
        if point == goal:
            return True, len(seen)
        x, y = point
        for nxt in ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1)):
            nx, ny = nxt
            if nx < 0 or ny < 0 or nx >= width or ny >= height or nxt in seen:
                continue
            if passable(value(grid, nxt)):
                seen.add(nxt)
                queue.append(nxt)
    return False, len(seen)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--goal", nargs=2, type=float, action="append", default=[])
    parser.add_argument("--map-topic", default="/global_costmap/costmap")
    parser.add_argument("--map-frame", default="map")
    parser.add_argument("--base-frame", default="base_link")
    parser.add_argument("--require-start-passable", action="store_true")
    parser.add_argument("--timeout", type=float, default=15.0)
    args = parser.parse_args()
    rclpy.init()
    node = Inspector(args.map_topic, args.map_frame, args.base_frame)
    deadline = node.get_clock().now().nanoseconds / 1e9 + args.timeout
    base_tf = None
    while rclpy.ok() and (node.grid is None or base_tf is None):
        rclpy.spin_once(node, timeout_sec=0.2)
        base_tf = node.base_transform()
        if node.get_clock().now().nanoseconds / 1e9 >= deadline:
            break
    if node.grid is None or base_tf is None:
        print(f"ERROR grid={node.grid is not None} map_to_base={base_tf is not None}")
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(1)

    current = base_tf.transform.translation
    q = base_tf.transform.rotation
    current_yaw = math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )
    start_cell = cell(node.grid, current.x, current.y)
    start_free = nearest_passable(node.grid, start_cell)
    print(
        f"map={node.grid.info.width}x{node.grid.info.height} "
        f"resolution={node.grid.info.resolution:.3f}")
    start_blocked = nearest_blocked_distance(node.grid, start_cell)
    component = blocked_component(node.grid, start_blocked[0]) if start_blocked else None
    start_cost = value(node.grid, start_cell)
    print(
        f"start world=({current.x:.3f},{current.y:.3f}) yaw={current_yaw:.3f} "
        f"cell={start_cell} "
        f"cost={start_cost} nearest_passable={start_free} "
        f"nearest_world={world(node.grid, start_free) if start_free else None} "
        f"nearest_blocked={start_blocked} blocked_component={component}")
    for gx, gy in args.goal:
        goal_cell = cell(node.grid, gx, gy)
        goal_free = nearest_passable(node.grid, goal_cell)
        is_connected, visited = connected(node.grid, start_free, goal_free)
        distance = None
        if goal_free is not None:
            wx, wy = world(node.grid, goal_free)
            distance = math.hypot(wx - gx, wy - gy)
        print(
            f"goal world=({gx:.3f},{gy:.3f}) cell={goal_cell} "
            f"cost={value(node.grid, goal_cell)} nearest_passable={goal_free} "
            f"offset={distance} connected={is_connected} visited={visited}")
    node.destroy_node()
    rclpy.shutdown()
    if args.require_start_passable and (
            start_cost is None or start_cost < 0 or start_cost >= 99):
        print(
            "ERROR navigation start is occupied/inscribed/unknown; "
            "keep the robot DISARMED")
        raise SystemExit(2)


if __name__ == "__main__":
    main()
