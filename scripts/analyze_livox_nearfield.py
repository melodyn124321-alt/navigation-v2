#!/usr/bin/env python3
"""Measure persistent near-field returns in Livox CustomMsg scans."""

import argparse
import math
from collections import defaultdict, deque

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


RANGE_BINS = (0.0, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 1.0, 1.2, 1.5, 2.0, 3.0, 5.0, 10.0, 30.0, math.inf)
CUTS = (0.5, 0.6, 0.7, 0.75, 0.8, 1.0)


class NearFieldAnalyzer(Node):
    def __init__(self, frames: int, radius: float, voxel: float, persistence: float):
        super().__init__('livox_nearfield_analyzer')
        self.target_frames = frames
        self.radius = radius
        self.voxel = voxel
        self.persistence = persistence
        self.frames = 0
        self.total_points = 0
        self.hist = [0] * (len(RANGE_BINS) - 1)
        self.cut_counts = {cut: 0 for cut in CUTS}
        self.voxel_frames = defaultdict(int)
        self.voxel_points = defaultdict(int)
        self.xyz_min = [math.inf, math.inf, math.inf]
        self.xyz_max = [-math.inf, -math.inf, -math.inf]
        self.create_subscription(CustomMsg, '/livox/lidar', self.on_scan, qos_profile_sensor_data)

    def on_scan(self, msg: CustomMsg) -> None:
        if self.frames >= self.target_frames:
            return
        frame_voxels = set()
        for point in msg.points:
            x, y, z = float(point.x), float(point.y), float(point.z)
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            distance = math.sqrt(x * x + y * y + z * z)
            if distance <= 1e-4:
                continue
            self.total_points += 1
            for index in range(len(RANGE_BINS) - 1):
                if RANGE_BINS[index] <= distance < RANGE_BINS[index + 1]:
                    self.hist[index] += 1
                    break
            for cut in CUTS:
                if distance < cut:
                    self.cut_counts[cut] += 1
            if distance <= self.radius:
                key = (
                    math.floor(x / self.voxel),
                    math.floor(y / self.voxel),
                    math.floor(z / self.voxel),
                )
                frame_voxels.add(key)
                self.voxel_points[key] += 1
                for axis, value in enumerate((x, y, z)):
                    self.xyz_min[axis] = min(self.xyz_min[axis], value)
                    self.xyz_max[axis] = max(self.xyz_max[axis], value)
        for key in frame_voxels:
            self.voxel_frames[key] += 1
        self.frames += 1
        if self.frames % 10 == 0 or self.frames == self.target_frames:
            self.get_logger().info(f'captured {self.frames}/{self.target_frames} scans')

    def print_report(self) -> None:
        print(f'frames={self.frames} valid_points={self.total_points} near_radius={self.radius:.2f}m voxel={self.voxel:.3f}m')
        if self.total_points == 0:
            print('RESULT: FAIL no valid Livox points received')
            return
        print('\nDistance histogram:')
        for index, count in enumerate(self.hist):
            lo, hi = RANGE_BINS[index], RANGE_BINS[index + 1]
            hi_label = 'inf' if math.isinf(hi) else f'{hi:.2f}'
            print(f'  [{lo:5.2f}, {hi_label:>5}) m: {count:9d} ({100.0 * count / self.total_points:6.3f}%)')
        print('\nExpected point loss from a radial minimum-range filter:')
        for cut in CUTS:
            count = self.cut_counts[cut]
            print(f'  blind={cut:4.2f} m removes {count:9d}/{self.total_points} ({100.0 * count / self.total_points:6.3f}%)')

        required = max(2, math.ceil(self.frames * self.persistence))
        persistent = {key for key, seen in self.voxel_frames.items() if seen >= required}
        print(f'\nPersistent near-field voxels: {len(persistent)} (seen in >= {required}/{self.frames} scans)')
        if not persistent:
            print('  None: no static near-field obstruction met the persistence threshold.')
            return

        components = []
        remaining = set(persistent)
        neighbors = [
            (dx, dy, dz)
            for dx in (-1, 0, 1)
            for dy in (-1, 0, 1)
            for dz in (-1, 0, 1)
            if (dx, dy, dz) != (0, 0, 0)
        ]
        while remaining:
            seed = remaining.pop()
            component = [seed]
            queue = deque([seed])
            while queue:
                x, y, z = queue.popleft()
                for dx, dy, dz in neighbors:
                    candidate = (x + dx, y + dy, z + dz)
                    if candidate in remaining:
                        remaining.remove(candidate)
                        component.append(candidate)
                        queue.append(candidate)
            components.append(component)

        components.sort(key=lambda comp: sum(self.voxel_points[key] for key in comp), reverse=True)
        print('Largest persistent components (coordinates are in the Livox frame):')
        for number, component in enumerate(components[:12], 1):
            mins = [min(key[axis] for key in component) * self.voxel for axis in range(3)]
            maxs = [(max(key[axis] for key in component) + 1) * self.voxel for axis in range(3)]
            corners = [
                math.sqrt(x * x + y * y + z * z)
                for x in (mins[0], maxs[0])
                for y in (mins[1], maxs[1])
                for z in (mins[2], maxs[2])
            ]
            points = sum(self.voxel_points[key] for key in component)
            max_seen = max(self.voxel_frames[key] for key in component)
            print(
                f'  #{number:02d}: voxels={len(component):4d} points={points:7d} '
                f'frames_max={max_seen:3d}/{self.frames} '
                f'r~[{min(corners):.3f},{max(corners):.3f}]m '
                f'x=[{mins[0]:+.3f},{maxs[0]:+.3f}] '
                f'y=[{mins[1]:+.3f},{maxs[1]:+.3f}] '
                f'z=[{mins[2]:+.3f},{maxs[2]:+.3f}]'
            )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--frames', type=int, default=80)
    parser.add_argument('--radius', type=float, default=2.0)
    parser.add_argument('--voxel', type=float, default=0.05)
    parser.add_argument('--persistence', type=float, default=0.60)
    args = parser.parse_args()

    rclpy.init()
    node = NearFieldAnalyzer(args.frames, args.radius, args.voxel, args.persistence)
    try:
        while rclpy.ok() and node.frames < node.target_frames:
            rclpy.spin_once(node, timeout_sec=1.0)
        node.print_report()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
