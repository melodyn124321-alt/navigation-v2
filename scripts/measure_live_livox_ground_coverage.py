#!/usr/bin/env python3
"""Measure current MID-360 ground-return coverage in vehicle coordinates."""

import argparse
import math
import time

import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def percentile(values, ratio):
    if not values:
        return float("nan")
    ordered = sorted(values)
    return ordered[round((len(ordered) - 1) * ratio)]


class GroundCoverage(Node):
    def __init__(self, args):
        super().__init__("measure_live_livox_ground_coverage")
        self.args = args
        self.rotation = rpy_matrix(
            math.radians(args.roll),
            math.radians(args.pitch),
            math.radians(args.yaw),
        )
        self.translation = (args.x, args.y, args.z)
        self.sectors = [{
            "ranges": [],
            "x": [],
            "y": [],
        } for _ in range(12)]
        self.frames = 0
        self.valid_points = 0
        self.ground_points = 0
        self.started = time.monotonic()
        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(CustomMsg, args.topic, self.on_scan, qos)

    def on_scan(self, message):
        if self.frames >= self.args.frames:
            return
        self.frames += 1
        for point in message.points[::self.args.point_stride]:
            raw = (float(point.x), float(point.y), float(point.z))
            vector = tuple(
                sum(
                    self.rotation[row][column] * raw[column]
                    for column in range(3)
                )
                for row in range(3)
            )
            if not all(math.isfinite(value) for value in vector):
                continue
            base = tuple(
                vector[axis] + self.translation[axis] for axis in range(3))
            self.valid_points += 1
            if abs(base[2]) > self.args.ground_half_band:
                continue
            radius = math.hypot(base[0], base[1])
            if radius < self.args.minimum_radius:
                continue
            azimuth = math.degrees(math.atan2(base[1], base[0]))
            sector_index = int(((azimuth + 180.0) % 360.0) // 30.0)
            sector = self.sectors[sector_index]
            sector["ranges"].append(radius)
            sector["x"].append(base[0])
            sector["y"].append(base[1])
            self.ground_points += 1

    def report(self):
        elapsed = time.monotonic() - self.started
        print(
            f"frames={self.frames}/{self.args.frames} elapsed={elapsed:.3f}s "
            f"rate={self.frames / elapsed:.3f}Hz "
            f"sampled_valid_points={self.valid_points} "
            f"ground_band_points={self.ground_points}")
        print(
            f"transform base<-lidar: xyz=({self.args.x:+.3f},"
            f"{self.args.y:+.3f},{self.args.z:+.3f})m "
            f"rpy=({self.args.roll:+.3f},{self.args.pitch:+.3f},"
            f"{self.args.yaw:+.3f})deg "
            f"ground_z=[{-self.args.ground_half_band:+.3f},"
            f"{self.args.ground_half_band:+.3f}]m")
        covered = 0
        all_x = []
        all_y = []
        for index, sector in enumerate(self.sectors):
            lo = -180 + 30 * index
            hi = lo + 30
            ranges = sector["ranges"]
            all_x.extend(sector["x"])
            all_y.extend(sector["y"])
            if len(ranges) >= self.args.minimum_sector_points:
                covered += 1
                status = "GROUND"
            else:
                status = "NO_GROUND"
            print(
                f"azimuth[{lo:+04d},{hi:+04d})deg {status:9s} "
                f"points={len(ranges):6d} "
                f"radius_min={min(ranges, default=float('nan')):5.2f}m "
                f"p05={percentile(ranges, .05):5.2f}m "
                f"p50={percentile(ranges, .50):5.2f}m "
                f"p95={percentile(ranges, .95):5.2f}m "
                f"max={max(ranges, default=float('nan')):5.2f}m")
        print(
            f"ground_azimuth_coverage={covered}/12 sectors "
            f"({covered * 30}deg nominal)")
        if all_x:
            print(
                f"measured_ground_bbox_in_base: "
                f"x=[{min(all_x):+.2f},{max(all_x):+.2f}]m "
                f"y=[{min(all_y):+.2f},{max(all_y):+.2f}]m")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--frames", type=int, default=120)
    parser.add_argument("--point-stride", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=20.0)
    parser.add_argument("--x", type=float, default=0.89)
    parser.add_argument("--y", type=float, default=-0.05)
    parser.add_argument("--z", type=float, default=0.70)
    parser.add_argument("--roll", type=float, default=15.732)
    parser.add_argument("--pitch", type=float, default=-0.611)
    # M12 (-X) faces vehicle right (-Y), so LiDAR +X faces vehicle left.
    parser.add_argument("--yaw", type=float, default=90.0)
    parser.add_argument("--ground-half-band", type=float, default=0.12)
    parser.add_argument("--minimum-radius", type=float, default=0.30)
    parser.add_argument("--minimum-sector-points", type=int, default=20)
    args = parser.parse_args()

    rclpy.init()
    node = GroundCoverage(args)
    deadline = time.monotonic() + args.timeout
    try:
        while (
            rclpy.ok()
            and node.frames < args.frames
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.5)
        node.report()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if node.frames < args.frames:
        raise SystemExit(2)


if __name__ == "__main__":
    main()
