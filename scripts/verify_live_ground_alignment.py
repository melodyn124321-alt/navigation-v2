#!/usr/bin/env python3
"""Fit the live floor after applying the configured LiDAR-to-base transform."""

import argparse
import math
import struct
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


class Collector(Node):
    def __init__(self, args):
        super().__init__("verify_live_ground_alignment")
        self.args = args
        self.clouds = []
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, args.topic, self.on_cloud, qos)

    @staticmethod
    def field_offset(message, name):
        for field in message.fields:
            if field.name == name and field.datatype == 7:
                return field.offset
        raise RuntimeError(f"missing FLOAT32 field: {name}")

    def on_cloud(self, message):
        if len(self.clouds) >= self.args.frames:
            return
        count = int(message.width) * int(message.height)
        if count <= 0:
            return
        point_step = int(message.point_step)
        endian = ">" if message.is_bigendian else "<"
        offsets = [self.field_offset(message, axis) for axis in "xyz"]
        points = []
        for index in range(0, count, self.args.point_stride):
            row, column = divmod(index, int(message.width))
            start = row * int(message.row_step) + column * point_step
            try:
                point = [
                    struct.unpack_from(endian + "f", message.data, start + offset)[0]
                    for offset in offsets
                ]
            except (IndexError, struct.error):
                continue
            if all(math.isfinite(value) for value in point):
                points.append(point)
        if points:
            self.clouds.append(np.asarray(points, dtype=np.float64))


def fit_floor(points, threshold, iterations):
    rng = np.random.default_rng(20260729)
    best_mask = None
    best_count = 0
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = np.linalg.norm(normal)
        if length < 1.0e-8:
            continue
        normal /= length
        if abs(normal[2]) < 0.90:
            continue
        distance = np.abs((points - sample[0]) @ normal)
        mask = distance <= threshold
        count = int(mask.sum())
        if count > best_count:
            best_count = count
            best_mask = mask
    if best_mask is None or best_count < 100:
        raise RuntimeError("no dominant near-horizontal ground plane found")
    inliers = points[best_mask]
    center = inliers.mean(axis=0)
    _, _, vh = np.linalg.svd(inliers - center, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0.0:
        normal = -normal
    residuals = np.abs((inliers - center) @ normal)
    plane_z_at_origin = float(normal @ center) / float(normal[2])
    tilt = math.degrees(math.acos(min(1.0, max(-1.0, float(normal[2])))))
    return normal, plane_z_at_origin, tilt, residuals, inliers


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/cloud_registered_body")
    parser.add_argument("--frames", type=int, default=30)
    parser.add_argument("--point-stride", type=int, default=4)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--x", type=float, default=0.10)
    parser.add_argument("--y", type=float, default=-0.11)
    parser.add_argument("--z", type=float, default=0.749)
    parser.add_argument("--roll-deg", type=float, default=-45.937)
    parser.add_argument("--pitch-deg", type=float, default=-0.4065)
    parser.add_argument("--yaw-deg", type=float, default=-90.0)
    parser.add_argument("--search-min-z", type=float, default=-0.50)
    parser.add_argument("--search-max-z", type=float, default=0.30)
    parser.add_argument("--min-radius", type=float, default=0.50)
    parser.add_argument("--max-radius", type=float, default=12.0)
    parser.add_argument("--plane-threshold", type=float, default=0.035)
    parser.add_argument("--iterations", type=int, default=240)
    parser.add_argument("--obstacle-min-z", type=float, default=0.08)
    parser.add_argument("--max-ground-tilt-deg", type=float, default=1.0)
    parser.add_argument("--max-ground-offset-m", type=float, default=0.05)
    args = parser.parse_args()

    rclpy.init()
    node = Collector(args)
    deadline = time.monotonic() + args.timeout
    try:
        while (
            rclpy.ok()
            and len(node.clouds) < args.frames
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if len(node.clouds) < max(5, args.frames // 2):
        raise SystemExit(
            f"RESULT: FAIL received {len(node.clouds)}/{args.frames} clouds")

    sensor_points = np.concatenate(node.clouds, axis=0)
    rotation = rpy_matrix(
        math.radians(args.roll_deg),
        math.radians(args.pitch_deg),
        math.radians(args.yaw_deg),
    )
    base_points = sensor_points @ rotation.T
    base_points += np.array([args.x, args.y, args.z])
    radius = np.hypot(base_points[:, 0], base_points[:, 1])
    candidates = base_points[
        (base_points[:, 2] >= args.search_min_z)
        & (base_points[:, 2] <= args.search_max_z)
        & (radius >= args.min_radius)
        & (radius <= args.max_radius)
    ]
    if len(candidates) > 60000:
        rng = np.random.default_rng(20260729)
        candidates = candidates[rng.choice(len(candidates), 60000, replace=False)]
    if len(candidates) < 500:
        raise SystemExit(
            f"RESULT: FAIL only {len(candidates)} floor candidates")

    normal, ground_z, tilt, residuals, floor_inliers = fit_floor(
        candidates, args.plane_threshold, args.iterations)
    recommended_z = args.z - ground_z
    ground_above_obstacle_min = float(np.mean(
        floor_inliers[:, 2] >= args.obstacle_min_z))
    print(
        f"clouds={len(node.clouds)} sampled_points={len(sensor_points)} "
        f"floor_candidates={len(candidates)}")
    print(
        f"base_from_lidar xyz=({args.x:+.3f},{args.y:+.3f},{args.z:+.3f})m "
        f"rpy=({args.roll_deg:+.3f},{args.pitch_deg:+.4f},"
        f"{args.yaw_deg:+.3f})deg")
    print(
        f"floor_normal=({normal[0]:+.6f},{normal[1]:+.6f},"
        f"{normal[2]:+.6f}) tilt={tilt:.3f}deg "
        f"floor_z_at_base_origin={ground_z:+.4f}m "
        f"residual_p50={np.median(residuals):.4f}m "
        f"residual_p99={np.percentile(residuals, 99):.4f}m")
    print(
        f"recommended_lidar_z_for_ground_zero={recommended_z:.4f}m "
        f"floor_inliers_above_obstacle_min={ground_above_obstacle_min:.1%}")
    if (
        tilt > args.max_ground_tilt_deg
        or abs(ground_z) > args.max_ground_offset_m
    ):
        print("RESULT: FAIL live floor is not aligned with base_link z=0")
        raise SystemExit(2)
    print("RESULT: PASS live floor is level and centred at base_link z=0")


if __name__ == "__main__":
    main()
