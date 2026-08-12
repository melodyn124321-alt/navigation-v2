#!/usr/bin/env python3
"""Find coarse chassis poses by matching a live body cloud to a level PCD."""

import argparse
import math
import struct
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2


PCD_FORMATS = {
    ("F", 4): "<f4",
    ("F", 8): "<f8",
    ("I", 1): "i1",
    ("I", 2): "<i2",
    ("I", 4): "<i4",
    ("U", 1): "u1",
    ("U", 2): "<u2",
    ("U", 4): "<u4",
}


def read_pcd_xyz(path):
    metadata = {}
    with Path(path).open("rb") as stream:
        while True:
            line = stream.readline()
            if not line:
                raise ValueError("PCD header ended before DATA")
            decoded = line.decode("ascii").strip()
            if not decoded or decoded.startswith("#"):
                continue
            key, *values = decoded.split()
            metadata[key.upper()] = values
            if key.upper() == "DATA":
                if values[0].lower() != "binary":
                    raise ValueError("only binary PCD is supported")
                break
        names = metadata.get("FIELDS") or metadata["FIELD"]
        sizes = [int(value) for value in metadata["SIZE"]]
        types = [value.upper() for value in metadata["TYPE"]]
        counts = [int(value) for value in metadata.get(
            "COUNT", ["1"] * len(names))]
        point_count = int((metadata.get("POINTS") or metadata["WIDTH"])[0])
        offsets = []
        formats = []
        offset = 0
        for name, size, kind, count in zip(names, sizes, types, counts):
            fmt = PCD_FORMATS.get((kind, size))
            if fmt is None:
                raise ValueError(f"unsupported PCD type {kind}{size}")
            offsets.append(offset)
            formats.append((fmt, count) if count > 1 else fmt)
            offset += size * count
        dtype = np.dtype({
            "names": names,
            "formats": formats,
            "offsets": offsets,
            "itemsize": offset,
        })
        records = np.frombuffer(stream.read(point_count * offset), dtype=dtype)
    return np.column_stack([
        records[axis].astype(np.float64, copy=False) for axis in "xyz"
    ])


def rpy_matrix(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ], dtype=np.float64)


def pointcloud_xyz(message):
    offsets = {
        field.name: field.offset
        for field in message.fields
        if field.name in ("x", "y", "z") and field.datatype == 7
    }
    if len(offsets) != 3:
        raise ValueError("PointCloud2 has no FLOAT32 x/y/z fields")
    endian = ">" if message.is_bigendian else "<"
    count = int(message.width) * int(message.height)
    dtype = np.dtype({
        "names": ["x", "y", "z"],
        "formats": [endian + "f4"] * 3,
        "offsets": [offsets[axis] for axis in "xyz"],
        "itemsize": int(message.point_step),
    })
    records = np.frombuffer(message.data, dtype=dtype, count=count)
    points = np.column_stack([records[axis] for axis in "xyz"])
    return points[np.isfinite(points).all(axis=1)].astype(np.float64)


class CloudCollector(Node):
    def __init__(self, topic, frames):
        super().__init__("auto_initial_pose_2d")
        self.frames = frames
        self.clouds = []
        qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.RELIABLE)
        self.create_subscription(PointCloud2, topic, self.on_cloud, qos)

    def on_cloud(self, message):
        if len(self.clouds) < self.frames:
            self.clouds.append(pointcloud_xyz(message))


def make_structural_map(points, resolution, min_z, max_z, min_points, min_span):
    selected = points[
        np.isfinite(points).all(axis=1)
        & (points[:, 2] >= min_z)
        & (points[:, 2] <= max_z)
    ]
    origin = np.floor(selected[:, :2].min(axis=0) / resolution) * resolution
    maximum = np.ceil(selected[:, :2].max(axis=0) / resolution) * resolution
    shape_xy = np.ceil((maximum - origin) / resolution).astype(int) + 1
    cells = np.floor((selected[:, :2] - origin) / resolution).astype(int)
    flat = cells[:, 1] * shape_xy[0] + cells[:, 0]
    size = int(shape_xy[0] * shape_xy[1])
    counts = np.bincount(flat, minlength=size)
    low = np.full(size, np.inf)
    high = np.full(size, -np.inf)
    np.minimum.at(low, flat, selected[:, 2])
    np.maximum.at(high, flat, selected[:, 2])
    occupied = (
        (counts >= min_points) & ((high - low) >= min_span)
    ).reshape((shape_xy[1], shape_xy[0]))
    return occupied, origin


def unique_voxels(points, resolution):
    cells = np.round(points / resolution).astype(np.int32)
    return np.unique(cells, axis=0).astype(np.float64) * resolution


def match_angle(distance_map, map_origin, live_xy, yaw, resolution):
    cosine, sine = math.cos(yaw), math.sin(yaw)
    rotation = np.array([[cosine, -sine], [sine, cosine]])
    rotated = live_xy @ rotation.T
    lower = np.floor(rotated.min(axis=0) / resolution) * resolution
    upper = np.ceil(rotated.max(axis=0) / resolution) * resolution
    size = np.maximum(1, np.round((upper - lower) / resolution).astype(int) + 1)
    if size[0] >= distance_map.shape[1] or size[1] >= distance_map.shape[0]:
        return []
    cells = np.round((rotated - lower) / resolution).astype(int)
    template = np.zeros((size[1], size[0]), dtype=np.float32)
    template[cells[:, 1], cells[:, 0]] = 1.0
    count = float(template.sum())
    if count < 20:
        return []
    scores = cv2.matchTemplate(
        distance_map.astype(np.float32), template, cv2.TM_CCORR) / count
    candidates = []
    working = scores.copy()
    for _ in range(3):
        value, _, location, _ = cv2.minMaxLoc(working)
        tx = map_origin[0] + location[0] * resolution - lower[0]
        ty = map_origin[1] + location[1] * resolution - lower[1]
        candidates.append((float(value), float(tx), float(ty), yaw))
        x0, y0 = location
        working[
            max(0, y0 - 5):min(working.shape[0], y0 + 6),
            max(0, x0 - 5):min(working.shape[1], x0 + 6),
        ] = np.inf
    return candidates


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("pcd_path")
    parser.add_argument("--topic", default="/cloud_registered_body")
    parser.add_argument("--frames", type=int, default=20)
    parser.add_argument("--timeout", type=float, default=15.0)
    parser.add_argument("--resolution", type=float, default=0.10)
    parser.add_argument("--angle-step-deg", type=float, default=5.0)
    parser.add_argument("--top", type=int, default=20)
    args = parser.parse_args()

    rclpy.init()
    collector = CloudCollector(args.topic, args.frames)
    deadline = time.monotonic() + args.timeout
    try:
        while (
                rclpy.ok() and len(collector.clouds) < args.frames
                and time.monotonic() < deadline):
            rclpy.spin_once(collector, timeout_sec=0.2)
    finally:
        collector.destroy_node()
        rclpy.shutdown()
    if len(collector.clouds) < max(5, args.frames // 2):
        raise SystemExit(
            f"FAIL live clouds={len(collector.clouds)}/{args.frames}")

    map_points = read_pcd_xyz(args.pcd_path)
    occupied, map_origin = make_structural_map(
        map_points, args.resolution, -0.635880326, 0.844119674, 3, 0.10)
    distance = cv2.distanceTransform(
        (~occupied).astype(np.uint8), cv2.DIST_L2, 5) * args.resolution
    distance = np.minimum(distance, 1.5).astype(np.float32)

    sensor_points = np.concatenate(collector.clouds, axis=0)
    rotation = rpy_matrix(
        math.radians(-45.937), math.radians(-0.4065), math.radians(-90.0))
    base_points = sensor_points @ rotation.T
    base_points += np.array([0.100, -0.110, 0.749])
    radius = np.hypot(base_points[:, 0], base_points[:, 1])
    live = base_points[
        (base_points[:, 2] >= 0.12)
        & (base_points[:, 2] <= 1.60)
        & (radius >= 0.45)
        & (radius <= 10.0)
    ][:, :2]
    live = unique_voxels(live, args.resolution)
    if len(live) < 100:
        raise SystemExit(f"FAIL live structural voxels={len(live)}")

    coarse = []
    angle = -180.0
    while angle < 180.0 - 1.0e-6:
        coarse.extend(match_angle(
            distance, map_origin, live, math.radians(angle), args.resolution))
        angle += args.angle_step_deg
    coarse.sort(key=lambda item: item[0])

    refine_angles = set()
    for _, _, _, yaw in coarse[:12]:
        center = math.degrees(yaw)
        for delta in np.arange(-args.angle_step_deg, args.angle_step_deg + 0.1, 1.0):
            refine_angles.add(round(center + float(delta), 3))
    refined = []
    for degrees in sorted(refine_angles):
        yaw = math.radians(((degrees + 180.0) % 360.0) - 180.0)
        refined.extend(match_angle(
            distance, map_origin, live, yaw, args.resolution))
    refined.sort(key=lambda item: item[0])

    selected = []
    for candidate in refined:
        _, x, y, yaw = candidate
        if any(
                math.hypot(x - old_x, y - old_y) < 0.50
                and abs(math.atan2(
                    math.sin(yaw - old_yaw), math.cos(yaw - old_yaw)))
                < math.radians(8.0)
                for _, old_x, old_y, old_yaw in selected):
            continue
        selected.append(candidate)
        if len(selected) >= args.top:
            break

    print(
        f"AUTO_POSE_2D map_voxels={int(occupied.sum())} "
        f"live_voxels={len(live)} clouds={len(collector.clouds)}")
    for index, (score, x, y, yaw) in enumerate(selected, 1):
        print(
            f"CANDIDATE rank={index} mean_distance={score:.4f}m "
            f"x={x:.3f} y={y:.3f} yaw_deg={math.degrees(yaw):+.1f}")


if __name__ == "__main__":
    main()
