#!/usr/bin/env python3
"""Find persistent near-field returns fixed in a Livox sensor frame."""

import argparse
import collections
import glob
import math
import os
import sqlite3

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def percentile(values, ratio):
    ordered = sorted(values)
    if not ordered:
        return float("nan")
    index = round((len(ordered) - 1) * ratio)
    return ordered[max(0, min(index, len(ordered) - 1))]


def connected_components(voxels):
    remaining = set(voxels)
    components = []
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
        queue = collections.deque([seed])
        while queue:
            current = queue.popleft()
            for delta in neighbors:
                neighbor = tuple(a + b for a, b in zip(current, delta))
                if neighbor in remaining:
                    remaining.remove(neighbor)
                    component.append(neighbor)
                    queue.append(neighbor)
        components.append(component)
    return sorted(components, key=len, reverse=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--sample-frames", type=int, default=300)
    parser.add_argument("--min-range", type=float, default=0.08)
    parser.add_argument("--max-range", type=float, default=1.50)
    parser.add_argument("--mapping-blind", type=float, default=0.10)
    parser.add_argument("--voxel", type=float, default=0.04)
    parser.add_argument("--min-persistence", type=float, default=0.08)
    parser.add_argument("--min-component-voxels", type=int, default=3)
    parser.add_argument("--base-x", type=float, default=0.89)
    parser.add_argument("--base-y", type=float, default=-0.05)
    parser.add_argument("--base-z", type=float, default=0.70)
    parser.add_argument("--base-yaw", type=float, default=math.pi)
    parser.add_argument("--check-exclusion", action="store_true")
    parser.add_argument("--check-range-gate", action="store_true")
    parser.add_argument("--exclusion-min-x", type=float, default=-0.68)
    parser.add_argument("--exclusion-max-x", type=float, default=0.03)
    parser.add_argument("--exclusion-min-y", type=float, default=-0.34)
    parser.add_argument("--exclusion-max-y", type=float, default=0.34)
    parser.add_argument("--exclusion-min-z", type=float, default=-0.08)
    parser.add_argument("--exclusion-max-z", type=float, default=0.38)
    args = parser.parse_args()

    databases = sorted(glob.glob(os.path.join(args.bag_path, "*.db3")))
    if len(databases) != 1:
        raise SystemExit(
            f"expected exactly one .db3 file, found {len(databases)}")
    connection = sqlite3.connect(f"file:{databases[0]}?mode=ro", uri=True)
    topic = connection.execute(
        "SELECT id, type FROM topics WHERE name=?", (args.topic,)
    ).fetchone()
    if topic is None:
        raise SystemExit(f"topic not found: {args.topic}")
    topic_id, type_name = topic
    message_type = get_message(type_name)
    rows = connection.execute(
        "SELECT id FROM messages WHERE topic_id=? ORDER BY timestamp",
        (topic_id,),
    ).fetchall()
    if not rows:
        raise SystemExit(f"topic has no messages: {args.topic}")

    requested = max(1, args.sample_frames)
    stride = max(1, len(rows) // requested)
    sampled = rows[::stride][:requested]
    frame_hits = collections.Counter()
    point_ranges = []
    near_points = 0
    for (message_id,) in sampled:
        data = connection.execute(
            "SELECT data FROM messages WHERE id=?", (message_id,)
        ).fetchone()[0]
        message = deserialize_message(data, message_type)
        occupied = set()
        for point in message.points:
            x, y, z = float(point.x), float(point.y), float(point.z)
            distance = math.sqrt(x * x + y * y + z * z)
            if (
                not math.isfinite(distance)
                or distance < args.min_range
                or distance > args.max_range
            ):
                continue
            near_points += 1
            point_ranges.append(distance)
            occupied.add((
                math.floor(x / args.voxel),
                math.floor(y / args.voxel),
                math.floor(z / args.voxel),
            ))
        frame_hits.update(occupied)
    connection.close()

    minimum_hits = max(2, math.ceil(len(sampled) * args.min_persistence))
    persistent = {
        voxel: hits for voxel, hits in frame_hits.items()
        if hits >= minimum_hits
    }
    components = [
        component for component in connected_components(persistent)
        if len(component) >= args.min_component_voxels
    ]

    print(
        f"bag={args.bag_path} frames_total={len(rows)} "
        f"frames_sampled={len(sampled)} voxel={args.voxel:.3f}m "
        f"range=[{args.min_range:.2f},{args.max_range:.2f}]m"
    )
    print(
        f"near_points={near_points} nearest={min(point_ranges, default=float('nan')):.3f}m "
        f"range_p01={percentile(point_ranges, 0.01):.3f}m "
        f"persistent_threshold={minimum_hits}/{len(sampled)}frames "
        f"persistent_voxels={len(persistent)} components={len(components)}"
    )

    cosine = math.cos(args.base_yaw)
    sine = math.sin(args.base_yaw)
    for index, component in enumerate(components[:20], start=1):
        centers = [
            tuple((coordinate + 0.5) * args.voxel for coordinate in voxel)
            for voxel in component
        ]
        sensor_min = [min(point[axis] for point in centers) for axis in range(3)]
        sensor_max = [max(point[axis] for point in centers) for axis in range(3)]
        ranges = [math.sqrt(sum(value * value for value in point)) for point in centers]
        nearest = min(ranges)
        farthest = max(ranges)
        base_points = [(
            args.base_x + cosine * x - sine * y,
            args.base_y + sine * x + cosine * y,
            args.base_z + z,
        ) for x, y, z in centers]
        base_min = [min(point[axis] for point in base_points) for axis in range(3)]
        base_max = [max(point[axis] for point in base_points) for axis in range(3)]
        hit_values = [persistent[voxel] for voxel in component]
        beyond_blind = sum(
            math.sqrt(sum(value * value for value in point))
            > args.mapping_blind
            for point in centers
        )
        print(
            f"component_{index}: voxels={len(component)} "
            f"persistence_max={max(hit_values)}/{len(sampled)} "
            f"nearest_sensor={nearest:.3f}m "
            f"farthest_sensor={farthest:.3f}m "
            f"voxels_beyond_blind_{args.mapping_blind:.2f}m="
            f"{beyond_blind}/{len(component)}"
        )
        print(
            "  sensor_bbox: "
            f"x=[{sensor_min[0]:+.3f},{sensor_max[0]:+.3f}] "
            f"y=[{sensor_min[1]:+.3f},{sensor_max[1]:+.3f}] "
            f"z=[{sensor_min[2]:+.3f},{sensor_max[2]:+.3f}]"
        )
        print(
            "  base_bbox:   "
            f"x=[{base_min[0]:+.3f},{base_max[0]:+.3f}] "
            f"y=[{base_min[1]:+.3f},{base_max[1]:+.3f}] "
            f"z=[{base_min[2]:+.3f},{base_max[2]:+.3f}]"
        )

    if not components:
        print(
            "RESULT: no persistent near-field component found; lower "
            "--min-persistence or increase --voxel only after checking the bag"
        )
        return 2
    if args.check_range_gate:
        primary = max(
            components,
            key=lambda component: (
                max(persistent[voxel] for voxel in component),
                len(component),
            ),
        )
        centers = [
            tuple((coordinate + 0.5) * args.voxel for coordinate in voxel)
            for voxel in primary
        ]
        outside = [
            point for point in centers
            if math.sqrt(sum(value * value for value in point))
            > args.mapping_blind
        ]
        if outside:
            farthest = max(
                math.sqrt(sum(value * value for value in point))
                for point in centers
            )
            print(
                f"RESULT: FAIL primary persistent component has "
                f"{len(outside)}/{len(primary)} voxels beyond the "
                f"{args.mapping_blind:.3f}m range gate; "
                f"farthest={farthest:.3f}m"
            )
            return 4
        print(
            "range_gate_check: PASS primary persistent component is fully "
            f"inside blind={args.mapping_blind:.3f}m"
        )
    if args.check_exclusion:
        primary = max(
            components,
            key=lambda component: (
                max(persistent[voxel] for voxel in component),
                len(component),
            ),
        )
        centers = [
            tuple((coordinate + 0.5) * args.voxel for coordinate in voxel)
            for voxel in primary
        ]
        limits = (
            (args.exclusion_min_x, args.exclusion_max_x),
            (args.exclusion_min_y, args.exclusion_max_y),
            (args.exclusion_min_z, args.exclusion_max_z),
        )
        outside = [
            point for point in centers
            if any(
                value < minimum or value > maximum
                for value, (minimum, maximum) in zip(point, limits)
            )
        ]
        if outside:
            print(
                f"RESULT: FAIL primary persistent component has "
                f"{len(outside)}/{len(primary)} voxels outside the exclusion box"
            )
            return 3
        print(
            "exclusion_check: PASS primary persistent component is fully "
            "inside x=[{:.3f},{:.3f}] y=[{:.3f},{:.3f}] "
            "z=[{:.3f},{:.3f}]".format(
                args.exclusion_min_x, args.exclusion_max_x,
                args.exclusion_min_y, args.exclusion_max_y,
                args.exclusion_min_z, args.exclusion_max_z,
            )
        )
    print("RESULT: persistent sensor-frame returns found")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
