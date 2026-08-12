#!/usr/bin/env python3
"""Analyze Livox bag continuity, raw angular coverage, and static IMU tilt."""

import argparse
import glob
import math
import os
import sqlite3
import statistics
import struct

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def percentile(values, ratio):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = round((len(ordered) - 1) * ratio)
    return ordered[max(0, min(index, len(ordered) - 1))]


def describe(name, values, unit=""):
    print(
        f"{name}: min={min(values):.3f}{unit} "
        f"p01={percentile(values, 0.01):.3f}{unit} "
        f"p05={percentile(values, 0.05):.3f}{unit} "
        f"p50={percentile(values, 0.50):.3f}{unit} "
        f"p95={percentile(values, 0.95):.3f}{unit} "
        f"p99={percentile(values, 0.99):.3f}{unit} "
        f"max={max(values):.3f}{unit}"
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    parser.add_argument("--cloud-step", type=int, default=20)
    parser.add_argument("--points-per-cloud", type=int, default=2500)
    parser.add_argument("--static-window", type=float, default=12.0)
    parser.add_argument("--search-static-window", type=float, default=6.0)
    parser.add_argument("--max-gyro-std", type=float, default=0.03)
    parser.add_argument("--max-accel-std", type=float, default=0.05)
    parser.add_argument("--max-lidar-gap", type=float, default=0.15)
    parser.add_argument("--max-imu-gap", type=float, default=0.05)
    parser.add_argument("--max-abs-roll", type=float, default=3.0)
    parser.add_argument("--max-abs-pitch", type=float, default=3.0)
    parser.add_argument("--expected-roll", type=float)
    parser.add_argument("--expected-pitch", type=float)
    parser.add_argument("--max-angle-error", type=float, default=0.75)
    args = parser.parse_args()

    databases = sorted(glob.glob(os.path.join(args.bag_path, "*.db3")))
    if len(databases) != 1:
        raise SystemExit(
            f"expected exactly one sqlite3 .db3 file, found {len(databases)}")
    connection = sqlite3.connect(f"file:{databases[0]}?mode=ro", uri=True)
    topic_rows = connection.execute(
        "SELECT id, name, type FROM topics").fetchall()
    topic_ids = {name: topic_id for topic_id, name, _ in topic_rows}
    topic_types = {name: type_name for _, name, type_name in topic_rows}
    required = ("/livox/lidar", "/livox/imu")
    missing = [topic for topic in required if topic not in topic_types]
    if missing:
        raise SystemExit(f"missing required topics: {', '.join(missing)}")
    message_types = {
        topic: get_message(type_name) for topic, type_name in topic_types.items()
    }

    limits = {
        "/livox/lidar": args.max_lidar_gap,
        "/livox/imu": args.max_imu_gap,
    }
    nominal_rates = {"/livox/lidar": 10.0, "/livox/imu": 200.0}
    lidar_point_counts = []
    elevations = []
    azimuth_bins = set()
    ranges = []
    accel = []
    gyro = []
    all_imu = []
    rows_by_topic = {}
    counts = {}
    first_stamp = {}
    last_stamp = {}
    max_gap = {}
    gap_over_limit = {}
    for topic in required:
        rows = connection.execute(
            "SELECT id, timestamp, substr(data,1,64) FROM messages "
            "WHERE topic_id=? "
            "ORDER BY timestamp", (topic_ids[topic],)).fetchall()
        if not rows:
            raise SystemExit(f"topic has no messages: {topic}")
        rows_by_topic[topic] = rows
        counts[topic] = len(rows)
        first_stamp[topic] = rows[0][1] / 1e9
        last_stamp[topic] = rows[-1][1] / 1e9
        record_gaps = [
            (current[1] - previous[1]) / 1e9
            for previous, current in zip(rows, rows[1:])
        ]
        sensor_stamps = []
        for _, _, prefix in rows:
            endian = "<" if prefix[1] == 1 else ">"
            sec, nanosec = struct.unpack_from(endian + "iI", prefix, 4)
            sensor_stamps.append(sec + nanosec / 1e9)
        sensor_gaps = [
            current - previous
            for previous, current in zip(sensor_stamps, sensor_stamps[1:])
        ]
        max_gap[topic] = max(sensor_gaps, default=0.0)
        gap_over_limit[topic] = sum(
            gap > limits[topic] for gap in sensor_gaps)
        median_gap = statistics.median(sensor_gaps)
        sensor_span = sensor_stamps[-1] - sensor_stamps[0]
        estimated_missing = max(
            0,
            round(sensor_span * nominal_rates[topic]) + 1 - len(rows),
        )
        delays = [
            row[1] / 1e9 - sensor_stamp
            for row, sensor_stamp in zip(rows, sensor_stamps)
        ]
        print(
            f"{topic}_timing: sensor_median_gap={median_gap:.6f}s "
            f"sensor_max_gap={max_gap[topic]:.6f}s "
            f"estimated_missing_frames={estimated_missing} "
            f"record_max_gap={max(record_gaps, default=0.0):.6f}s "
            f"delivery_delay_p95={percentile(delays, 0.95):.6f}s "
            f"delivery_delay_max={max(delays, default=0.0):.6f}s"
        )

    initial_limit_ns = (
        rows_by_topic["/livox/imu"][0][1]
        + int(args.static_window * 1e9)
    )
    imu_rows = connection.execute(
        "SELECT timestamp, data FROM messages "
        "WHERE topic_id=? AND timestamp<=? ORDER BY timestamp",
        (topic_ids["/livox/imu"], initial_limit_ns),
    )
    for timestamp, data in imu_rows:
        message = deserialize_message(data, message_types["/livox/imu"])
        accel_sample = (
            float(message.linear_acceleration.x),
            float(message.linear_acceleration.y),
            float(message.linear_acceleration.z),
        )
        gyro_sample = (
            float(message.angular_velocity.x),
            float(message.angular_velocity.y),
            float(message.angular_velocity.z),
        )
        accel.append(accel_sample)
        gyro.append(gyro_sample)

    sampled_lidar_rows = rows_by_topic["/livox/lidar"][
        ::max(1, args.cloud_step)]
    for message_id, _, _ in sampled_lidar_rows:
        row = connection.execute(
            "SELECT data FROM messages WHERE id=?", (message_id,)).fetchone()
        if row is None:
            continue
        message = deserialize_message(
            row[0], message_types["/livox/lidar"])
        lidar_point_counts.append(len(message.points))
        stride = max(1, len(message.points) // max(1, args.points_per_cloud))
        for point in message.points[::stride]:
            x, y, z = float(point.x), float(point.y), float(point.z)
            distance = math.sqrt(x * x + y * y + z * z)
            if not math.isfinite(distance) or distance < 0.20 or distance > 100.0:
                continue
            horizontal = math.hypot(x, y)
            elevation = math.degrees(math.atan2(z, horizontal))
            azimuth = math.degrees(math.atan2(y, x))
            elevations.append(elevation)
            ranges.append(distance)
            azimuth_bins.add(int(math.floor((azimuth + 180.0) / 5.0)) % 72)

    static_label = "initial_static"
    if accel and gyro:
        initial_accel_std = [statistics.pstdev(axis) for axis in zip(*accel)]
        initial_gyro_std = [statistics.pstdev(axis) for axis in zip(*gyro)]
        initial_is_static = (
            max(initial_accel_std) <= args.max_accel_std
            and max(initial_gyro_std) <= args.max_gyro_std
        )
        if not initial_is_static:
            print(
                "WARNING: initial IMU window is moving; searching the bag "
                f"for a {args.search_static_window:.1f}s stationary window."
            )
            imu_rows = connection.execute(
                "SELECT timestamp, data FROM messages WHERE topic_id=? "
                "ORDER BY timestamp",
                (topic_ids["/livox/imu"],),
            )
            for timestamp, data in imu_rows:
                message = deserialize_message(
                    data, message_types["/livox/imu"])
                all_imu.append((
                    timestamp,
                    (
                        float(message.linear_acceleration.x),
                        float(message.linear_acceleration.y),
                        float(message.linear_acceleration.z),
                    ),
                    (
                        float(message.angular_velocity.x),
                        float(message.angular_velocity.y),
                        float(message.angular_velocity.z),
                    ),
                ))
            nominal_imu_hz = nominal_rates["/livox/imu"]
            window_samples = max(
                500, round(args.search_static_window * nominal_imu_hz))
            step_samples = max(1, round(nominal_imu_hz))
            best = None
            for start in range(
                0, max(0, len(all_imu) - window_samples + 1), step_samples
            ):
                samples = all_imu[start:start + window_samples]
                candidate_accel = [sample[1] for sample in samples]
                candidate_gyro = [sample[2] for sample in samples]
                accel_std = [
                    statistics.pstdev(axis)
                    for axis in zip(*candidate_accel)
                ]
                gyro_std = [
                    statistics.pstdev(axis)
                    for axis in zip(*candidate_gyro)
                ]
                score = (
                    max(accel_std) / args.max_accel_std
                    + max(gyro_std) / args.max_gyro_std
                )
                if best is None or score < best[0]:
                    best = (
                        score, start, candidate_accel, candidate_gyro,
                        accel_std, gyro_std,
                    )
            if best is not None:
                _, start, candidate_accel, candidate_gyro, accel_std, gyro_std = best
                if (
                    max(accel_std) <= args.max_accel_std
                    and max(gyro_std) <= args.max_gyro_std
                ):
                    accel = candidate_accel
                    gyro = candidate_gyro
                    static_label = "selected_static"
                    print(
                        "stationary_window_search: PASS "
                        f"start_offset={start / nominal_imu_hz:.1f}s "
                        f"samples={len(accel)} "
                        f"accel_std_max={max(accel_std):.5f} "
                        f"gyro_std_max={max(gyro_std):.5f}rad/s"
                    )
                else:
                    print(
                        "stationary_window_search: FAIL best window "
                        f"accel_std_max={max(accel_std):.5f}/"
                        f"{args.max_accel_std:.5f} "
                        f"gyro_std_max={max(gyro_std):.5f}/"
                        f"{args.max_gyro_std:.5f}rad/s"
                    )

    connection.close()

    print(f"bag={args.bag_path}")
    failed = False
    for topic in required:
        span = last_stamp[topic] - first_stamp[topic]
        hz = (counts[topic] - 1) / span if span > 0 else 0.0
        print(
            f"{topic}: count={counts[topic]} span={span:.3f}s "
            f"average_hz={hz:.3f} max_gap={max_gap[topic]:.6f}s "
            f"gaps_over_{limits[topic]:.3f}s={gap_over_limit[topic]}"
        )
        if gap_over_limit[topic]:
            failed = True

    if lidar_point_counts:
        describe("sampled_points_per_lidar_frame", lidar_point_counts)
    if ranges:
        describe("sampled_range", ranges, "m")
    if elevations:
        describe("raw_elevation", elevations, "deg")
        print(
            f"raw_azimuth_5deg_bins={len(azimuth_bins)}/72 "
            f"coverage={100.0 * len(azimuth_bins) / 72.0:.1f}% "
            f"sampled_points={len(elevations)}"
        )

    tilt_failed = False
    if accel:
        means = [statistics.fmean(axis) for axis in zip(*accel)]
        stddev = [statistics.pstdev(axis) for axis in zip(*accel)]
        gyro_means = [statistics.fmean(axis) for axis in zip(*gyro)]
        gyro_stddev = [statistics.pstdev(axis) for axis in zip(*gyro)]
        ax, ay, az = means
        roll = math.degrees(math.atan2(ay, az))
        pitch = math.degrees(math.atan2(-ax, math.hypot(ay, az)))
        accel_norm = math.sqrt(ax * ax + ay * ay + az * az)
        print(
            f"{static_label}_imu: "
            f"samples={len(accel)} "
            f"accel_mean=({ax:.4f},{ay:.4f},{az:.4f})msg_units "
            f"accel_std=({stddev[0]:.4f},{stddev[1]:.4f},{stddev[2]:.4f}) "
            f"norm={accel_norm:.4f}msg_units"
        )
        print(
            f"{static_label}_gyro: "
            f"mean=({gyro_means[0]:.5f},{gyro_means[1]:.5f},"
            f"{gyro_means[2]:.5f})rad/s "
            f"std=({gyro_stddev[0]:.5f},{gyro_stddev[1]:.5f},"
            f"{gyro_stddev[2]:.5f})"
        )
        print(
            f"gravity_tilt_estimate: roll={roll:.3f}deg "
            f"pitch={pitch:.3f}deg"
        )
        if (
            max(stddev) > args.max_accel_std
            or max(gyro_stddev) > args.max_gyro_std
        ):
            print(
                "WARNING: no sufficiently static IMU window was found; "
                "the tilt estimate is low confidence."
            )
            tilt_failed = True
        calibrated_mode = (
            args.expected_roll is not None
            and args.expected_pitch is not None
        )
        if calibrated_mode:
            roll_error = abs(roll - args.expected_roll)
            pitch_error = abs(pitch - args.expected_pitch)
            print(
                "mount_calibration: "
                f"expected_roll={args.expected_roll:+.3f}deg "
                f"expected_pitch={args.expected_pitch:+.3f}deg "
                f"error=({roll_error:.3f},{pitch_error:.3f})deg "
                f"limit={args.max_angle_error:.3f}deg"
            )
            if (
                roll_error > args.max_angle_error
                or pitch_error > args.max_angle_error
            ):
                print("ERROR: recorded mount pose differs from calibration")
                tilt_failed = True
        elif abs(roll) > args.max_abs_roll or abs(pitch) > args.max_abs_pitch:
            print(
                "ERROR: mount tilt exceeds limit: "
                f"|roll|={abs(roll):.3f}/{args.max_abs_roll:.3f}deg "
                f"|pitch|={abs(pitch):.3f}/{args.max_abs_pitch:.3f}deg"
            )
            tilt_failed = True

    reasons = []
    if failed:
        reasons.append("continuity")
    if tilt_failed:
        reasons.append("mount_tilt")
    if reasons:
        print("RESULT: FAIL " + ",".join(reasons))
        raise SystemExit(2)
    print("RESULT: PASS continuity and calibrated mount pose")


if __name__ == "__main__":
    main()
