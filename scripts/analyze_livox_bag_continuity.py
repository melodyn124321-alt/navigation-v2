#!/usr/bin/env python3
"""Fast, read-only continuity gate for Livox rosbag2 SQLite recordings."""

import argparse
import glob
import os
import sqlite3
import struct
import sys


def percentile(values, ratio):
    if not values:
        return float("nan")
    ordered = sorted(values)
    index = round((len(ordered) - 1) * ratio)
    return ordered[max(0, min(index, len(ordered) - 1))]


def header_stamp_from_cdr(prefix):
    if len(prefix) < 12:
        raise ValueError(f"CDR message prefix is too short: {len(prefix)} bytes")
    endian = "<" if prefix[1] == 1 else ">"
    sec, nanosec = struct.unpack_from(endian + "iI", prefix, 4)
    if nanosec >= 1_000_000_000:
        raise ValueError(f"invalid header nanoseconds: {nanosec}")
    return float(sec) + float(nanosec) * 1e-9


def continuity(label, stamps, storage_stamps, minimum_rate, maximum_gap):
    failures = []
    if len(stamps) < 2:
        return [f"{label}: fewer than two messages"]

    deltas = [current - previous for previous, current in zip(stamps, stamps[1:])]
    positive = [delta for delta in deltas if delta > 0.0]
    nonpositive = sum(delta <= 0.0 for delta in deltas)
    duration = stamps[-1] - stamps[0]
    rate = (len(stamps) - 1) / duration if duration > 0.0 else 0.0
    maximum = max(positive, default=float("inf"))
    gap_count = sum(delta > maximum_gap for delta in positive)
    storage_lag = [
        storage_stamp - header_stamp
        for storage_stamp, header_stamp in zip(storage_stamps, stamps)
    ]

    print(
        f"{label}_continuity: messages={len(stamps)} duration={duration:.3f}s "
        f"rate={rate:.3f}Hz gap_p99={percentile(positive, 0.99):.6f}s "
        f"max_gap={maximum:.6f}s gaps_over_{maximum_gap:.3f}s={gap_count} "
        f"nonpositive={nonpositive}",
        flush=True,
    )
    print(
        f"{label}_storage_lag: p50={percentile(storage_lag, 0.50):.6f}s "
        f"p99={percentile(storage_lag, 0.99):.6f}s "
        f"max={max(storage_lag, default=float('nan')):.6f}s",
        flush=True,
    )

    if rate < minimum_rate:
        failures.append(
            f"{label}: rate {rate:.3f} Hz is below {minimum_rate:.3f} Hz"
        )
    if maximum > maximum_gap:
        failures.append(
            f"{label}: maximum header gap {maximum:.6f}s "
            f"exceeds {maximum_gap:.6f}s"
        )
    if nonpositive:
        failures.append(
            f"{label}: {nonpositive} non-monotonic header intervals"
        )
    return failures


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag_path")
    parser.add_argument("--lidar-topic", default="/livox/lidar")
    parser.add_argument("--imu-topic", default="/livox/imu")
    parser.add_argument("--minimum-lidar-rate", type=float, default=8.0)
    parser.add_argument("--minimum-imu-rate", type=float, default=180.0)
    parser.add_argument("--maximum-lidar-gap", type=float, default=0.25)
    parser.add_argument("--maximum-imu-gap", type=float, default=0.05)
    args = parser.parse_args()

    metadata = os.path.join(args.bag_path, "metadata.yaml")
    databases = sorted(glob.glob(os.path.join(args.bag_path, "*.db3")))
    if not os.path.isfile(metadata) or os.path.getsize(metadata) == 0:
        print(f"RESULT: FAIL missing bag metadata: {metadata}", file=sys.stderr)
        return 2
    if not databases:
        print(f"RESULT: FAIL no sqlite3 database in {args.bag_path}", file=sys.stderr)
        return 2

    topics = (args.lidar_topic, args.imu_topic)
    header_stamps = {topic: [] for topic in topics}
    storage_stamps = {topic: [] for topic in topics}
    failures = []
    print(
        "mount_pose_check: SKIPPED "
        "(fast continuity mode; this is not a failure)",
        flush=True,
    )
    print(
        "chassis_self_return_check: SKIPPED "
        "(fast continuity mode; this is not a failure)",
        flush=True,
    )

    for database in databases:
        connection = sqlite3.connect(
            f"file:{database}?mode=ro&immutable=1", uri=True
        )
        try:
            quick_check = connection.execute("PRAGMA quick_check").fetchone()[0]
            if quick_check != "ok":
                failures.append(
                    f"{os.path.basename(database)}: quick_check={quick_check}"
                )
                continue
            topic_ids = {
                name: topic_id
                for topic_id, name in connection.execute(
                    "SELECT id, name FROM topics"
                )
            }
            for topic in topics:
                if topic not in topic_ids:
                    failures.append(
                        f"{os.path.basename(database)}: missing topic {topic}"
                    )
                    continue
                rows = connection.execute(
                    "SELECT timestamp, substr(data, 1, 64) FROM messages "
                    "WHERE topic_id=? ORDER BY timestamp",
                    (topic_ids[topic],),
                )
                for database_stamp, prefix in rows:
                    try:
                        header_stamp = header_stamp_from_cdr(prefix)
                    except (ValueError, struct.error) as error:
                        failures.append(f"{topic}: {error}")
                        continue
                    header_stamps[topic].append(header_stamp)
                    storage_stamps[topic].append(database_stamp * 1e-9)
        finally:
            connection.close()

    failures.extend(
        continuity(
            "lidar",
            header_stamps[args.lidar_topic],
            storage_stamps[args.lidar_topic],
            args.minimum_lidar_rate,
            args.maximum_lidar_gap,
        )
    )
    failures.extend(
        continuity(
            "imu",
            header_stamps[args.imu_topic],
            storage_stamps[args.imu_topic],
            args.minimum_imu_rate,
            args.maximum_imu_gap,
        )
    )

    if failures:
        for failure in failures:
            print(f"FAIL: {failure}", file=sys.stderr)
        print(f"RESULT: FAIL ({len(failures)} checks)", file=sys.stderr)
        return 4

    print("RESULT: PASS bag continuity")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
