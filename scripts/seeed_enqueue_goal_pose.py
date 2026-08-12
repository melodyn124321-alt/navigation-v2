#!/usr/bin/env python3
"""Atomically enqueue one HN RViz goal for the persistent Seeed bridge."""

import argparse
import json
import math
import os
from pathlib import Path
import sys
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--x", type=float, required=True)
    parser.add_argument("--y", type=float, required=True)
    parser.add_argument("--z", type=float, default=0.0)
    parser.add_argument("--qx", type=float, default=0.0)
    parser.add_argument("--qy", type=float, default=0.0)
    parser.add_argument("--qz", type=float, required=True)
    parser.add_argument("--qw", type=float, required=True)
    parser.add_argument("--frame", default="map")
    parser.add_argument("--created-at-ns", type=int, required=True)
    parser.add_argument("--source-sequence", type=int, default=0)
    parser.add_argument("--max-age-sec", type=float, default=10.0)
    parser.add_argument(
        "--inbox",
        default="/home/seeed/ros2/logs/hn_goal_pose_inbox.json")
    args = parser.parse_args()

    values = (args.x, args.y, args.z, args.qx, args.qy, args.qz, args.qw)
    norm = math.sqrt(sum(value * value for value in values[3:]))
    if not all(math.isfinite(value) for value in values):
        print("GOAL_INBOX_REJECTED non-finite pose", file=sys.stderr)
        return 2
    if args.frame != "map" or norm < 0.5:
        print("GOAL_INBOX_REJECTED invalid frame/orientation", file=sys.stderr)
        return 2

    now_ns = time.time_ns()
    age_sec = (now_ns - args.created_at_ns) / 1e9
    if age_sec > args.max_age_sec or age_sec < -5.0:
        print(
            "GOAL_INBOX_REJECTED stale/future goal "
            f"age={age_sec:.3f}s max={args.max_age_sec:.3f}s",
            file=sys.stderr,
        )
        return 3

    inbox = Path(args.inbox)
    inbox.parent.mkdir(parents=True, exist_ok=True)
    temporary = inbox.with_name(f".{inbox.name}.{os.getpid()}.tmp")
    payload = {
        "frame": "map",
        "x": args.x,
        "y": args.y,
        "z": args.z,
        "qx": args.qx / norm,
        "qy": args.qy / norm,
        "qz": args.qz / norm,
        "qw": args.qw / norm,
        "created_at_ns": args.created_at_ns,
        "received_at_ns": now_ns,
        "source_sequence": args.source_sequence,
    }
    temporary.write_text(json.dumps(payload), encoding="utf-8")
    os.replace(temporary, inbox)
    print(f"GOAL_INBOX_QUEUED target=({args.x:.3f},{args.y:.3f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
