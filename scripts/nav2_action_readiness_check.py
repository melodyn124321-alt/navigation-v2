#!/usr/bin/env python3
"""Check core Nav2 action servers with one persistent DDS participant."""

import argparse
import sys
import time

import rclpy
from nav2_msgs.action import (
    ComputePathThroughPoses,
    ComputePathToPose,
    DriveOnHeading,
    FollowPath,
)
from rclpy.action import ActionClient
from rclpy.node import Node


class ActionReadinessCheck(Node):
    def __init__(self):
        super().__init__("nav2_action_readiness_check")
        self.action_clients = {
            "/follow_path": ActionClient(self, FollowPath, "/follow_path"),
            "/compute_path_to_pose": ActionClient(
                self, ComputePathToPose, "/compute_path_to_pose"),
            "/compute_path_through_poses": ActionClient(
                self,
                ComputePathThroughPoses,
                "/compute_path_through_poses",
            ),
            "/drive_on_heading": ActionClient(
                self, DriveOnHeading, "/drive_on_heading"),
        }

    def wait_ready(self, timeout_sec, consecutive):
        deadline = time.monotonic() + timeout_sec
        next_report = 0.0
        ready_streak = 0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.2)
            missing = [
                name for name, client in self.action_clients.items()
                if not client.server_is_ready()
            ]
            if not missing:
                ready_streak += 1
                if ready_streak >= consecutive:
                    for name in self.action_clients:
                        print(f"{name}: READY", flush=True)
                    print(
                        "NAV2_ACTIONS_READY "
                        f"consecutive={ready_streak}",
                        flush=True,
                    )
                    return True
            else:
                ready_streak = 0
                now = time.monotonic()
                if now >= next_report:
                    print(
                        "Waiting for Nav2 actions: " + ", ".join(missing),
                        flush=True,
                    )
                    next_report = now + 2.0
        missing = [
            name for name, client in self.action_clients.items()
            if not client.server_is_ready()
        ]
        print(
            "NAV2_ACTIONS_NOT_READY missing=" + ",".join(missing),
            file=sys.stderr,
            flush=True,
        )
        return False


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--consecutive", type=int, default=3)
    args = parser.parse_args()

    rclpy.init()
    node = ActionReadinessCheck()
    try:
        return 0 if node.wait_ready(
            max(1.0, args.timeout),
            max(1, args.consecutive),
        ) else 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
