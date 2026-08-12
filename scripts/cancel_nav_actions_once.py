#!/usr/bin/env python3
"""Cancel every navigation action through one bounded ROS participant."""

import time

import rclpy
from action_msgs.srv import CancelGoal
from rclpy.node import Node


ACTIONS = (
    "/aligned_navigate_to_pose",
    "/navigate_to_pose_raw",
    "/navigate_through_poses",
    "/compute_path_to_pose",
    "/compute_path_through_poses",
    "/follow_path",
    "/drive_on_heading",
    "/backup",
    "/spin",
)


def main():
    rclpy.init()
    node = Node("cancel_nav_actions_once")
    clients = {
        action: node.create_client(
            CancelGoal, f"{action}/_action/cancel_goal")
        for action in ACTIONS
    }
    futures = {}
    deadline = time.monotonic() + 6.0
    try:
        while time.monotonic() < deadline:
            for action, client in clients.items():
                if action not in futures and client.service_is_ready():
                    futures[action] = client.call_async(CancelGoal.Request())
            rclpy.spin_once(node, timeout_sec=0.10)
            if (
                    "/aligned_navigate_to_pose" in futures
                    and all(future.done() for future in futures.values())
                    and len(futures) == len(clients)):
                break

        confirmed = 0
        for action in ACTIONS:
            future = futures.get(action)
            if future is None:
                print(f"CANCEL_UNAVAILABLE action={action}", flush=True)
                continue
            if not future.done():
                print(f"CANCEL_TIMEOUT action={action}", flush=True)
                continue
            try:
                response = future.result()
            except Exception as error:  # noqa: BLE001 - diagnostic boundary
                print(
                    f"CANCEL_FAILED action={action} error={error}",
                    flush=True,
                )
                continue
            confirmed += 1
            print(
                f"CANCEL_CONFIRMED action={action} "
                f"return_code={response.return_code} "
                f"goals={len(response.goals_canceling)}",
                flush=True,
            )

        if "/aligned_navigate_to_pose" not in futures:
            print(
                "CANCEL_RESULT FAIL aligned action service was not discovered; "
                "motion remains DISARMED and Nav2 must be restarted",
                flush=True,
            )
            return 3
        aligned_future = futures["/aligned_navigate_to_pose"]
        if not aligned_future.done():
            print(
                "CANCEL_RESULT FAIL aligned action cancellation timed out; "
                "motion remains DISARMED and Nav2 must be restarted",
                flush=True,
            )
            return 4
        print(
            f"CANCEL_RESULT PASS confirmed={confirmed}/{len(ACTIONS)} "
            "physical motion remains DISARMED",
            flush=True,
        )
        return 0
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
