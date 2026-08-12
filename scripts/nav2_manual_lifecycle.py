#!/usr/bin/env python3
"""Configure and activate Nav2 lifecycle nodes with clear per-node errors."""

import argparse
import sys
import time

import rclpy
from lifecycle_msgs.msg import State, Transition
from lifecycle_msgs.srv import ChangeState, GetState
from rclpy.node import Node


class ManualLifecycle(Node):
    def __init__(self):
        super().__init__("nav2_manual_lifecycle")

    def wait_client(self, client, name, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if client.wait_for_service(timeout_sec=1.0):
                return True
            self.get_logger().info(f"waiting for {name}")
        return False

    def call(self, client, request, timeout):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            if future.done():
                return future.result()
        raise TimeoutError("service call timed out")

    def state(self, node_name, timeout):
        client = self.create_client(GetState, f"{node_name}/get_state")
        if not self.wait_client(client, f"{node_name}/get_state", timeout):
            raise TimeoutError(f"{node_name}/get_state not available")
        response = self.call(client, GetState.Request(), timeout)
        return response.current_state.id, response.current_state.label

    def transition(self, node_name, transition_id, timeout):
        client = self.create_client(ChangeState, f"{node_name}/change_state")
        if not self.wait_client(client, f"{node_name}/change_state", timeout):
            raise TimeoutError(f"{node_name}/change_state not available")
        request = ChangeState.Request()
        request.transition.id = transition_id
        response = self.call(client, request, timeout)
        if not response.success:
            raise RuntimeError(
                f"{node_name} rejected transition id={transition_id}")

    def wait_state(self, node_name, target, timeout):
        deadline = time.monotonic() + timeout
        last = None
        while rclpy.ok() and time.monotonic() < deadline:
            state_id, label = self.state(node_name, timeout=3.0)
            last = f"{label} [{state_id}]"
            if state_id == target:
                return last
            time.sleep(0.5)
        raise TimeoutError(
            f"{node_name} did not reach state id={target}; last={last}")

    def bringup(self, nodes, timeout, post_configure_settle):
        for node_name in nodes:
            state_id, label = self.state(node_name, timeout)
            self.get_logger().info(f"{node_name}: current {label} [{state_id}]")
            if state_id == State.PRIMARY_STATE_ACTIVE:
                continue
            if state_id == State.PRIMARY_STATE_UNCONFIGURED:
                self.get_logger().info(f"{node_name}: configure")
                self.transition(
                    node_name, Transition.TRANSITION_CONFIGURE, timeout)
                self.wait_state(node_name, State.PRIMARY_STATE_INACTIVE, timeout)
                state_id = State.PRIMARY_STATE_INACTIVE
                if post_configure_settle > 0.0:
                    self.get_logger().info(
                        f"{node_name}: waiting {post_configure_settle:.1f}s "
                        "for TF and map callbacks before activation")
                    time.sleep(post_configure_settle)
            if state_id == State.PRIMARY_STATE_INACTIVE:
                self.get_logger().info(f"{node_name}: activate")
                self.transition(
                    node_name, Transition.TRANSITION_ACTIVATE, timeout)
                self.wait_state(node_name, State.PRIMARY_STATE_ACTIVE, timeout)
            final_id, final_label = self.state(node_name, timeout)
            if final_id != State.PRIMARY_STATE_ACTIVE:
                raise RuntimeError(
                    f"{node_name} final state is {final_label} [{final_id}]")
            self.get_logger().info(f"{node_name}: active [3]")

    def check_active(self, nodes, timeout):
        failed = []
        for node_name in nodes:
            try:
                state_id, label = self.state(node_name, timeout)
                print(f"/{node_name}: {label} [{state_id}]", flush=True)
                if state_id != State.PRIMARY_STATE_ACTIVE:
                    failed.append(node_name)
            except Exception as exc:
                print(f"/{node_name}: not discovered ({exc})", flush=True)
                failed.append(node_name)
        if failed:
            raise RuntimeError(
                "lifecycle nodes not active: " + ", ".join(failed))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=35.0)
    parser.add_argument("--post-configure-settle", type=float, default=4.0)
    parser.add_argument("--check-only", action="store_true")
    parser.add_argument("nodes", nargs="+")
    args = parser.parse_args()

    rclpy.init()
    node = ManualLifecycle()
    try:
        if args.check_only:
            node.check_active(args.nodes, args.timeout)
        else:
            node.bringup(
                args.nodes, args.timeout, args.post_configure_settle)
        return 0
    except Exception as exc:
        node.get_logger().error(str(exc))
        return 1
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
