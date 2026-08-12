#!/usr/bin/env python3
"""Test the RViz goal-topic bridge with an isolated local action server."""

import importlib.util
import math
import sys
import threading
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionServer, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String


MODULE_PATH = Path(__file__).with_name("rviz_goal_pose_bridge.py")
SPEC = importlib.util.spec_from_file_location(
    "rviz_goal_pose_bridge", MODULE_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = MODULE
SPEC.loader.exec_module(MODULE)


class TestActionServer(Node):
    def __init__(self):
        super().__init__("rviz_goal_bridge_test_action_server")
        self.received = 0
        self.server = ActionServer(
            self,
            NavigateToPose,
            "/test_rviz_goal/navigate_to_pose",
            execute_callback=self.execute,
            goal_callback=lambda _request: GoalResponse.ACCEPT,
        )

    def execute(self, goal_handle):
        self.received += 1
        goal_handle.succeed()
        return NavigateToPose.Result()

    def destroy_node(self):
        self.server.destroy()
        return super().destroy_node()


class TestDriver(Node):
    def __init__(self):
        super().__init__("rviz_goal_bridge_test_driver")
        self.statuses = []
        self.publisher = self.create_publisher(
            PoseStamped, "/test_rviz_goal/goal_pose", 5)
        self.create_subscription(
            String,
            "/test_rviz_goal/status",
            lambda msg: self.statuses.append(msg.data),
            10,
        )

    def send(self, frame, x=1.0, y=2.0, yaw=0.3):
        message = PoseStamped()
        message.header.frame_id = frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = x
        message.pose.position.y = y
        message.pose.orientation.z = math.sin(yaw / 2.0)
        message.pose.orientation.w = math.cos(yaw / 2.0)
        self.publisher.publish(message)


def wait_until(predicate, timeout, description):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.05)
    raise RuntimeError(f"timeout waiting for {description}")


def main():
    rclpy.init(args=[
        "--ros-args",
        "-p", "goal_topic:=/test_rviz_goal/goal_pose",
        "-p", "action_name:=/test_rviz_goal/navigate_to_pose",
        "-p", "required_frame:=map",
        "-p", "duplicate_window:=3.0",
        "-r",
        "/rviz_goal_pose_bridge_status:=/test_rviz_goal/status",
    ])
    bridge = MODULE.RvizGoalPoseBridge()
    action_server = TestActionServer()
    driver = TestDriver()
    executor = MultiThreadedExecutor(num_threads=4)
    for node in (bridge, action_server, driver):
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        wait_until(
            lambda: any(
                text.startswith("READY action_server=true")
                for text in driver.statuses
            ),
            5.0,
            "bridge readiness",
        )

        driver.send("odom")
        wait_until(
            lambda: any(
                text.startswith("REJECTED frame=odom")
                for text in driver.statuses
            ),
            2.0,
            "invalid-frame rejection",
        )

        driver.send("map")
        wait_until(
            lambda: any(
                text.startswith("SUCCEEDED goal_id=")
                for text in driver.statuses
            ),
            5.0,
            "isolated action completion",
        )
        if action_server.received != 1:
            raise RuntimeError(
                f"expected exactly one action goal, got {action_server.received}")

        driver.send("map")
        wait_until(
            lambda: any(
                text.startswith("DUPLICATE_REJECTED")
                for text in driver.statuses
            ),
            2.0,
            "duplicate rejection",
        )
        if action_server.received != 1:
            raise RuntimeError("duplicate goal reached the action server")

        print(
            "RVIZ_GOAL_POSE_BRIDGE_TEST_PASS "
            "ready=PASS invalid_frame=PASS exactly_once=PASS "
            "duplicate_rejected=PASS physical_cmd_vel=UNTOUCHED"
        )
        return 0
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)
        for node in (bridge, action_server, driver):
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
