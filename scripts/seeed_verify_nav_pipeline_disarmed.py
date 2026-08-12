#!/usr/bin/env python3
"""Verify the core Nav2 command chain while the physical gate is disarmed.

The operator-facing /navigate_to_pose adapter intentionally rejects disarmed
goals. This probe therefore targets its internal /navigate_through_poses action
and proves planner -> controller -> smoother -> collision monitor -> safety
gate behavior without allowing a physical chassis command.
"""

import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateThroughPoses
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from tf2_ros import Buffer, TransformException, TransformListener


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )


class DisarmedPipelineProbe(Node):
    def __init__(self):
        super().__init__("disarmed_nav_pipeline_probe")
        latched = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.client = ActionClient(
            self, NavigateThroughPoses, "/navigate_through_poses")
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.armed = None
        self.statuses = []
        self.pre_gate_nonzero = []
        self.physical_nonzero = []
        self.create_subscription(
            Bool, "/nav_motion_armed", self.on_armed, 10)
        self.create_subscription(
            String, "/aligned_goal_status", self.on_status, latched)
        self.create_subscription(
            Twist,
            "/cmd_vel_nav_collision_safe",
            self.on_pre_gate_command,
            10,
        )
        self.create_subscription(
            Twist, "/cmd_vel", self.on_physical_command, 10)

    def on_armed(self, message):
        self.armed = bool(message.data)

    def on_status(self, message):
        text = message.data.strip()
        if not self.statuses or text != self.statuses[-1]:
            self.statuses.append(text)

    def on_pre_gate_command(self, message):
        if abs(message.linear.x) > 0.001 or abs(message.angular.z) > 0.001:
            self.pre_gate_nonzero.append(
                (float(message.linear.x), float(message.angular.z)))

    def on_physical_command(self, message):
        if abs(message.linear.x) > 0.001 or abs(message.angular.z) > 0.001:
            self.physical_nonzero.append(
                (float(message.linear.x), float(message.angular.z)))

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.20)
            if predicate():
                return True
        return bool(predicate())

    def current_pose(self, timeout):
        deadline = time.monotonic() + timeout
        last_error = None
        while rclpy.ok() and time.monotonic() < deadline:
            try:
                return self.tf_buffer.lookup_transform(
                    "map",
                    "base_link",
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.20),
                )
            except TransformException as error:
                last_error = error
                rclpy.spin_once(self, timeout_sec=0.20)
        raise RuntimeError(
            f"map->base_link unavailable: {last_error}")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--distance", type=float, default=0.40)
    parser.add_argument("--timeout", type=float, default=20.0)
    arguments = parser.parse_args()

    rclpy.init()
    node = DisarmedPipelineProbe()
    handle = None
    try:
        if not node.wait_until(
                lambda: node.armed is not None, 8.0):
            print("PIPELINE_FAIL no /nav_motion_armed state", flush=True)
            return 2
        if node.armed:
            print(
                "PIPELINE_REFUSED motion gate is ARMED; disarm first",
                flush=True,
            )
            return 3
        if not node.wait_until(
                node.client.server_is_ready, 25.0):
            print(
                "PIPELINE_FAIL /navigate_through_poses unavailable",
                flush=True,
            )
            return 4

        transform = node.current_pose(12.0)
        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        yaw = yaw_from_quaternion(quaternion)

        goal = NavigateThroughPoses.Goal()
        target = PoseStamped()
        target.header.frame_id = "map"
        target.header.stamp = node.get_clock().now().to_msg()
        target.pose.position.x = (
            translation.x + arguments.distance * math.cos(yaw))
        target.pose.position.y = (
            translation.y + arguments.distance * math.sin(yaw))
        target.pose.orientation = quaternion
        goal.poses = [target]
        print(
            "PIPELINE_GOAL "
            f"start=({translation.x:.3f},{translation.y:.3f},{yaw:.3f}) "
            f"goal=({target.pose.position.x:.3f},"
            f"{target.pose.position.y:.3f},{yaw:.3f})",
            flush=True,
        )

        send_future = node.client.send_goal_async(goal)
        node.wait_until(send_future.done, 10.0)
        handle = send_future.result() if send_future.done() else None
        if handle is None or not handle.accepted:
            print("PIPELINE_FAIL goal rejected", flush=True)
            return 5
        print("PIPELINE_ACCEPTED", flush=True)

        def planner_chain_reached():
            return bool(node.pre_gate_nonzero)

        node.wait_until(planner_chain_reached, arguments.timeout)
        cancel_future = handle.cancel_goal_async()
        node.wait_until(cancel_future.done, 8.0)
        node.wait_until(
            lambda: any(status.startswith("CANCELED")
                        for status in node.statuses),
            5.0,
        )

        for status in node.statuses[-24:]:
            print(f"STATUS {status}", flush=True)
        print(
            "PIPELINE_PRE_GATE_NONZERO="
            f"{'YES' if node.pre_gate_nonzero else 'NO'}",
            flush=True,
        )
        if node.pre_gate_nonzero:
            linear, angular = node.pre_gate_nonzero[-1]
            print(
                f"PIPELINE_PRE_GATE_SAMPLE "
                f"linear.x={linear:.3f} angular.z={angular:.3f}",
                flush=True,
            )
        print(
            "PIPELINE_PHYSICAL_NONZERO="
            f"{'YES' if node.physical_nonzero else 'NO'}",
            flush=True,
        )

        if node.physical_nonzero:
            print(
                "PIPELINE_FAIL physical /cmd_vel was nonzero while disarmed",
                flush=True,
            )
            return 6
        if not planner_chain_reached():
            print(
                "PIPELINE_FAIL planner/controller command chain incomplete",
                flush=True,
            )
            return 7
        print(
            "PIPELINE_PASS inner goal accepted, forward plan executed, "
            "pre-gate command generated, physical output remained zero",
            flush=True,
        )
        return 0
    finally:
        if handle is not None and handle.status in (1, 2, 3):
            handle.cancel_goal_async()
        node.client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
