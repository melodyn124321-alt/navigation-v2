#!/usr/bin/env python3
"""Verify DriveOnHeading terminal control while physical motion is disarmed."""

import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav2_msgs.action import DriveOnHeading, Spin
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import Bool


class TerminalDriveProbe(Node):
    def __init__(self):
        super().__init__("terminal_drive_disarmed_probe")
        self.client = ActionClient(
            self, DriveOnHeading, "/drive_on_heading")
        self.spin_client = ActionClient(self, Spin, "/spin")
        self.armed = None
        self.pre_gate_nonzero = []
        self.spin_pre_gate = []
        self.physical_nonzero = []
        self.create_subscription(
            Bool, "/nav_motion_armed", self.on_armed, 10)
        self.create_subscription(
            Twist,
            "/cmd_vel_nav_collision_safe",
            self.on_pre_gate,
            10,
        )
        self.create_subscription(
            Twist, "/cmd_vel", self.on_physical, 10)

    def on_armed(self, message):
        self.armed = bool(message.data)

    def on_pre_gate(self, message):
        if abs(message.linear.x) > 0.001 or abs(message.angular.z) > 0.001:
            self.pre_gate_nonzero.append(
                (float(message.linear.x), float(message.angular.z)))
        if abs(message.angular.z) > 0.001:
            self.spin_pre_gate.append(float(message.angular.z))

    def on_physical(self, message):
        if abs(message.linear.x) > 0.001 or abs(message.angular.z) > 0.001:
            self.physical_nonzero.append(
                (float(message.linear.x), float(message.angular.z)))

    def wait_until(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.10)
            if predicate():
                return True
        return bool(predicate())


def main():
    rclpy.init()
    node = TerminalDriveProbe()
    handle = None
    spin_handle = None
    try:
        if not node.wait_until(lambda: node.armed is not None, 8.0):
            print("TERMINAL_TEST_FAIL no /nav_motion_armed", flush=True)
            return 2
        if node.armed:
            print(
                "TERMINAL_TEST_REFUSED motion gate is ARMED",
                flush=True,
            )
            return 3
        if not node.wait_until(node.client.server_is_ready, 20.0):
            print(
                "TERMINAL_TEST_FAIL /drive_on_heading unavailable",
                flush=True,
            )
            return 4

        goal = DriveOnHeading.Goal()
        goal.target.x = 0.12
        goal.speed = 0.03
        goal.time_allowance.sec = 3
        print(
            "TERMINAL_TEST_GOAL distance=0.120m speed=0.030m/s "
            "physical_gate=DISARMED",
            flush=True,
        )
        send_future = node.client.send_goal_async(goal)
        if not node.wait_until(send_future.done, 8.0):
            print("TERMINAL_TEST_FAIL send timeout", flush=True)
            return 5
        handle = send_future.result()
        if handle is None or not handle.accepted:
            print("TERMINAL_TEST_FAIL goal rejected", flush=True)
            return 6
        print("TERMINAL_TEST_ACCEPTED", flush=True)

        result_future = handle.get_result_async()
        node.wait_until(
            lambda: bool(node.pre_gate_nonzero) or result_future.done(),
            5.0,
        )
        node.wait_until(result_future.done, 5.0)
        if not result_future.done():
            cancel_future = handle.cancel_goal_async()
            node.wait_until(cancel_future.done, 5.0)
        node.wait_until(lambda: node.armed is False, 2.0)

        print(
            "TERMINAL_PRE_GATE_NONZERO="
            f"{'YES' if node.pre_gate_nonzero else 'NO'}",
            flush=True,
        )
        if node.pre_gate_nonzero:
            linear, angular = node.pre_gate_nonzero[-1]
            print(
                "TERMINAL_PRE_GATE_SAMPLE "
                f"linear.x={linear:.3f} angular.z={angular:.3f}",
                flush=True,
            )
        print(
            "TERMINAL_PHYSICAL_NONZERO="
            f"{'YES' if node.physical_nonzero else 'NO'}",
            flush=True,
        )
        if node.armed is not False:
            print("TERMINAL_TEST_FAIL gate state changed", flush=True)
            return 7
        if node.physical_nonzero:
            print(
                "TERMINAL_TEST_FAIL physical output was nonzero",
                flush=True,
            )
            return 8
        if not node.pre_gate_nonzero:
            print(
                "TERMINAL_TEST_FAIL terminal command chain produced no command",
                flush=True,
            )
            return 9
        if not node.wait_until(node.spin_client.server_is_ready, 15.0):
            print("TERMINAL_TEST_FAIL /spin unavailable", flush=True)
            return 10
        node.spin_pre_gate.clear()
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = 0.10
        spin_goal.time_allowance.sec = 3
        spin_future = node.spin_client.send_goal_async(spin_goal)
        if not node.wait_until(spin_future.done, 8.0):
            print("TERMINAL_TEST_FAIL spin send timeout", flush=True)
            return 11
        spin_handle = spin_future.result()
        if spin_handle is None or not spin_handle.accepted:
            print("TERMINAL_TEST_FAIL bounded spin rejected", flush=True)
            return 12
        spin_result = spin_handle.get_result_async()
        node.wait_until(
            lambda: bool(node.spin_pre_gate) or spin_result.done(),
            5.0,
        )
        if not spin_result.done():
            spin_cancel = spin_handle.cancel_goal_async()
            node.wait_until(spin_cancel.done, 5.0)
        print(
            "TERMINAL_SPIN_PRE_GATE_NONZERO="
            f"{'YES' if node.spin_pre_gate else 'NO'}",
            flush=True,
        )
        if node.spin_pre_gate:
            print(
                f"TERMINAL_SPIN_SAMPLE angular.z="
                f"{node.spin_pre_gate[-1]:.3f}",
                flush=True,
            )
        print(
            "TERMINAL_SPIN_PHYSICAL_NONZERO="
            f"{'YES' if node.physical_nonzero else 'NO'}",
            flush=True,
        )
        if not node.spin_pre_gate or node.physical_nonzero:
            print(
                "TERMINAL_TEST_FAIL bounded spin command chain failed",
                flush=True,
            )
            return 13
        print(
            "TERMINAL_TEST_PASS drive_on_heading and bounded spin generated "
            "collision-checked pre-gate commands while physical /cmd_vel "
            "remained zero",
            flush=True,
        )
        return 0
    finally:
        if handle is not None and handle.status in (1, 2, 3):
            handle.cancel_goal_async()
        if spin_handle is not None and spin_handle.status in (1, 2, 3):
            spin_handle.cancel_goal_async()
        node.client.destroy()
        node.spin_client.destroy()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
