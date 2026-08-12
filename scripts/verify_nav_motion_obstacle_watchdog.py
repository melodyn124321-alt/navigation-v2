#!/usr/bin/env python3
"""Verify obstacle-staleness fail-safe in an isolated ROS domain."""

import argparse
import os
import subprocess
import sys
import time
from pathlib import Path

domain_id = os.environ.get("ROS_DOMAIN_ID", "")
if not domain_id.isdigit() or int(domain_id) == 0:
    print(
        "REFUSED: run this test in an isolated non-zero ROS_DOMAIN_ID, "
        "for example: use_fastdds_env.sh env ROS_DOMAIN_ID=77 /usr/bin/python3 ...",
        file=sys.stderr,
    )
    raise SystemExit(2)

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from ranger_msgs.msg import SystemState
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class WatchdogHarness(Node):
    def __init__(self):
        super().__init__("nav_motion_obstacle_watchdog_test")
        self.command_pub = self.create_publisher(Twist, "/watchdog_test_input", 10)
        self.localization_pub = self.create_publisher(
            Odometry, "/relocalization_odom", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.system_pub = self.create_publisher(SystemState, "/system_state", 10)
        self.arm_client = self.create_client(
            SetBool, "/set_nav_motion_enabled")
        self.arm_requested = False
        self.obstacle_pub = self.create_publisher(
            String, "/nav_obstacle_cloud_status", 10)
        self.create_subscription(
            Twist, "/watchdog_test_output", self.on_output, 10)
        self.create_subscription(
            String, "/nav_motion_status", self.on_status, 10)
        self.started = time.monotonic()
        self.straight_outputs = []
        self.spin_outputs = []
        self.curve_outputs = []
        self.stale_outputs = []
        self.statuses = []
        self.last_published_age = None
        self.create_timer(0.05, self.publish_inputs)

    def on_output(self, msg):
        elapsed = time.monotonic() - self.started
        if 2.0 <= elapsed < 2.5:
            self.straight_outputs.append((msg.linear.x, msg.angular.z))
        elif 2.5 <= elapsed < 3.0:
            self.spin_outputs.append((msg.linear.x, msg.angular.z))
        elif 3.0 <= elapsed < 3.5:
            self.curve_outputs.append((msg.linear.x, msg.angular.z))
        elif elapsed >= 4.0:
            self.stale_outputs.append(abs(msg.linear.x) + abs(msg.angular.z))

    def on_status(self, msg):
        self.statuses.append(msg.data)

    def publish_inputs(self):
        elapsed = time.monotonic() - self.started
        stamp = self.get_clock().now().to_msg()

        localization = Odometry()
        localization.header.stamp = stamp
        localization.pose.covariance[0] = 0.01
        self.localization_pub.publish(localization)

        odom = Odometry()
        odom.header.stamp = stamp
        self.odom_pub.publish(odom)

        system = SystemState()
        system.header.stamp = stamp
        system.vehicle_state = SystemState.VEHICLE_STATE_NORMAL
        system.control_mode = SystemState.CONTROL_MODE_CAN
        system.error_code = 0
        self.system_pub.publish(system)

        if not self.arm_requested and self.arm_client.service_is_ready():
            request = SetBool.Request()
            request.data = True
            self.arm_client.call_async(request)
            self.arm_requested = True
        obstacle_age = 0.01 if elapsed < 3.5 else 2.0
        self.last_published_age = obstacle_age
        self.obstacle_pub.publish(String(data=(
            "ok received=100 published=100 "
            f"age={obstacle_age:.3f}s points=700->700 frame=nav_lidar"
        )))

        command = Twist()
        if 2.5 <= elapsed < 3.0:
            command.angular.z = 0.18
        elif 3.0 <= elapsed < 3.5:
            command.linear.x = 0.04
            command.angular.z = 0.18
        else:
            command.linear.x = 0.05
        self.command_pub.publish(command)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--gate",
        default=str(Path(__file__).with_name("nav_motion_safety_gate.py")),
    )
    args = parser.parse_args()
    gate = subprocess.Popen(
        [
            sys.executable, args.gate, "--ros-args",
            "-p", "input_topic:=/watchdog_test_input",
            "-p", "output_topic:=/watchdog_test_output",
            "-p", "localization_timeout:=0.50",
            "-p", "odom_timeout:=0.50",
            "-p", "chassis_timeout:=0.50",
            "-p", "obstacle_status_timeout:=0.50",
            "-p", "max_obstacle_age:=1.00",
        ],
        env=os.environ.copy(),
        stdout=subprocess.DEVNULL,
        stderr=None,
    )
    try:
        rclpy.init()
        node = WatchdogHarness()
        deadline = time.monotonic() + 6.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        straight_max = max(
            (abs(x) for x, _ in node.straight_outputs), default=0.0)
        spin_max = max(
            (abs(x) + abs(z) for x, z in node.spin_outputs),
            default=float("inf"))
        curve_linear_max = max(
            (abs(x) for x, _ in node.curve_outputs), default=0.0)
        curve_angular_max = max(
            (abs(z) for _, z in node.curve_outputs), default=float("inf"))
        stale_max = max(node.stale_outputs, default=float("inf"))
        passed = (
            len(node.straight_outputs) >= 3
            and straight_max >= 0.04
            and len(node.spin_outputs) >= 3
            and spin_max < 1.0e-4
            and len(node.curve_outputs) >= 3
            and curve_linear_max >= 0.03
            and 0.04 <= curve_angular_max <= 0.11
            and len(node.stale_outputs) >= 5
            and stale_max < 1.0e-4
        )
        print(
            "OBSTACLE_WATCHDOG_TEST "
            f"{'PASS' if passed else 'FAIL'} "
            f"straight_samples={len(node.straight_outputs)} "
            f"straight_max={straight_max:.6f} "
            f"spin_samples={len(node.spin_outputs)} spin_max={spin_max:.6f} "
            f"curve_samples={len(node.curve_outputs)} "
            f"curve_linear_max={curve_linear_max:.6f} "
            f"curve_angular_max={curve_angular_max:.6f} "
            f"stale_samples={len(node.stale_outputs)} "
            f"stale_max={stale_max:.6f} "
            f"published_age={node.last_published_age} "
            f"last_status={node.statuses[-1] if node.statuses else 'missing'}"
        )
        node.destroy_node()
        rclpy.shutdown()
        raise SystemExit(0 if passed else 1)
    finally:
        gate.terminate()
        try:
            gate.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            gate.kill()


if __name__ == "__main__":
    main()
