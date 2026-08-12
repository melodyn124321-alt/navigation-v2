#!/usr/bin/env python3
"""Verify that reverse is allowed only on a fresh, confirmed straight path."""

import os
import subprocess
import sys
import time
from pathlib import Path

domain_id = os.environ.get("ROS_DOMAIN_ID", "")
if not domain_id.isdigit() or int(domain_id) == 0:
    print("REFUSED: use an isolated non-zero ROS_DOMAIN_ID", file=sys.stderr)
    raise SystemExit(2)

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path as NavPath
from ranger_msgs.msg import SystemState
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool


class ReversePolicyHarness(Node):
    def __init__(self):
        super().__init__("nav_reverse_path_policy_test")
        self.cmd_pub = self.create_publisher(Twist, "/reverse_policy_input", 10)
        self.plan_pub = self.create_publisher(NavPath, "/reverse_policy_plan", 10)
        self.public_plan_noise_pub = self.create_publisher(
            NavPath, "/plan", 10)
        self.localization_pub = self.create_publisher(
            Odometry, "/relocalization_odom", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)
        self.system_pub = self.create_publisher(SystemState, "/system_state", 10)
        self.obstacle_pub = self.create_publisher(
            String, "/nav_obstacle_cloud_status", 10)
        self.arm_client = self.create_client(SetBool, "/set_nav_motion_enabled")
        self.create_subscription(
            Twist, "/reverse_policy_output", self.on_output, 10)
        self.started = time.monotonic()
        self.arm_requested = False
        self.curved_forward = []
        self.curved_reverse = []
        self.straight_reverse = []
        self.stale_reverse = []
        self.create_timer(0.05, self.publish_inputs)

    def on_output(self, msg):
        elapsed = time.monotonic() - self.started
        value = (float(msg.linear.x), float(msg.angular.z))
        if 1.5 <= elapsed < 2.5:
            self.curved_forward.append(value)
        elif 2.7 <= elapsed < 3.5:
            self.curved_reverse.append(value)
        elif 4.0 <= elapsed < 5.0:
            self.straight_reverse.append(value)
        elif elapsed >= 5.8:
            self.stale_reverse.append(value)

    @staticmethod
    def make_plan(curved):
        path = NavPath()
        path.header.frame_id = "map"
        coordinates = (
            [(0.0, 0.0), (0.35, 0.0), (0.65, 0.18), (0.85, 0.50)]
            if curved else
            [(0.0, 0.0), (-0.30, 0.0), (-0.60, 0.0), (-0.90, 0.0)]
        )
        for x, y in coordinates:
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        return path

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
        system.control_mode = SystemState.CONTROL_MODE_CAN
        system.error_code = 0
        self.system_pub.publish(system)
        self.obstacle_pub.publish(String(
            data="ok received=100 published=100 age=0.010s points=700->700"))

        if not self.arm_requested and self.arm_client.service_is_ready():
            request = SetBool.Request()
            request.data = True
            self.arm_client.call_async(request)
            self.arm_requested = True

        if 0.5 <= elapsed < 3.5:
            self.plan_pub.publish(self.make_plan(curved=True))
        elif 3.5 <= elapsed < 5.0:
            self.plan_pub.publish(self.make_plan(curved=False))
            # Reproduce the production failure: an unrelated Nav2 planner is
            # publishing a curved global path while the dedicated reverse
            # authorization remains straight. The safety gate must ignore it.
            self.public_plan_noise_pub.publish(self.make_plan(curved=True))

        command = Twist()
        if elapsed < 2.5:
            command.linear.x = 0.05
            command.angular.z = 0.05
        else:
            command.linear.x = -0.04
        self.cmd_pub.publish(command)


def maximum_abs(samples):
    return max((abs(linear) + abs(angular) for linear, angular in samples),
               default=float("inf"))


def main():
    gate_path = str(Path(__file__).with_name("nav_motion_safety_gate.py"))
    gate = subprocess.Popen(
        [
            sys.executable,
            gate_path,
            "--ros-args",
            "-p", "input_topic:=/reverse_policy_input",
            "-p", "output_topic:=/reverse_policy_output",
            "-p", "plan_topic:=/reverse_policy_plan",
            "-p", "plan_timeout:=0.60",
            "-p", "straight_path_confirmations_required:=2",
            "-p", "ultrasonic_enabled:=false",
            "-p", "operator_heartbeat_required:=false",
            "-p", "goal_lease_required:=false",
            "-p", "localization_timeout:=0.50",
            "-p", "odom_timeout:=0.50",
            "-p", "chassis_timeout:=0.50",
            "-p", "obstacle_status_timeout:=0.50",
        ],
        env=os.environ.copy(),
        stdout=subprocess.DEVNULL,
    )
    try:
        rclpy.init()
        node = ReversePolicyHarness()
        deadline = time.monotonic() + 7.0
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        curved_forward_max = max(
            (linear for linear, _ in node.curved_forward), default=0.0)
        curved_reverse_max = maximum_abs(node.curved_reverse)
        straight_reverse_min = min(
            (linear for linear, _ in node.straight_reverse), default=0.0)
        stale_reverse_max = maximum_abs(node.stale_reverse)
        passed = (
            len(node.curved_forward) >= 4
            and curved_forward_max >= 0.04
            and len(node.curved_reverse) >= 4
            and curved_reverse_max < 1.0e-4
            and len(node.straight_reverse) >= 4
            and straight_reverse_min <= -0.03
            and len(node.stale_reverse) >= 4
            and stale_reverse_max < 1.0e-4
        )
        print(
            f"REVERSE_PATH_POLICY_TEST {'PASS' if passed else 'FAIL'} "
            f"curved_forward_samples={len(node.curved_forward)} "
            f"curved_forward_max={curved_forward_max:.4f} "
            f"curved_reverse_samples={len(node.curved_reverse)} "
            f"curved_reverse_max={curved_reverse_max:.6f} "
            f"straight_reverse_samples={len(node.straight_reverse)} "
            f"straight_reverse_min={straight_reverse_min:.4f} "
            "competing_public_plan_ignored="
            f"{'PASS' if straight_reverse_min <= -0.03 else 'FAIL'} "
            f"stale_reverse_samples={len(node.stale_reverse)} "
            f"stale_reverse_max={stale_reverse_max:.6f}"
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
